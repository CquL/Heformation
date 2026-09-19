#!/usr/bin/env python3
"""One figure for the whole mission chain: allocation -> dispatch -> flight -> control.

Panels
  task layer      the 30 s qualification window, then per task the planned window
                  at dispatch, the actual window, the dispatch instant and the
                  completion event that fed planRepair
  allocation      which executor took which task, with which members, endpoint,
                  reference-adoption verdict and repair bookkeeping
  path layer      the seven flown trajectories, the published formation centres,
                  the formation as held, and the declared obstacle box
  control layer   per-member distance to its slot, with the hold phases marked,
                  because that is where the completion threshold applies
  safety layer    altitude against the AIR floor, and the actual clearances
  time layer      qn model clock against the ROS clock, with the verdicts

Everything is read back from the recorded bag plus metrics.json/config.json.
Nothing is re-simulated: if a number is not in the recording it is not plotted.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import numpy as np  # noqa: E402

AGENTS = tuple(range(7))
PLATFORM_RADIUS_M = 0.25
PLATFORM_DIAMETER_M = 2.0 * PLATFORM_RADIUS_M
AIR_FLOOR_M = 0.085
COLOURS = plt.get_cmap("tab10")


def load_json(path):
    return json.loads(Path(path).read_text())


def agent_of(topic):
    return int(topic.split("/")[1].split("_")[1])


def read_bag(bag_path):
    """Return the recorded flight.  Plain lists and dicts so it stays sortable."""
    import rosbag

    data = {"odometry": {a: [] for a in AGENTS},
            "reference": {a: [] for a in AGENTS},
            "diagnostics": {a: [] for a in AGENTS},
            "centres": [],
            "action_goals": []}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _stamp in bag.read_messages():
            if topic.endswith("_qn/odometry"):
                point = message.pose.pose.position
                data["odometry"][agent_of(topic)].append(
                    (message.header.stamp.to_sec(), point.x, point.y, point.z))
            elif topic.endswith("_qn/used_reference_pose"):
                point = message.pose.position
                data["reference"][agent_of(topic)].append(
                    (message.header.stamp.to_sec(), point.x, point.y, point.z))
            elif topic.endswith("_qn/diagnostics"):
                values = {entry.key: entry.value
                          for status in message.status for entry in status.values}
                if "model_time_s" not in values:
                    continue
                data["diagnostics"][agent_of(topic)].append({
                    "stamp": message.header.stamp.to_sec(),
                    "model_time_s": float(values["model_time_s"]),
                    "position": (float(values["position_x"]), float(values["position_y"]),
                                 float(values["position_z"])),
                    "medium_flag": float(values.get("medium_flag", 0.0)),
                    "used_reference_valid": values.get("used_reference_valid") == "true",
                })
            elif topic == "/move_base_simple/goal":
                point = message.pose.position
                data["centres"].append(
                    (message.header.stamp.to_sec(), point.x, point.y, point.z))
            elif topic == "/formation_action/goal":
                point = message.goal.formation_center.point
                data["action_goals"].append(
                    (message.header.stamp.to_sec(), message.goal.task_id,
                     point.x, point.y, point.z))
    for series in list(data["odometry"].values()) + list(data["reference"].values()):
        series.sort(key=lambda row: row[0])
    for series in data["diagnostics"].values():
        series.sort(key=lambda row: row["stamp"])
    data["centres"].sort(key=lambda row: row[0])
    data["action_goals"].sort(key=lambda row: row[0])
    return data


def sample(rows, stamp):
    """Nearest recorded row at or before ``stamp`` (falls back to the first row)."""
    index = int(np.searchsorted([row[0] for row in rows], stamp, side="right")) - 1
    return rows[min(max(index, 0), len(rows) - 1)]


def slot_target(data, monitor, centre, agent):
    slot = monitor["relative_slots"][str(agent)]
    scale = float(monitor["swarm_scale"])
    return (centre[0] + scale * slot[0], centre[1] + scale * slot[1],
            centre[2] + scale * slot[2])


def slot_errors(data, monitor, mission_epoch):
    """Per-member distance from the actual state to the slot it was sent to."""
    stamps = [row[0] for row in data["centres"]]
    out = {}
    for agent in AGENTS:
        series = []
        for stamp, x, y, z in data["odometry"][agent]:
            index = int(np.searchsorted(stamps, stamp, side="right")) - 1
            centre = data["centres"][index][1:4] if index >= 0 else monitor["start_center"]
            series.append((stamp - mission_epoch,
                           math.dist((x, y, z), slot_target(data, monitor, centre, agent))))
        out[agent] = series
    return out


def tracking_errors(data, mission_epoch):
    """Distance between the flown state and the reference qn actually adopted."""
    out = {}
    for agent in AGENTS:
        reference = data["reference"][agent]
        stamps = [row[0] for row in reference]
        series = []
        for stamp, x, y, z in data["odometry"][agent]:
            if not stamps:
                break
            index = int(np.searchsorted(stamps, stamp, side="left"))
            index = min(max(index, 0), len(reference) - 1)
            series.append((stamp - mission_epoch,
                           math.dist((x, y, z), reference[index][1:4])))
        out[agent] = series
    return out


def min_inter_agent(odometry, stamp):
    positions = []
    for agent in AGENTS:
        rows = odometry[agent]
        if not rows:
            return None
        positions.append(sample(rows, stamp)[1:4])
    if len(positions) < 2:
        return None
    return min(math.dist(a, b) for index, a in enumerate(positions)
               for b in positions[index + 1:])


def box_signed_distance(point, centre, size):
    """Signed distance from a point to an axis-aligned box (negative inside)."""
    dx = abs(point[0] - centre[0]) - size[0] / 2.0
    dy = abs(point[1] - centre[1]) - size[1] / 2.0
    dz = abs(point[2] - centre[2]) - size[2] / 2.0
    outside = math.sqrt(max(dx, 0.0) ** 2 + max(dy, 0.0) ** 2 + max(dz, 0.0) ** 2)
    return outside + min(max(dx, dy, dz), 0.0)


def distinct_ids(value):
    if isinstance(value, dict):
        value = value.values()
    items = sorted({item for item in (value or []) if item is not None})
    return "[" + ",".join(str(item) for item in items) + "]"


def verdict_of(execution):
    verdict = execution.get("verdict") or {}
    return (verdict.get("task_outcome", "?"), verdict.get("safety_outcome", "?"),
            verdict.get("experiment_validity", "?"))


def hold_window(execution):
    hold = float(execution.get("model_time_hold_seconds") or 5.0)
    return execution["actual_finish"] - hold, execution["actual_finish"]


def panel_task(axis, metrics, executions):
    epoch = metrics["mission_epoch"]
    tolerance = float(metrics.get("delay_tolerance", 0.1))
    baseline = metrics.get("baseline_qualification") or {}
    if baseline.get("duration_s"):
        axis.axvspan(baseline["duration_s"] * -1.0, 0.0, color="#eef2f7", zorder=0)
        axis.text(baseline["duration_s"] * -0.5, 0.02, "qualification\n{:.0f} s".format(
            baseline["duration_s"]), transform=axis.get_xaxis_transform(),
            ha="center", va="bottom", fontsize=8, color="0.45")
    for index, execution in enumerate(executions):
        y = len(executions) - index
        planned_start = execution["planned_start_at_dispatch"]
        planned_finish = execution["planned_finish_at_dispatch"]
        actual_start = execution["actual_start"]
        actual_finish = execution["actual_finish"]
        dispatch = execution["goal_dispatch_ros_time_s"] - epoch
        axis.barh(y + 0.17, planned_finish - planned_start, left=planned_start,
                  height=0.26, facecolor="none", edgecolor="0.45", linewidth=1.3,
                  label="planned at dispatch" if index == 0 else None)
        axis.barh(y - 0.17, actual_finish - actual_start, left=actual_start,
                  height=0.26, color="#3b7dd8", label="actual" if index == 0 else None)
        hold_start, hold_end = hold_window(execution)
        axis.barh(y - 0.17, hold_end - hold_start, left=hold_start, height=0.26,
                  color="#12457f", label="hold" if index == 0 else None)
        axis.plot([dispatch], [y], marker="v", color="#c0392b", markersize=9,
                  linestyle="none", label="dispatch" if index == 0 else None)
        axis.plot([actual_finish], [y - 0.17], marker="o", color="#1f6f3f", markersize=7,
                  linestyle="none",
                  label="completion -> DelayEvent" if index == 0 else None)
        if actual_finish > planned_finish + tolerance:
            axis.annotate("+{:.1f} s late".format(actual_finish - planned_finish),
                          (actual_finish, y + 0.30), fontsize=8, color="#c0392b")
        text = execution["task_id"]
        if execution.get("plan_updated"):
            text += "  [repaired -> rev {}]".format(
                execution.get("plan_revision_after_completion", "?"))
        axis.text(-0.5, y, text, ha="right", va="center", fontsize=9)
    axis.axvline(0.0, color="0.55", linewidth=0.9, linestyle=":")
    axis.set_yticks([])
    axis.set_ylim(0.4, len(executions) + 1.05)
    right = max(float(execution["actual_finish"]) for execution in executions)
    axis.set_xlim(-baseline.get("duration_s", 0.0) * 1.1, right * 1.06)
    axis.set_xlabel("time since mission epoch [s]")
    axis.set_title("task layer: planned vs actual, dispatch, hold and the completion event")
    axis.grid(axis="x", alpha=0.3)
    axis.legend(fontsize=8, loc="lower right", ncol=3, framealpha=0.95)


def panel_allocation(axis, metrics, executions, config):
    axis.axis("off")
    lines = ["mode {mode}   speed {speed} m/s   repair {repair}   obstacle {obs}".format(
        mode=metrics.get("mode"), speed=metrics.get("planner_speed_mps"),
        repair=metrics.get("repair_mode"),
        obs="on" if metrics.get("obstacle_scenario") else "off"), ""]
    for execution in executions:
        task, safety, validity = verdict_of(execution)
        assigned = execution.get("assigned_members") or []
        executed = execution.get("executed_members") or []
        before = execution.get("pre_dispatch_qn_source_trajectory_ids") or {}
        adoption = (execution.get("adoption_verdict") or {}).get("state", "?")
        lines.append("{}  {} -> {}  {}".format(
            execution["task_id"], execution["execution_id"],
            execution.get("executor_id"), execution.get("action_endpoint")))
        lines.append("    members {}/{} {}   traj qn {}->{}  {}".format(
            len(executed), len(assigned),
            "match" if execution.get("executor_matches_execution") else "MISMATCH",
            distinct_ids(before), distinct_ids(execution.get("trajectory_ids")), adoption))
        lines.append("    {}/{}/{}  lag {:.3f}s  {}".format(
            task, safety, validity, float(execution.get("release_lag_s") or 0.0),
            "plan repaired" if execution.get("plan_updated") else "no repair needed"))
        lines.append("")
    test_c = metrics.get("test_c") or {}
    lines.append("plan_revision {}   plan_updated {}   updated_plan_used {}".format(
        test_c.get("plan_revision"), test_c.get("plan_updated"),
        test_c.get("updated_plan_used")))
    lines.append("dispatch_changed {} (serial single resource: expected)".format(
        test_c.get("dispatch_changed")))
    lines.append("")
    lines.append("mission {}   planned makespan {:.1f} s   actual {:.1f} s".format(
        metrics.get("status"), float(metrics.get("planned_makespan") or 0.0),
        float(metrics.get("actual_makespan") or 0.0)))
    axis.text(0.0, 1.02, "\n".join(lines), transform=axis.transAxes, fontsize=8,
              va="top", ha="left", family="monospace")
    axis.set_title("allocation: which resource took which task")


def panel_path(axis, data, metrics, monitor, executions):
    for agent in AGENTS:
        rows = data["odometry"][agent]
        if not rows:
            continue
        axis.plot([row[1] for row in rows], [row[2] for row in rows], linewidth=1.0,
                  alpha=0.85, color=COLOURS(agent % 10), label="drone_{}".format(agent))
    for stamp, task, x, y, _z in data["action_goals"]:
        axis.plot(x, y, marker="*", color="k", markersize=14, linestyle="none")
        axis.annotate(task, (x, y), textcoords="offset points", xytext=(0, 13),
                      ha="center", fontsize=10, fontweight="bold")
        axis.annotate("({:.0f}, {:.0f})".format(x, y), (x, y),
                      textcoords="offset points", xytext=(0, -19), ha="center",
                      fontsize=8, color="0.3")
    for index, (_stamp, task, x, y, z) in enumerate(data["action_goals"]):
        targets = [slot_target(data, monitor, (x, y, z), agent) for agent in AGENTS]
        loop = list(range(len(AGENTS))) + [0]
        axis.plot([targets[a][0] for a in loop], [targets[a][1] for a in loop],
                  color=COLOURS((index + 3) % 10), linewidth=1.3, linestyle="--",
                  alpha=0.9, label="{} slots".format(task))
    if metrics.get("obstacle_scenario") and metrics.get("obstacle_center"):
        centre = metrics["obstacle_center"]
        size = metrics["obstacle_size"]
        axis.add_patch(Rectangle((centre[0] - size[0] / 2.0, centre[1] - size[1] / 2.0),
                                 size[0], size[1], facecolor="#c0392b", alpha=0.3,
                                 edgecolor="#c0392b", linewidth=1.5,
                                 label="declared obstacle"))
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("path layer: the seven flown trajectories and the slot layout "
                   "each task asked for")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=6.5, ncol=5, loc="upper center", framealpha=0.9)


def panel_slot_error(axis, errors, metrics, executions):
    epsilon_p = float(metrics["monitor_epsilon_p"])
    for agent in AGENTS:
        series = errors[agent]
        axis.plot([row[0] for row in series], [row[1] for row in series], linewidth=0.8,
                  alpha=0.8, color=COLOURS(agent % 10))
    for index, execution in enumerate(executions):
        start, finish = hold_window(execution)
        axis.axvspan(start, finish, color="#cfe0f5", zorder=0,
                     label="hold (threshold applies here)" if index == 0 else None)
        final = float((execution.get("verdict") or {}).get("metrics", {})
                      .get("final_slot_error_m") or 0.0)
        axis.annotate("{}\n{:.3f} m".format(execution["task_id"], final),
                      ((start + finish) / 2.0, final + 0.15), ha="center", fontsize=8,
                      color="#12457f")
    axis.axhline(epsilon_p, color="#c0392b", linestyle="--", linewidth=1.2,
                 label="epsilon_p = {:.2f} m".format(epsilon_p))
    axis.set_ylabel("slot error [m]")
    axis.set_title("control layer: distance to the commanded slot\n"
                   "(gated by epsilon_p only while holding)")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=7, loc="upper left")
    axis.set_xlim(right=max(float(execution["actual_finish"]) for execution in executions) * 1.05)


def panel_tracking_error(axis, tracking, executions):
    worst = 0.0
    for agent in AGENTS:
        series = tracking[agent]
        if not series:
            continue
        axis.plot([row[0] for row in series], [row[1] for row in series], linewidth=0.8,
                  alpha=0.8, color=COLOURS(agent % 10))
        worst = max(worst, max(row[1] for row in series))
    for execution in executions:
        axis.axvspan(*hold_window(execution), color="#cfe0f5", zorder=0)
    axis.annotate("max {:.2f} m".format(worst), (0.98, 0.92), xycoords="axes fraction",
                  ha="right", va="top", fontsize=8, color="#c0392b")
    axis.set_xlabel("time since mission epoch [s]")
    axis.set_ylabel("tracking error [m]")
    axis.set_title("distance to the reference qn actually adopted")
    axis.grid(alpha=0.3)
    axis.set_xlim(right=max(float(execution["actual_finish"]) for execution in executions) * 1.05)


def panel_altitude(axis, data, metrics):
    epoch = metrics["mission_epoch"]
    for agent in AGENTS:
        rows = data["diagnostics"][agent]
        if not rows:
            continue
        axis.plot([row["stamp"] - epoch for row in rows],
                  [row["position"][2] for row in rows], linewidth=0.7, alpha=0.8,
                  color=COLOURS(agent % 10))
    floor = AIR_FLOOR_M
    axis.axhline(floor, color="#c0392b", linestyle="--", linewidth=1.2,
                 label="AIR floor hg/2 = {:.3f} m".format(floor))
    breaches = [(row["stamp"] - epoch, row["position"][2])
                for agent in AGENTS for row in data["diagnostics"][agent]
                if row["position"][2] < floor or row["medium_flag"] > 0.0]
    if breaches:
        axis.plot([row[0] for row in breaches], [row[1] for row in breaches], marker="x",
                  linestyle="none", color="#c0392b", markersize=4,
                  label="{} domain sample(s) below floor".format(len(breaches)))
    axis.set_xlabel("time since mission epoch [s]")
    axis.set_ylabel("altitude [m]")
    axis.set_title("safety layer: altitude against the AIR domain floor")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=7.5, loc="lower right")


def panel_clearance(axis, data, metrics, executions):
    epoch = metrics["mission_epoch"]
    step = max(len(data["odometry"][0]) // 400, 1)
    stamps = [row[0] for row in data["odometry"][0]][::step]
    obstacle = bool(metrics.get("obstacle_scenario"))
    centre = metrics.get("obstacle_center") if obstacle else None
    size = metrics.get("obstacle_size") if obstacle else None
    pair_t, pair_d, box_t, box_d = [], [], [], []
    for stamp in stamps:
        distance = min_inter_agent(data["odometry"], stamp)
        if distance is not None:
            pair_t.append(stamp - epoch)
            pair_d.append(distance - PLATFORM_DIAMETER_M)
        if centre and size:
            surfaces = [box_signed_distance(sample(data["odometry"][agent], stamp)[1:4],
                                            centre, size) for agent in AGENTS]
            box_t.append(stamp - epoch)
            box_d.append(min(surfaces) - PLATFORM_RADIUS_M)
    axis.plot(pair_t, pair_d, linewidth=1.0, color="#3b7dd8",
              label="min inter-agent surface clearance")
    axis.axhline(0.5, color="#3b7dd8", linestyle="--", linewidth=1.0,
                 label="required 0.50 m")
    if box_t:
        axis.plot(box_t, box_d, linewidth=1.0, color="#c0392b",
                  label="min box surface clearance (actual)")
        axis.axhline(0.2, color="#c0392b", linestyle="--", linewidth=1.0,
                     label="required 0.20 m")
    else:
        axis.text(0.5, 0.45, "no obstacle in this scenario\nbox check = NOT_APPLICABLE",
                  transform=axis.transAxes, ha="center", va="center", fontsize=9,
                  family="monospace", color="0.35")
    for execution in executions:
        axis.axvspan(*hold_window(execution), color="#cfe0f5", zorder=0)
    axis.set_xlabel("time since mission epoch [s]")
    axis.set_ylabel("surface clearance [m]")
    axis.set_title("safety layer: actual clearances against requirements")
    axis.grid(alpha=0.3)
    axis.legend(fontsize=7.5, loc="upper right")


def panel_time(axis, data, metrics, executions, config):
    epoch = metrics["mission_epoch"]
    for agent in AGENTS:
        rows = data["diagnostics"][agent]
        if len(rows) < 2:
            continue
        base_ros, base_model = rows[0]["stamp"], rows[0]["model_time_s"]
        axis.plot([row["stamp"] - epoch for row in rows],
                  [(row["model_time_s"] - base_model) - (row["stamp"] - base_ros)
                   for row in rows], linewidth=0.8, color=COLOURS(agent % 10))
    summary = metrics.get("time_alignment_summary") or {}
    gate = float(config["monitor"].get("max_model_ros_drift", 0.05))
    axis.axhline(gate, color="#c0392b", linestyle="--", linewidth=1.0,
                 label="gate +/- {:.2f} s".format(gate))
    axis.axhline(-gate, color="#c0392b", linestyle="--", linewidth=1.0)
    axis.set_xlabel("time since mission epoch [s]")
    axis.set_ylabel("model - ROS [s]")
    axis.set_title("time layer: max drift {:.5f} s, worst model/ROS rate {:.6f}".format(
        float(summary.get("max_abs_model_ros_drift_s") or 0.0),
        float(summary.get("worst_model_ros_rate") or 0.0)))
    axis.grid(alpha=0.3)
    axis.legend(fontsize=7.5, loc="upper left")
    axis.text(0.98, 0.92, "valid samples {}/{}   cross-agent drift {:.5f} s   "
              "within_thresholds {}".format(
                  summary.get("valid_sample_count"), summary.get("expected_sample_count"),
                  float(summary.get("max_cross_agent_drift_s") or 0.0),
                  summary.get("within_thresholds")),
              transform=axis.transAxes, fontsize=8, va="top", ha="right",
              family="monospace")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_dir")
    parser.add_argument("--output", default=None)
    parser.add_argument("--title", default=None)
    arguments = parser.parse_args()
    directory = Path(arguments.experiment_dir)
    output = Path(arguments.output) if arguments.output else directory / "mission_overview.png"
    title = arguments.title or directory.name

    metrics = load_json(directory / "metrics.json")
    config = load_json(directory / "config.json")
    executions = metrics.get("executions", [])
    data = read_bag(directory / "execution.bag")
    monitor = config["monitor"]
    monitor["start_center"] = config["runner"]["centers"][
        config["runner"].get("initial_target_ref", "start")]
    metrics["monitor_epsilon_p"] = monitor["epsilon_p"]

    mission_epoch = metrics["mission_epoch"]
    errors = slot_errors(data, monitor, mission_epoch)
    tracking = tracking_errors(data, mission_epoch)

    figure = plt.figure(figsize=(21, 14))
    grid = GridSpec(3, 3, figure=figure, height_ratios=[1.0, 1.75, 1.0], hspace=0.5,
                    wspace=0.3, top=0.93, bottom=0.08, left=0.06, right=0.98)
    panel_task(figure.add_subplot(grid[0, 0:2]), metrics, executions)
    panel_allocation(figure.add_subplot(grid[0, 2]), metrics, executions, config)
    panel_path(figure.add_subplot(grid[1, 0:2]), data, metrics, monitor, executions)
    control = GridSpecFromSubplotSpec(2, 1, subplot_spec=grid[1, 2], hspace=0.55)
    panel_slot_error(figure.add_subplot(control[0]), errors, metrics, executions)
    panel_tracking_error(figure.add_subplot(control[1]), tracking, executions)
    panel_altitude(figure.add_subplot(grid[2, 0]), data, metrics)
    panel_clearance(figure.add_subplot(grid[2, 1]), data, metrics, executions)
    panel_time(figure.add_subplot(grid[2, 2]), data, metrics, executions, config)
    figure.suptitle("{}: task allocation -> dispatch -> formation flight -> control".format(
        title), fontsize=15)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=110)
    plt.close(figure)
    print("wrote {}".format(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
