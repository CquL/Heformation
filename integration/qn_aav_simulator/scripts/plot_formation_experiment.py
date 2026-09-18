#!/usr/bin/env python3
"""Render the recorded seven-member formation runs (plan.md P0-P1 evidence).

Reads one experiment directory produced by ``docker_test_qn_formation_action.sh``
and writes figures that show what actually happened: the flown trajectories, the
per-member slot error against epsilon_p, and the qn model-to-ROS time drift.

This is a read-only analysis tool: it never touches the simulation, the planner
or the plant.  It needs the ROS 1 Python environment for the rosbag reader.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


AGENT_IDS = tuple(range(7))


def load_json(path: Path):
    return json.loads(path.read_text())


def read_bag(bag_path: Path):
    """Return per-agent odometry, qn diagnostics and the published group goals."""
    import rosbag

    odometry = {agent_id: [] for agent_id in AGENT_IDS}
    model_clock = {agent_id: [] for agent_id in AGENT_IDS}
    group_goals = []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _stamp in bag.read_messages():
            if topic.endswith("_qn/odometry"):
                agent_id = int(topic.split("/")[1].split("_")[1])
                position = message.pose.pose.position
                odometry[agent_id].append(
                    (message.header.stamp.to_sec(), position.x, position.y, position.z))
            elif topic.endswith("_qn/diagnostics"):
                agent_id = int(topic.split("/")[1].split("_")[1])
                values = {}
                for status in message.status:
                    for entry in status.values:
                        values[entry.key] = entry.value
                if "model_time_s" in values:
                    model_clock[agent_id].append(
                        (message.header.stamp.to_sec(), float(values["model_time_s"])))
            elif topic == "/move_base_simple/goal":
                point = message.pose.position
                group_goals.append(
                    (message.header.stamp.to_sec(), (point.x, point.y, point.z)))
    for series in list(odometry.values()) + list(model_clock.values()):
        series.sort(key=lambda row: row[0])
    group_goals.sort(key=lambda row: row[0])
    return odometry, model_clock, group_goals


def array(series):
    return np.asarray(series, dtype=float) if series else np.zeros((0, 4))


def targets_over_time(group_goals, slots, scale):
    """Piecewise-constant slot targets: the goal published last before t wins."""
    stamps = np.asarray([row[0] for row in group_goals], dtype=float)

    def centre_at(time_s):
        index = int(np.searchsorted(stamps, time_s, side="right")) - 1
        return group_goals[max(index, 0)][1]

    def target(agent_id, time_s):
        centre = centre_at(time_s)
        slot = slots[str(agent_id)]
        return tuple(centre[axis] + scale * slot[axis] for axis in range(3))

    return target


def plot_trajectories(odometry, group_goals, target, metrics, output):
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(9, 10))
    for agent_id in AGENT_IDS:
        rows = array(odometry[agent_id])
        if rows.size == 0:
            continue
        top.plot(rows[:, 1], rows[:, 2], linewidth=1.0,
                 label="drone_{}".format(agent_id))
        top.plot(rows[0, 1], rows[0, 2], "o", markersize=3, color="0.3")
        bottom.plot(rows[:, 0] - metrics["mission_epoch"], rows[:, 3], linewidth=1.0)
    for stamp, centre in group_goals:
        top.plot(centre[0], centre[1], "k*", markersize=12)
        top.annotate("task goal", (centre[0], centre[1]),
                     textcoords="offset points", xytext=(6, 6), fontsize=8)
    top.set_title("Seven-member AIR formation: flown trajectories (world XY)")
    top.set_xlabel("x [m]")
    top.set_ylabel("y [m]")
    top.axis("equal")
    top.grid(alpha=0.3)
    top.legend(fontsize=7, ncol=2)
    bottom.set_title("Altitude")
    bottom.set_xlabel("time since mission epoch [s]")
    bottom.set_ylabel("z [m]")
    bottom.grid(alpha=0.3)
    figure.tight_layout()
    figure.savefig(output, dpi=140)
    plt.close(figure)


def plot_slot_error(odometry, target, metrics, monitor, output):
    epsilon_p = float(monitor["epsilon_p"])
    epsilon_v = float(monitor["epsilon_v"])
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for agent_id in AGENT_IDS:
        rows = array(odometry[agent_id])
        if rows.size == 0:
            continue
        errors = []
        for row in rows:
            wanted = target(agent_id, row[0])
            errors.append(math.dist((row[1], row[2], row[3]), wanted))
        top.plot(rows[:, 0] - metrics["mission_epoch"], errors,
                 linewidth=1.0, label="drone_{}".format(agent_id))
    top.axhline(epsilon_p, color="r", linestyle="--", linewidth=1.0,
                label="epsilon_p = {} m".format(epsilon_p))
    top.set_ylabel("slot error [m]")
    top.set_title("Distance from each member to its current slot target")
    top.grid(alpha=0.3)
    top.legend(fontsize=7, ncol=2)
    for execution in metrics["executions"]:
        window = (metrics["mission_epoch"] + execution["actual_start"],
                  metrics["mission_epoch"] + execution["actual_finish"])
        for axis in (top, bottom):
            axis.axvspan(window[0] - metrics["mission_epoch"],
                         window[1] - metrics["mission_epoch"],
                         color="0.85", alpha=0.5, zorder=0)
            axis.axvline(window[0] - metrics["mission_epoch"],
                         color="0.4", linewidth=0.8, zorder=1)
    for execution in metrics["executions"]:
        for agent_id in AGENT_IDS:
            rows = array(odometry[agent_id])
            if rows.size == 0:
                continue
            start = metrics["mission_epoch"] + execution["actual_start"]
            end = metrics["mission_epoch"] + execution["actual_finish"]
            inside = rows[(rows[:, 0] >= start) & (rows[:, 0] <= end)]
            if inside.size == 0:
                continue
            step = np.gradient(inside[:, 1:4], inside[:, 0], axis=0)
            bottom.plot(inside[:, 0] - metrics["mission_epoch"],
                        np.linalg.norm(step, axis=1), linewidth=0.8)
    bottom.axhline(epsilon_v, color="r", linestyle="--", linewidth=1.0,
                   label="epsilon_v = {} m/s".format(epsilon_v))
    bottom.set_xlabel("time since mission epoch [s]")
    bottom.set_ylabel("speed [m/s]")
    bottom.set_title("Member speed during the accepted task windows")
    bottom.grid(alpha=0.3)
    bottom.legend(fontsize=8)
    for index, execution in enumerate(metrics["executions"]):
        label = "{}  {}".format(execution["task_id"], execution["execution_id"])
        top.annotate(label, (metrics["mission_epoch"] + execution["actual_start"]
                             - metrics["mission_epoch"], epsilon_p),
                     textcoords="offset points", xytext=(4, 4), fontsize=8)
    figure.tight_layout()
    figure.savefig(output, dpi=140)
    plt.close(figure)


def plot_time_alignment(model_clock, metrics, output):
    """Two panels: the gate view, then the same drift on its own scale.

    The absolute drift is a few hundred microseconds, so a single gate-sized
    axis hides the whole signal.  The lower panel removes each member's initial
    offset and autoscales, which is what shows whether the seven clocks stay
    together or drift apart.
    """
    gate = float((metrics.get("time_alignment_summary") or {}).get(
        "max_abs_model_ros_drift_s", 0.05)) or 0.05
    limit = float((metrics.get("time_alignment_summary") or {}).get(
        "max_abs_model_ros_drift_s", 0.05)) or 0.05
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(10, 8))
    spread = 0.0
    for agent_id in AGENT_IDS:
        rows = array(model_clock[agent_id])
        if rows.size == 0:
            continue
        elapsed_ros = rows[:, 0] - rows[0, 0]
        drift = rows[:, 1] - rows[1, 1] - elapsed_ros
        top.plot(elapsed_ros, drift, linewidth=1.0,
                 label="drone_{}".format(agent_id))
        bottom.plot(elapsed_ros, drift - drift[0], linewidth=1.0,
                    label="drone_{}".format(agent_id))
        spread = max(spread, float(np.max(np.abs(drift - drift[0]))))
    top.axhline(0.05, color="r", linestyle="--", linewidth=1.0)
    top.axhline(-0.05, color="r", linestyle="--", linewidth=1.0,
                label="gate +/- 0.05 s")
    top.set_ylabel("model - ROS [s]")
    top.set_title("qn model clock vs ROS clock against the engineering gate "
                  "(largest drift {:.6f} s)".format(limit))
    top.grid(alpha=0.3)
    top.legend(fontsize=8, ncol=2)
    bottom.set_xlabel("elapsed ROS time [s]")
    bottom.set_ylabel("model - ROS change [s]")
    bottom.set_title("Same clock, initial offset removed (peak-to-peak variation "
                     "{:.3f} ms)".format(1e3 * spread))
    bottom.grid(alpha=0.3)
    bottom.legend(fontsize=8, ncol=2)
    figure.tight_layout()
    figure.savefig(output, dpi=140)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_dir")
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-bag", action="store_true",
                        help="plot from the monitor CSV instead of the rosbag")
    arguments = parser.parse_args()
    directory = Path(arguments.experiment_dir)
    output = Path(arguments.output) if arguments.output else directory / "figures"
    output.mkdir(parents=True, exist_ok=True)

    config = load_json(directory / "config.json")
    metrics = load_json(directory / "metrics.json")
    monitor = config["monitor"]
    slots = monitor["relative_slots"]
    scale = float(monitor["swarm_scale"])

    odometry, model_clock, group_goals = read_bag(directory / "execution.bag")
    if not group_goals:
        raise SystemExit("no group goal recorded in {}".format(directory))
    target = targets_over_time(group_goals, slots, scale)

    plot_trajectories(odometry, group_goals, target, metrics,
                      output / "trajectories.png")
    plot_slot_error(odometry, target, metrics, monitor, output / "slot_error.png")
    plot_time_alignment(model_clock, metrics, output / "time_alignment.png")
    print("wrote {}".format(", ".join(sorted(path.name for path in output.glob("*.png")))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
