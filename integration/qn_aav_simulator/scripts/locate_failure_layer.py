#!/usr/bin/env python3
"""Offline timeline for the M2 obstacle scenario.

Answers "where did it fail" from recorded evidence rather than from a
parameter sweep: scene ready -> first local scan carrying obstacle points ->
first planner map output containing obstacle -> trajectory published -> qn
adopted -> first clearance shortfall, plus the actual and reference clearance
curves for the threatened member.

The reference curve is built from the reference qn actually used
(``used_reference_pose``); the native ``pos_cmd`` stream is only used to
cross-check the upstream output, and only when its trajectory id, command stamp
and the recorded adoption agree.  Missing evidence is reported as "not
recorded" instead of being inferred.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def signed_box_distance(point, center, size):
    deltas = [abs(point[axis] - center[axis]) - 0.5 * size[axis] for axis in range(3)]
    outside = math.sqrt(sum(value * value for value in deltas if value > 0.0))
    return outside + min(max(deltas), 0.0)


def read_bag(bag_path, box_center, box_size, radius):
    import rosbag
    data = {
        "scene": [], "scans": defaultdict(list), "planner_map": defaultdict(list),
        "commands": defaultdict(list), "adopted": defaultdict(list),
        "odometry": defaultdict(list), "reference": defaultdict(list),
    }
    near = 1.5
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _stamp in bag.read_messages():
            stamp = message.header.stamp.to_sec()
            if topic.endswith("/global_cloud"):
                data["scene"].append((stamp, int(message.width) * int(message.height)))
            elif topic.endswith("_pcl_render_node/cloud"):
                agent = int(topic.split("/")[1].split("_")[1])
                data["scans"][agent].append(
                    (stamp, int(message.width) * int(message.height)))
            elif topic.endswith("grid_map/occupancy_inflate"):
                agent = int(topic.split("/")[1].split("_")[1])
                data["planner_map"][agent].append(
                    (stamp, int(message.width) * int(message.height)))
            elif topic.endswith("/planning/pos_cmd"):
                agent = int(topic.split("/")[1].split("_")[1])
                data["commands"][agent].append((stamp, int(message.trajectory_id)))
            elif topic.endswith("_qn/diagnostics"):
                agent = int(topic.split("/")[1].split("_")[1])
                values = {entry.key: entry.value
                          for status in message.status for entry in status.values}
                if "source_trajectory_id" in values:
                    data["adopted"][agent].append(
                        (stamp, int(values["source_trajectory_id"])))
            elif topic.endswith("_qn/odometry"):
                agent = int(topic.split("/")[1].split("_")[1])
                point = message.pose.pose.position
                position = (point.x, point.y, point.z)
                data["odometry"][agent].append(
                    (stamp, position,
                     signed_box_distance(position, box_center, box_size) - radius))
            elif topic.endswith("_qn/used_reference_pose"):
                agent = int(topic.split("/")[1].split("_")[1])
                point = message.pose.position
                position = (point.x, point.y, point.z)
                data["reference"][agent].append(
                    (stamp, position,
                     signed_box_distance(position, box_center, box_size) - radius))
    for key, series in data.items():
        if isinstance(series, dict):
            for agent in series:
                series[agent].sort(key=lambda row: row[0])
        else:
            series.sort(key=lambda row: row[0])
    return data


def first(predicate, series):
    for row in series:
        if predicate(row):
            return row[0]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_dir")
    parser.add_argument("--member", type=int, default=None,
                        help="threatened member; default is the closest approach")
    parser.add_argument("--non-empty-scan", action="store_true", default=True)
    arguments = parser.parse_args()
    directory = Path(arguments.experiment_dir)
    metrics = json.loads((directory / "metrics.json").read_text())
    config = json.loads((directory / "config.json").read_text())
    monitor = config["monitor"]
    radius = float(monitor.get("platform_radius_m", 0.25))
    required = float(monitor.get("obstacle_clearance", 0.2))
    budget = float(monitor.get("tracking_budget_m", 0.0))
    center = [float(v) for v in metrics["obstacle_center"]]
    size = [float(v) for v in metrics["obstacle_size"]]

    data = read_bag(directory / "execution.bag", center, size, radius)
    worst = None
    for agent, series in data["odometry"].items():
        for _stamp, _position, clearance in series:
            if worst is None or clearance < worst[1]:
                worst = (agent, clearance)
    member = arguments.member if arguments.member is not None else (
        None if worst is None else worst[0])

    timeline = {
        "member": member,
        "scene_ready_s": data["scene"][0][0] if data["scene"] else None,
        "scene_points": data["scene"][0][1] if data["scene"] else None,
    }
    if member is not None:
        timeline["first_scan_with_points_s"] = first(
            lambda row: row[1] > 0, data["scans"].get(member, []))
        timeline["first_planner_map_s"] = first(
            lambda row: row[1] > 0, data["planner_map"].get(member, []))
        commands = data["commands"].get(member, [])
        timeline["first_command_s"] = commands[0][0] if commands else None
        timeline["command_trajectory_ids"] = sorted({row[1] for row in commands})
        adopted = data["adopted"].get(member, [])
        timeline["first_adoption_s"] = adopted[0][0] if adopted else None
        timeline["adopted_trajectory_ids"] = sorted({row[1] for row in adopted})
        violations = [row for row in data["odometry"].get(member, [])
                      if row[2] < required]
        timeline["first_clearance_shortfall_s"] = (
            violations[0][0] if violations else None)
        actual = data["odometry"].get(member, [])
        reference = data["reference"].get(member, [])
        timeline["min_actual_clearance_m"] = min(
            (row[2] for row in actual), default=None)
        timeline["min_reference_clearance_m"] = min(
            (row[2] for row in reference), default=None)
        timeline["reference_budget_required_m"] = required + budget

    # Cross-check: the native command may only be treated as the actual input
    # reference when its trajectory id appears in the adoption record.
    consistent = None
    if member is not None and timeline.get("command_trajectory_ids") and \
            timeline.get("adopted_trajectory_ids"):
        consistent = bool(set(timeline["command_trajectory_ids"])
                          & set(timeline["adopted_trajectory_ids"]))
    timeline["command_adoption_consistent"] = consistent

    if member is not None:
        actual = data["odometry"].get(member, [])
        reference = data["reference"].get(member, [])
        figure, axis = plt.subplots(figsize=(10, 5))
        if actual:
            axis.plot([row[0] - actual[0][0] for row in actual],
                      [row[2] for row in actual], linewidth=1.0,
                      label="actual clearance (state)")
        if reference:
            axis.plot([row[0] - actual[0][0] for row in reference],
                      [row[2] for row in reference], linewidth=1.0,
                      label="reference clearance (qn used reference)")
        axis.axhline(required, color="r", linestyle="--", linewidth=1.0,
                     label="required {:.2f} m".format(required))
        axis.axhline(required + budget, color="orange", linestyle=":", linewidth=1.0,
                     label="reference budget {:.2f} m".format(required + budget))
        axis.set_xlabel("time since first odometry sample [s]")
        axis.set_ylabel("surface clearance to the box [m]")
        axis.set_title("M2 clearance to the declared box (drone_{})".format(member))
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
        figure.tight_layout()
        figure.savefig(directory / "clearance_curves.png", dpi=140)
        plt.close(figure)

    (directory / "failure_timeline.json").write_text(
        json.dumps(timeline, indent=2, sort_keys=True, default=str))
    for key in sorted(timeline):
        print("{:34s} {}".format(key, timeline[key]))
    if timeline.get("first_clearance_shortfall_s") is None:
        print("定位结论: 未观察到实际净距不足（或证据不足，标为尚未定位）")
    elif timeline.get("min_reference_clearance_m") is not None and \
            timeline["min_reference_clearance_m"] < required:
        print("定位结论: 参考轨迹本身已进入不足区域 -> 先查地图/规划/几何，不调 qn")
    elif timeline.get("min_reference_clearance_m") is not None and \
            timeline["min_reference_clearance_m"] >= required:
        print("定位结论: 参考安全、实际不足 -> 查参考强度与跟踪余量")
    else:
        print("定位结论: 尚未定位（缺少参考记录）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
