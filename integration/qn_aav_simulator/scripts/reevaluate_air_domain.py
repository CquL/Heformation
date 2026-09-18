#!/usr/bin/env python3
"""Re-evaluate a recorded run against the AIR-domain rule added in M1.

The three runs that predate the fix were decoded with the message semantics they
were recorded with (no odometry-contract check is applied here); what this tool
adds is the check those runs never had: did the qn plant stay inside its AIR
model, and how low did the members actually fly.  It writes ``revaluation.json``
next to the original record and never rewrites the original verdict.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_dir")
    parser.add_argument("--label", default=None,
                        help="run name to record (defaults to the directory name)")
    parser.add_argument("--air-floor-m", type=float, default=0.085,
                        help="hg_m / 2 of the qn model")
    arguments = parser.parse_args()
    directory = Path(arguments.experiment_dir)
    floor = float(arguments.air_floor_m)

    import rosbag
    max_flag = defaultdict(float)
    min_height = defaultdict(lambda: float("inf"))
    first_stamp = {}
    last_stamp = {}
    with rosbag.Bag(str(directory / "execution.bag")) as bag:
        for topic, message, _stamp in bag.read_messages():
            if topic.endswith("_qn/medium_flag"):
                agent = int(topic.split("/")[1].split("_")[1])
                max_flag[agent] = max(max_flag[agent], float(message.data))
            elif topic.endswith("_qn/odometry"):
                agent = int(topic.split("/")[1].split("_")[1])
                height = float(message.pose.pose.position.z)
                min_height[agent] = min(min_height[agent], height)
                stamp = message.header.stamp.to_sec()
                first_stamp.setdefault(agent, stamp)
                last_stamp[agent] = stamp

    agents = {}
    violation = False
    for agent in sorted(set(list(max_flag) + list(min_height))):
        flag = max_flag.get(agent, 0.0)
        height = min_height.get(agent, float("nan"))
        agent_violation = bool(flag > 0.0 or (math.isfinite(height) and height < floor))
        violation = violation or agent_violation
        agents[str(agent)] = {
            "max_medium_flag": flag,
            "min_height_m": None if not math.isfinite(height) else height,
            "air_floor_m": floor,
            "air_domain_violation": agent_violation,
            "duration_s": (last_stamp.get(agent, 0.0) - first_stamp.get(agent, 0.0)),
        }

    report = {
        "tool": "reevaluate_air_domain",
        "experiment": arguments.label or directory.name,
        "air_floor_m": floor,
        "agents": agents,
        "air_domain_violation": violation,
        "air_domain_ok": not violation,
        "result": (
            "the run stayed inside the AIR model"
            if not violation else
            "the run left the AIR model (non-zero medium_flag or member below "
            "hg_m/2), so it cannot be counted as an AIR verification"),
        "note": (
            "Recorded before the M1 fix, decoded with the message semantics of "
            "that recording.  The original verification.json is unchanged; this "
            "adds the AIR-domain check the run never had."),
    }
    (directory / "revaluation.json").write_text(
        json.dumps(report, indent=2, sort_keys=True))
    print("{}: air_domain_ok={} max_flag={} min_height={}".format(
        directory.name, report["air_domain_ok"],
        max((entry["max_medium_flag"] for entry in agents.values()), default=None),
        min((entry["min_height_m"] for entry in agents.values()
             if entry["min_height_m"] is not None), default=None)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
