#!/usr/bin/env python3
"""Check saved Action/Plan evidence against the recorded qn Odometry."""
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import rosbag


def verify(directory):
    root = Path(directory)
    metrics = json.loads((root / "metrics.json").read_text())
    config = json.loads((root / "config.json").read_text())
    monitor = config["monitor"]
    executions = metrics["executions"]
    expected = 1 if metrics["mode"] == "single" else len(config["tasks"])
    assert metrics["status"] == "PASS", metrics.get("failure", metrics["status"])
    assert len(executions) == expected
    assert len({e["execution_id"] for e in executions}) == expected
    assert len({e["goal_id"] for e in executions}) == expected
    tasks = {t["task_id"]: t for t in config["tasks"]}
    diagnostics = [json.loads(p.read_text()) for p in root.glob("*.diagnostics.json")]
    assert len(diagnostics) == expected, "each Action needs one diagnostic record"
    odometry = defaultdict(list)
    commands = defaultdict(list)
    action_goals, action_results, group_goals = {}, {}, []
    with rosbag.Bag(str(root / "execution.bag")) as bag:
        for topic, message, timestamp in bag.read_messages():
            t = timestamp.to_sec()
            if topic == "/move_base_simple/goal":
                group_goals.append((t, message))
            elif topic == "/formation_action/goal":
                action_goals[message.goal_id.id] = message
            elif topic == "/formation_action/result":
                action_results[message.status.goal_id.id] = message
            elif topic.endswith("_visual_slam/odom"):
                agent = int(topic.split("_")[1])
                assert message._connection_header["callerid"] == "/drone_{}_qn_aav".format(agent)
                assert message.header.frame_id == "world"
                p, v = message.pose.pose.position, message.twist.twist.linear
                odometry[agent].append((message.header.stamp.to_sec(), (p.x, p.y, p.z), (v.x, v.y, v.z)))
            elif topic.endswith("_planning/pos_cmd"):
                agent = int(topic.split("_")[1])
                commands[agent].append(t)
    assert len(group_goals) == expected, "each task must publish exactly one group target"
    assert set(action_goals) == set(action_results) == {e["goal_id"] for e in executions}
    assert set(odometry) == set(commands) == set(range(7))
    epoch = metrics["mission_epoch"]
    for index, execution in enumerate(executions):
        goal_id = execution["goal_id"]
        task = tasks[execution["task_id"]]
        goal = action_goals[goal_id].goal
        result = action_results[goal_id]
        assert result.status.status == execution["terminal_status"] == 3
        assert result.result.reason == execution["reason"] == 0
        assert goal.task_id == execution["task_id"]
        assert goal.hold_duration.to_sec() == task["service_time"]
        assert execution["actual_start"] + 1e-6 >= execution["planned_start_at_dispatch"]
        start = result.result.actual_start_time.to_sec()
        finish = result.result.actual_finish_time.to_sec()
        assert abs(finish - epoch - execution["actual_finish"]) < 1e-6
        assert abs(execution["delay_seconds"] - (execution["actual_finish"] - execution["planned_finish_at_dispatch"])) < 1e-6
        diagnostic = next(d for d in diagnostics if d["task_id"] == task["task_id"])
        assert diagnostic["terminal_state"] == "SUCCEEDED"
        assert diagnostic["goal_publish_count"] == 1
        assert set(diagnostic["planner_nominal_finish_times"]) <= {str(i) for i in range(7)}
        assert all(start <= t <= finish for t in diagnostic["planner_nominal_finish_times"].values())
        center = config["runner"]["centers"][task["target_ref"]]
        group_time, group_message = group_goals[index]
        assert start - .05 <= group_time <= finish
        assert group_message.header.frame_id == "world"
        assert tuple(getattr(group_message.pose.position, c) for c in "xyz") == tuple(center)
        hold_start = finish - task["service_time"]
        for agent in range(7):
            target = tuple(center[j] + monitor["swarm_scale"] * monitor["relative_slots"][str(agent)][j] for j in range(3))
            samples = [s for s in odometry[agent] if hold_start - monitor["odom_timeout"] <= s[0] <= finish]
            during = [s for s in samples if s[0] >= hold_start]
            before = [s for s in samples if s[0] <= hold_start]
            assert before, (task["task_id"], agent, "missing fresh sample at hold start")
            window = [before[-1]] + [s for s in during if s[0] > before[-1][0]]
            assert during, (task["task_id"], agent, "missing hold samples")
            assert during[0][0] - hold_start <= monitor["odom_timeout"]
            assert finish - during[-1][0] <= monitor["odom_timeout"]
            assert max((b[0] - a[0] for a, b in zip(window, window[1:])), default=0) <= monitor["odom_timeout"]
            for _, p, v in window:
                assert math.sqrt(sum((p[j] - target[j]) ** 2 for j in range(3))) <= monitor["epsilon_p"]
                assert math.sqrt(sum(x * x for x in v)) <= monitor["epsilon_v"]
            assert any(start <= t <= finish for t in commands[agent])
        if index:
            assert execution["actual_start"] >= executions[index - 1]["actual_finish"]
    initial, final = metrics["initial_plan"]["items"], metrics["final_plan"]["items"]
    assert [i["execution_id"] for i in initial] == [i["execution_id"] for i in final]
    assert [i["task_id"] for i in initial] == [e["task_id"] for e in executions]
    for item, execution in zip(final, executions):
        assert item["status"] == "COMPLETED"
        assert item["planned_finish"] == execution["planned_finish_at_dispatch"]
        assert item["planned_start"] == execution["planned_start_at_dispatch"]
        assert abs(item["planned_finish"] - item["planned_start"] - item["travel_time"] - item["wait_time"] - item["service_time"]) < 1e-6
        assert item["wait_time"] == 0
    if metrics["mode"] == "mission":
        assert len(metrics["events"]) == len(set(metrics["processed_events"])) == expected
        assert len(metrics["plan_history"]) == expected
        assert any(e["repaired"] for e in executions[:-1]), "no observed repair of a subsequent task"
        previous = initial
        for k, history in enumerate(metrics["plan_history"]):
            event, execution = metrics["events"][k], executions[k]
            assert event["event_id"] == execution["goal_id"] + ":completion" == history["event_id"]
            assert event["execution_id"] == execution["execution_id"]
            assert event["task_id"] == execution["task_id"]
            assert event["planned_finish"] == execution["planned_finish_at_dispatch"]
            assert event["actual_finish"] == execution["actual_finish"]
            items = history["plan"]["items"]
            assert [i["execution_id"] for i in items] == [i["execution_id"] for i in initial]
            for completed in range(k + 1):
                assert items[completed] == final[completed], "completed history was rewritten"
            timing_changed = any((old["planned_start"], old["planned_finish"]) !=
                                 (new["planned_start"], new["planned_finish"])
                                 for old, new in zip(previous[k + 1:], items[k + 1:]))
            assert execution["repaired"] == timing_changed
            release = execution["actual_finish"]
            for old, new in zip(previous[k + 1:], items[k + 1:]):
                assert new["status"] == "PLANNED"
                if event["actual_finish"] > event["planned_finish"] + metrics["delay_tolerance"]:
                    expected_start = max(old["planned_start"], release)
                    assert math.isclose(new["planned_start"], expected_start, abs_tol=1e-6)
                    release = expected_start + old["travel_time"] + old["service_time"]
                    assert math.isclose(new["planned_finish"], release, abs_tol=1e-6)
                else:
                    assert new == old
            if k + 1 < len(items):
                assert items[k + 1]["planned_start"] >= executions[k]["actual_finish"] - metrics["delay_tolerance"]
            previous = items
    summary = {"status": "PASS", "actions": expected, "qn_agents": 7,
               "group_goals": len(group_goals), "independent_odom_hold_check": True,
               "delay_seconds": [e["delay_seconds"] for e in executions]}
    (root / "verification.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    verify(sys.argv[1])
