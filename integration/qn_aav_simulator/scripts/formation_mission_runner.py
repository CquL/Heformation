#!/usr/bin/env python3
"""Dispatch the current Plan and repair it from native Action completion results."""

import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from mrta_python import (
    Agent, DelayEvent, Plan, PlanItem, Task, TravelTimeProvider,
    build_plan, process_completion,
)
from qn_aav_simulator.msg import (
    FormationAction, FormationActionGoal, FormationActionResult, FormationGoal,
)


def json_default(value):
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError("Cannot encode {}".format(type(value).__name__))


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, default=json_default, allow_nan=False) + "\n")
    temporary.replace(path)


class MissionRunner:
    def __init__(self):
        self.output = Path(rospy.get_param("~output_dir", "/experiments/current"))
        self.output.mkdir(parents=True, exist_ok=True)
        self.mode = rospy.get_param("~mode", "mission")
        if self.mode not in ("single", "mission"):
            raise ValueError("mode must be single or mission")
        self.seed = int(rospy.get_param("~seed", 0))
        self.tolerance = float(rospy.get_param("~delay_tolerance", 0.1))
        self.speed = float(rospy.get_param("~nominal_speed_mps", 1.5))
        self.centers = rospy.get_param("~centers")
        self.initial_ref = rospy.get_param("~initial_target_ref", "start")
        self.tasks = [Task(
            task_id=t["task_id"], required_capabilities=frozenset(t["required_capabilities"]),
            required_agent_count=t["required_agent_count"], service_time=float(t["service_time"]),
            deadline=float(t["deadline"]), target_ref=t["target_ref"],
        ) for t in rospy.get_param("~tasks")]
        self.agents = [Agent("drone_{}".format(i), frozenset({"AAV"}), 0.0) for i in range(7)]
        self.travel = TravelTimeProvider(self.centers, self.speed)
        if self.mode == "single":
            task = next(t for t in self.tasks if t.task_id == "T1")
            self.tasks = [task]
            coalition = tuple(a.id for a in self.agents)
            transit = self.travel(coalition, self.initial_ref, task.target_ref)
            # Test A deliberately does not invoke task selection or plan repair.
            self.plan = Plan([PlanItem(
                "single-T1", task.task_id, coalition, 0.0,
                transit + task.service_time, transit, 0.0, task.service_time, "PLANNED",
            )])
        else:
            self.plan = build_plan(self.agents, self.tasks, self.travel,
                                   initial_target_ref=self.initial_ref, seed=self.seed)
        self.tasks_by_id = {t.task_id: t for t in self.tasks}
        self.final_events = {}
        self.goal_ids = {}
        self.native_results = {}
        self.condition = threading.Condition()
        self.client = actionlib.SimpleActionClient("formation_action", FormationAction)
        self.goal_sub = rospy.Subscriber("/formation_action/goal", FormationActionGoal,
                                         self.on_goal, queue_size=10)
        self.result_sub = rospy.Subscriber("/formation_action/result", FormationActionResult,
                                           self.on_result, queue_size=10)
        self.metrics = {
            "mode": self.mode, "status": "STARTING", "seed": self.seed,
            "nominal_speed_mps": self.speed,
            "planner_speed_mps": rospy.get_param("~planner_speed", 1.5),
            "delay_tolerance": self.tolerance, "mission_epoch": None,
            "initial_plan": asdict(self.plan), "plan_history": [],
            "executions": [], "events": [],
        }

    def on_goal(self, message):
        with self.condition:
            self.goal_ids.setdefault(message.goal.task_id, set()).add(message.goal_id.id)
            self.condition.notify_all()

    def on_result(self, message):
        with self.condition:
            self.native_results[message.status.goal_id.id] = message
            self.condition.notify_all()

    def native_result(self, task_id):
        deadline = time.monotonic() + 5.0
        with self.condition:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                ids = self.goal_ids.get(task_id, set())
                if len(ids) > 1:
                    raise RuntimeError("more than one Action GoalID for {}".format(task_id))
                if ids:
                    goal_id = next(iter(ids))
                    if goal_id in self.native_results:
                        return goal_id, self.native_results[goal_id]
                self.condition.wait(0.05)
        raise RuntimeError("native Action goal/result envelope missing for {}".format(task_id))

    def save(self):
        self.metrics["final_plan"] = asdict(self.plan)
        self.metrics["planned_makespan"] = self.plan.makespan
        self.metrics["events"] = [asdict(e) for e in self.final_events.values()]
        self.metrics["processed_events"] = sorted(e.event_id for e in self.final_events.values())
        save_json(self.output / "metrics.json", self.metrics)

    def wait_ready(self):
        deadline = time.monotonic() + 90.0
        if not self.client.wait_for_server(rospy.Duration(60.0)):
            raise RuntimeError("FormationAction server unavailable")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (rospy.get_param("/formation_action_server/ready", False)
                    and self.goal_sub.get_num_connections() > 0
                    and self.result_sub.get_num_connections() > 0):
                break
            rospy.sleep(0.05)
        else:
            raise RuntimeError("seven-agent formation or Action observation is not ready")
        save_json(self.output / "config.json", {
            "runner": rospy.get_param(rospy.get_name()),
            "monitor": rospy.get_param("/formation_action_server"),
            "agents": [asdict(a) for a in self.agents],
            "tasks": [asdict(t) for t in self.tasks],
            "use_sim_time": rospy.get_param("/use_sim_time", False),
            "planner_parameters": {
                str(i): rospy.get_param("/drone_{}_ego_planner_node".format(i)) for i in range(7)
            },
        })

    def run(self):
        self.save()
        try:
            self.wait_ready()
            epoch = rospy.Time.now().to_sec()
            self.metrics["mission_epoch"] = epoch
            self.metrics["status"] = "RUNNING"
            while any(i.status == "PLANNED" for i in self.plan.items):
                item = next(i for i in self.plan.items if i.status == "PLANNED")
                task = self.tasks_by_id[item.task_id]
                while not rospy.is_shutdown() and rospy.Time.now().to_sec() < epoch + item.planned_start:
                    rospy.sleep(min(0.05, max(0.0, epoch + item.planned_start - rospy.Time.now().to_sec())))
                if rospy.is_shutdown():
                    raise RuntimeError("ROS shutdown before dispatch")
                dispatched_start, dispatched_finish = item.planned_start, item.planned_finish
                goal = FormationGoal()
                goal.task_id = task.task_id
                goal.formation_center.header.frame_id = "world"
                goal.formation_center.header.stamp = rospy.Time.now()
                goal.formation_center.point.x, goal.formation_center.point.y, goal.formation_center.point.z = self.centers[task.target_ref]
                goal.hold_duration = rospy.Duration.from_sec(task.service_time)
                item.status = "RUNNING"
                self.client.send_goal(goal)
                timeout = rospy.get_param("/formation_action_server/execution_timeout", 180.0) + 10.0
                if not self.client.wait_for_result(rospy.Duration(timeout)):
                    self.client.cancel_goal()
                    self.client.wait_for_result(rospy.Duration(3.0))
                    item.status = "CANCELLED"
                    raise RuntimeError("Action transport/result timeout for {}".format(task.task_id))
                state = self.client.get_state()
                result = self.client.get_result()
                goal_id, envelope = self.native_result(task.task_id)
                execution = {
                    "execution_id": item.execution_id, "task_id": task.task_id,
                    "goal_id": goal_id, "terminal_status": state,
                    "planned_start_at_dispatch": dispatched_start,
                    "planned_finish_at_dispatch": dispatched_finish,
                    "reason": result.reason if result is not None else None,
                }
                self.metrics["executions"].append(execution)
                if state != envelope.status.status:
                    raise RuntimeError("Action result status/envelope mismatch")
                if state != GoalStatus.SUCCEEDED or result is None or result.reason != 0:
                    item.status = "CANCELLED" if state == GoalStatus.PREEMPTED else "FAILED"
                    raise RuntimeError("{} ended with status={} reason={}".format(
                        task.task_id, state, execution["reason"]))
                actual_start = result.actual_start_time.to_sec() - epoch
                actual_finish = result.actual_finish_time.to_sec() - epoch
                if not (math.isfinite(actual_start) and math.isfinite(actual_finish)
                        and actual_finish >= actual_start >= 0):
                    raise RuntimeError("invalid Action execution timestamps")
                execution.update(actual_start=actual_start, actual_finish=actual_finish,
                                 delay_seconds=actual_finish - dispatched_finish,
                                 deadline_lateness=max(0.0, actual_finish - task.deadline))
                if self.mode == "mission":
                    event = DelayEvent(goal_id + ":completion", item.execution_id, task.task_id,
                                       dispatched_finish, actual_finish)
                    self.plan, changed = process_completion(
                        self.plan, event, self.final_events, tolerance=self.tolerance)
                    execution["repaired"] = changed
                    self.metrics["plan_history"].append({"event_id": event.event_id, "plan": asdict(self.plan)})
                else:
                    item.status = "COMPLETED"
                    execution["repaired"] = False
                rospy.loginfo("%s complete actual=%.3f planned=%.3f delay=%.3f repaired=%s",
                              task.task_id, actual_finish, dispatched_finish,
                              actual_finish - dispatched_finish, execution["repaired"])
                self.save()
            self.metrics["actual_makespan"] = max(e["actual_finish"] for e in self.metrics["executions"])
            self.metrics["status"] = "PASS"
        except Exception as error:
            # Preserve partial evidence while keeping the failure visible to the caller.
            self.metrics["status"] = "FAILED"
            self.metrics["failure"] = str(error)
            raise
        finally:
            self.save()


def main():
    rospy.init_node("formation_mission_runner")
    MissionRunner().run()


if __name__ == "__main__":
    main()
