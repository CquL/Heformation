#!/usr/bin/env python3
"""Dispatch the current Plan and repair it from native Action completion results.

The runner owns the scheduler-side ``execution_id`` and maps it onto the native
Action GoalID.  It also keeps its own trajectory-ownership evidence, so the
Action server's adoption verdict can be cross-checked instead of trusted.

Test C outputs per action: ``plan_updated``, ``updated_plan_used`` and the
release lag in seconds.  In a serial single-resource scenario the release lag
is not a measure of repair: repair changes planned times, while every dispatch
still waits for the previous action's real completion.  What the runner records
instead is which plan revision each dispatch actually read.
"""

import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from quadrotor_msgs.msg import PositionCommand
from mrta_python import (
    Agent, DelayEvent, Plan, PlanItem, Task, TravelTimeProvider,
    build_plan, process_completion,
)
from qn_aav_simulator.executor_routing import (
    ExecutionUnit, conflicting_active_unit, dispatchable_units, load_routing,
    unit_for_coalition,
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
    temporary.write_text(json.dumps(value, indent=2, default=json_default,
                                    allow_nan=False) + "\n")
    temporary.replace(path)


class MissionRunner:
    def __init__(self):
        self.output = Path(rospy.get_param("~output_dir", "/experiments/current"))
        self.output.mkdir(parents=True, exist_ok=True)
        self.planning_mode = rospy.get_param("~planning_mode", "fixed_coalition")
        if self.planning_mode == "executor":
            self._init_executor()
            return
        if self.planning_mode != "fixed_coalition":
            raise ValueError("planning_mode must be fixed_coalition or executor")
        self.mode = rospy.get_param("~mode", "mission")
        if self.mode not in ("single", "mission"):
            raise ValueError("mode must be single or mission")
        self.seed = int(rospy.get_param("~seed", 0))
        self.tolerance = float(rospy.get_param("~delay_tolerance", 0.1))
        self.speed = float(rospy.get_param("~nominal_speed_mps", 1.5))
        # One scene definition, shared with the publisher, the action server and
        # the verifier; the launch arg only toggles whether the box is present.
        scene = rospy.get_param("/scene", {})
        self.obstacle_scenario = bool(rospy.get_param(
            "~obstacle_scenario", scene.get("obstacle_present", False)))
        self.scene_topic = str(rospy.get_param(
            "~scene_topic", scene.get("topic", "/scene/global_cloud")))
        self.obstacle_center = [float(value) for value in rospy.get_param(
            "~obstacle_center", scene.get("obstacle_center", [-23.0, 0.0, 0.5]))]
        self.obstacle_size = [float(value) for value in rospy.get_param(
            "~obstacle_size", scene.get("obstacle_size", [1.0, 1.0, 1.2]))]
        self.centers = rospy.get_param("~centers")
        self.initial_ref = rospy.get_param("~initial_target_ref", "start")
        raw_repair_mode = rospy.get_param("~repair_mode", "on")
        if isinstance(raw_repair_mode, bool):
            # YAML 1.1 turns an unquoted on/off into a boolean.
            raw_repair_mode = "on" if raw_repair_mode else "off"
        self.repair_mode = str(raw_repair_mode).strip().lower()
        if self.repair_mode not in ("on", "off"):
            raise ValueError("repair_mode must be on or off")
        self.tasks = [Task(
            task_id=t["task_id"],
            required_capabilities=frozenset(t["required_capabilities"]),
            required_agent_count=t["required_agent_count"],
            service_time=float(t["service_time"]),
            deadline=float(t["deadline"]), target_ref=t["target_ref"],
        ) for t in rospy.get_param("~tasks")]
        self.agents = [Agent("drone_{}".format(i), frozenset({"AAV"}), 0.0) for i in range(7)]
        self.travel = TravelTimeProvider(self.centers, self.speed)
        if self.mode == "single":
            task = next(t for t in self.tasks if t.task_id == "T1")
            self.tasks = [task]
            coalition = tuple(a.id for a in self.agents)
            transit = self.travel(coalition, self.initial_ref, task.target_ref)
            self.plan = Plan([PlanItem(
                "single-T1", task.task_id, coalition, 0.0,
                transit + task.service_time, transit, 0.0, task.service_time, "PLANNED",
            )])
        else:
            self.plan = build_plan(self.agents, self.tasks, self.travel,
                                   initial_target_ref=self.initial_ref, seed=self.seed)
        self.tasks_by_id = {t.task_id: t for t in self.tasks}
        # Static routing: executor -> member list -> Action endpoint.  A unit
        # without an endpoint can take part in offline planning but is never
        # dispatched, so it can never produce a fabricated "actual completion".
        self.routing = self._load_routing()
        self.online_executors = self._online_executors()
        # Units currently occupied.  Registration may overlap on purpose; being
        # occupied at the same time may not.
        self.active_executor_ids = set()
        # Increments whenever the completion step replaces the plan.  A dispatch
        # records the revision it read, so "the next action used the updated
        # plan" is evidenced instead of assumed.
        self.plan_revision = 0
        self.final_events = {}
        self.goal_ids = {}
        self.native_results = {}
        self.condition = threading.Condition()
        self.command_trajectory = {}
        self.qn_source = {}
        self.client = actionlib.SimpleActionClient("formation_action", FormationAction)
        self.goal_sub = rospy.Subscriber("/formation_action/goal", FormationActionGoal,
                                         self.on_goal, queue_size=10)
        self.result_sub = rospy.Subscriber("/formation_action/result",
                                           FormationActionResult, self.on_result,
                                           queue_size=10)
        for agent_id in range(7):
            rospy.Subscriber("/drone_{}_planning/pos_cmd".format(agent_id),
                             PositionCommand,
                             self.on_command, callback_args=agent_id, queue_size=10)
            rospy.Subscriber("/drone_{}_qn/diagnostics".format(agent_id),
                             DiagnosticArray,
                             self.on_diagnostics, callback_args=agent_id, queue_size=5)
        self.metrics = {
            "mode": self.mode, "status": "STARTING", "seed": self.seed,
            "nominal_speed_mps": self.speed,
            "planner_speed_mps": rospy.get_param("~planner_speed", 1.5),
            "delay_tolerance": self.tolerance, "repair_mode": self.repair_mode,
            "obstacle_scenario": self.obstacle_scenario,
            "obstacle_center": self.obstacle_center,
            "obstacle_size": self.obstacle_size,
            "scene_topic": self.scene_topic,
            "mission_epoch": None,
            "initial_plan": asdict(self.plan), "plan_history": [],
            "executions": [], "events": [], "test_c": {},
        }

    def _init_executor(self):
        """Opt-in task line; preserve the original seven-member runner path."""
        from nav_msgs.msg import Odometry
        from qn_aav_simulator.task_line import load_request, load_formation_phase
        from qn_aav_simulator.observation_coverage import CoverageResult
        configured = rospy.get_param("~executors")
        self.routing = load_routing(configured, default_members=(),
                                    default_initial_target_ref="start")
        self.units = dispatchable_units(self.routing)
        if not self.units:
            raise ValueError("no online executors")
        self.request_path = Path(rospy.get_param("~request_file"))
        self.request = load_request(self.request_path)
        self.formation_phase = load_formation_phase(self.request_path)
        self.fleet = sorted({m for u in self.units for m in u.physical_agent_ids})
        self.condition = threading.Condition()
        self.actual = {}
        self.goal_ids, self.native_results = {}, {}
        self.active_executor_ids = set(rospy.get_param("~resource_locks", []))
        if self.active_executor_ids:
            raise RuntimeError("UNKNOWN_LOCKED from previous run: restart the whole execution chain")
        self.clients, self.action_subs = {}, []
        self.server_nodes, self.member_slots = {}, {}
        self.coverage = CoverageResult()
        self.final_events = {}
        self.plan_revision = 0
        self.speed = float(rospy.get_param("~nominal_speed_mps", 1.5))
        self.seed = int(rospy.get_param("~seed", 0))
        self.metrics = {"planning_mode": "executor", "status": "STANDBY",
                        "request_id": self.request.request_id, "executions": [],
                        "plan_history": [], "results_received": [], "failure_reason": "",
                        "observation_model": "DECLARED_GEOMETRY_AND_DWELL_ONLY",
                        "payload_quality": "UNVERIFIED", "delivery_model": "ZERO_LATENCY_LOCAL_RESULT",
                        "formation_business_shape_verdict": "NOT_DEFINED"}
        self.plan = None
        for unit in self.units:
            endpoint = unit.action_endpoint.rstrip("/")
            self.clients[unit.executor_id] = actionlib.SimpleActionClient(endpoint, FormationAction)
            self.action_subs.extend([
                rospy.Subscriber(endpoint + "/goal", FormationActionGoal, self.on_goal, queue_size=20),
                rospy.Subscriber(endpoint + "/result", FormationActionResult, self.on_result, queue_size=20)])
        for member in self.fleet:
            self.action_subs.append(rospy.Subscriber("/" + member + "_qn/odometry", Odometry,
                self._on_executor_odom, callback_args=member, queue_size=10))

    def _on_executor_odom(self, message, member):
        from qn_aav_simulator.odometry import parse_standard_odometry, OdometryContractError
        try:
            sample = parse_standard_odometry(message, member)
        except OdometryContractError:
            with self.condition:
                self.actual.pop(member, None)
            return
        with self.condition:
            self.actual[member] = sample
            self.condition.notify_all()

    def _wait_executor_ready(self, unit, timeout=120.0):
        import rosgraph
        endpoint = unit.action_endpoint.rstrip("/")
        deadline = time.monotonic() + timeout
        client = self.clients[unit.executor_id]
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if client.wait_for_server(rospy.Duration(.2)):
                publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                nodes = publishers.get(endpoint + "/status", [])
                if len(nodes) != 1:
                    raise RuntimeError("Action endpoint must have exactly one owner: " + endpoint)
                node = nodes[0]
                self.server_nodes[unit.executor_id] = node
                if rospy.get_param(node + "/safety_latched_members", []):
                    raise RuntimeError("physical member safety hold locked: " + endpoint)
                if rospy.get_param(node + "/ready", False):
                    declared = rospy.get_param(node + "/agent_ids")
                    if {"drone_{}".format(a) for a in declared} != unit.members():
                        raise RuntimeError("endpoint member configuration disagrees with routing")
                    safety_members = rospy.get_param(node + "/safety_agent_ids", declared)
                    if {"drone_{}".format(a) for a in safety_members} != set(self.fleet):
                        raise RuntimeError("endpoint does not monitor whole-fleet collision safety")
                    raw = rospy.get_param(node + "/relative_slots")
                    scale = float(rospy.get_param(node + "/swarm_scale"))
                    self.member_slots[unit.executor_id] = {
                        "drone_" + str(a): tuple(scale*float(x) for x in raw[str(a)]) for a in declared}
                    return
            rospy.sleep(.1)
        node = self.server_nodes.get(unit.executor_id, "")
        raise RuntimeError("endpoint not ready: {}: {}".format(endpoint,
            rospy.get_param(node + "/readiness_reason", "unavailable")))

    def _actual_positions(self):
        now = rospy.Time.now().to_sec()
        # Use the action servers' existing freshness contract.
        timeout = min(float(rospy.get_param(n + "/odom_timeout", .25))
                      for n in self.server_nodes.values())
        with self.condition:
            missing = [m for m in self.fleet if m not in self.actual
                       or not self.actual[m].is_fresh(now, timeout)]
            if missing:
                raise RuntimeError("missing/fresh actual positions: " + str(missing))
            return {m: self.actual[m].position for m in self.fleet}

    def _executor_plan(self, observations, *, include_formation=False, release=0.0):
        from mrta_python import Executor, ExecutorPlan, ExecutorTravelTimeProvider, build_executor_plan
        from qn_aav_simulator.task_line import request_centres, to_plan_tasks
        positions = self._actual_positions()
        centers = request_centres(observations)
        centers["start"] = positions[self.fleet[0]]  # member positions, not this fallback, determine cost
        executors = [Executor(u.executor_id, u.physical_agent_ids, frozenset(u.capabilities),
                              release, self.speed, "start") for u in self.units]
        travel = ExecutorTravelTimeProvider(centers, {u.executor_id: self.speed for u in self.units},
                    member_slots=self.member_slots, member_positions=dict(positions))
        tasks = list(to_plan_tasks(observations))
        plan = build_executor_plan(executors, tasks, travel, initial_target_ref="start", seed=self.seed, serial=True,
                                  serial_units={u.executor_id for u in self.units}) if tasks else ExecutorPlan()
        if include_formation and self.formation_phase:
            phase = self.formation_phase
            # Explicit assembly then transfer stages, appended with the same
            # serial release and physical-member predictions. No online USV clock.
            for suffix, point in (("assemble", phase.path_start), ("transfer", phase.path_end)):
                task_id = phase.phase_id + "-" + suffix
                if task_id in centers:
                    raise ValueError("formation task id collides with observation")
                centers[task_id] = point
                task = Task(task_id, frozenset({"AIR"}), phase.required_agent_count,
                            self.request.service_time_s, self.request.deadline_s, task_id)
                start = max(release, plan.makespan)
                from dataclasses import replace
                staged = [replace(e, available_from=start) for e in executors]
                part = build_executor_plan(staged, [task], travel, initial_target_ref="start", seed=self.seed, serial=True,
                                            serial_units={e.executor_id for e in staged})
                plan.items.extend(part.items)
                tasks.append(task)
        self.centers = getattr(self, "centers", {})
        self.centers.update(centers)
        self.tasks_by_id = getattr(self, "tasks_by_id", {})
        self.tasks_by_id.update({t.task_id: t for t in tasks})
        self.observation_tasks = getattr(self, "observation_tasks", {})
        self.observation_tasks.update({t.task_id: t for t in observations})
        return plan

    def _save_executor(self):
        self.metrics["plan"] = asdict(self.plan) if self.plan is not None else None
        self.metrics["plan_revision"] = self.plan_revision
        self.metrics["resource_locks"] = sorted(self.active_executor_ids)
        self.metrics["coverage"] = asdict(self.coverage)
        self.metrics["observed_fraction"] = self.coverage.observed_fraction(self.weights)
        self.metrics["delivered_fraction"] = self.coverage.delivered_fraction(self.weights)
        self.metrics["updated_at_ros_s"] = rospy.Time.now().to_sec()
        save_json(self.output / "metrics.json", self.metrics)
        # A compact view of the task authority, using the existing ROS parameter
        # service. No new message definition or independent dashboard completion.
        state = {key: self.metrics[key] for key in (
            "status", "request_id", "plan", "plan_revision", "resource_locks", "coverage",
            "observed_fraction", "delivered_fraction", "results_received", "failure_reason",
            "observation_model", "payload_quality", "updated_at_ros_s")}
        state["current_action"] = self.metrics.get("current_action")
        state["safety_disposition"] = self.metrics.get("safety_disposition")
        rospy.set_param("~task_state", json.dumps(state, default=json_default, allow_nan=False))
        rospy.set_param("~resource_locks", sorted(self.active_executor_ids))

    def _refresh_executor_timing(self, release):
        """Fixed remaining assignments, costs refreshed from actual physical members."""
        from mrta_python import ExecutorTravelTimeProvider
        travel = ExecutorTravelTimeProvider(self.centers,
            {u.executor_id: self.speed for u in self.units}, member_slots=self.member_slots,
            member_positions=self._actual_positions())
        for item in self.plan.items:
            if item.status != "PLANNED":
                continue
            task = self.tasks_by_id[item.task_id]
            item.travel_time = travel(item.executor_id, "start", task.target_ref)
            item.planned_start = max(item.planned_start, release)
            item.planned_finish = item.planned_start + item.travel_time + item.service_time
            release = item.planned_finish
            for member in item.coalition:
                slot = self.member_slots[item.executor_id][member]
                travel.member_positions[member] = tuple(self.centers[task.target_ref][a] + slot[a]
                                                       for a in range(3))
        self.plan_revision += 1
        self.metrics["plan_history"].append({"revision": self.plan_revision,
                                            "plan": asdict(self.plan)})

    @staticmethod
    def _release_result_ok(state, result, execution_id):
        return (state == GoalStatus.SUCCEEDED and result is not None
                and result.task_id == execution_id and bool(result.goal_id)
                and result.reason == 0 and result.task_outcome == 1
                and result.safety_outcome == 1 and result.experiment_validity == 1)

    def _dispatch_executor_item(self, item):
        from mrta_python.repair import process_executor_completion
        unit = unit_for_coalition(self.routing, item.coalition)
        if unit is None or unit.executor_id != item.executor_id:
            raise RuntimeError("plan coalition has no matching online endpoint")
        if conflicting_active_unit(self.routing, self.active_executor_ids, unit.executor_id):
            raise RuntimeError("physical members already occupied")
        self._wait_executor_ready(unit)
        point = self.centers[self.tasks_by_id[item.task_id].target_ref]
        goal = FormationGoal()
        goal.task_id = item.execution_id
        goal.formation_center.header.frame_id = "world"
        goal.formation_center.header.stamp = rospy.Time.now()
        goal.formation_center.point.x, goal.formation_center.point.y, goal.formation_center.point.z = point
        goal.hold_duration = rospy.Duration(item.service_time)
        item.status = "RUNNING"
        self.active_executor_ids.add(unit.executor_id)
        self.metrics["current_action"] = {"task_id": item.task_id, "execution_id": item.execution_id,
                                         "endpoint": unit.action_endpoint, "phase": "DISPATCHING"}
        self._save_executor()  # reserve before sending, survive runner interruption
        client = self.clients[unit.executor_id]
        def feedback(message):
            self.metrics["current_action"]["phase"] = "HOLDING" if message.phase == 1 else "MOVING"
        client.send_goal(goal, feedback_cb=feedback)
        node = self.server_nodes[unit.executor_id]
        disposition_budget = (float(rospy.get_param(node + "/safety_hold_timeout_s", 180))
                              if rospy.get_param(node + "/safety_hold_enabled", False) else 0.0)
        deadline = time.monotonic() + float(rospy.get_param(node + "/execution_timeout", 180)) + disposition_budget + 10.0
        while not rospy.is_shutdown():
            if client.wait_for_result(rospy.Duration(.5)):
                break
            disposition = rospy.get_param(node + "/safety_disposition", None)
            if disposition is not None:
                self.metrics["current_action"]["phase"] = "SAFETY_HOLD"
                self.metrics["safety_disposition"] = json.loads(disposition)
            self._save_executor()
            if time.monotonic() >= deadline:
                raise RuntimeError("result timeout; endpoint={} client_state={} server_state={}".format(
                    unit.action_endpoint, client.get_state(), rospy.get_param(node + "/run_state", "unknown")))
        result = client.get_result()
        state = client.get_state()
        if not self._release_result_ok(state, result, item.execution_id):
            detail = "missing native Result"
            if result is not None:
                detail = "reason={} task={} safety={} validity={}".format(
                    result.reason, result.task_outcome, result.safety_outcome, result.experiment_validity)
                evidence_path = self.output / result.evidence_file
                if evidence_path.parent == self.output and evidence_path.is_file():
                    failed_evidence = json.loads(evidence_path.read_text())
                    if failed_evidence.get("goal_id") == result.goal_id:
                        detail += "; " + failed_evidence.get("reason_text", "")
                        if failed_evidence.get("safety_hold"):
                            self.metrics["safety_disposition"] = failed_evidence["safety_hold"]
                self.metrics["executions"].append({"task_id": item.task_id,
                    "execution_id": item.execution_id, "endpoint": unit.action_endpoint,
                    "goal_id": result.goal_id, "result_received_at": rospy.Time.now().to_sec(),
                    "evidence_file": result.evidence_file, "result": "NOT_ACCEPTED",
                    "native_state": state, "native_execution_id": result.task_id,
                    "plan_updated": False, "detail": detail})
                self.metrics["results_received"].append(item.task_id)
            raise RuntimeError("native Result cannot release members; endpoint={} state={} {}".format(
                unit.action_endpoint, state, detail))
        goal_id, native = self.native_result(item.execution_id)
        if goal_id != result.goal_id or native.status.status != GoalStatus.SUCCEEDED:
            raise RuntimeError("native GoalID/Result envelope mismatch")
        evidence = json.loads((self.output / result.evidence_file).read_text())
        if (evidence.get("goal_id") != goal_id or not evidence.get("accepted_for_dispatch")
                or not evidence.get("resource_released")):
            raise RuntimeError("server evidence does not authorize physical release")
        received = rospy.Time.now().to_sec()
        event = DelayEvent(goal_id, item.execution_id, item.task_id, item.planned_finish,
                           max(result.actual_finish_time.to_sec(), received) - self.epoch)
        self.plan, changed = process_executor_completion(self.plan, event, self.final_events)
        self.metrics["executions"].append({"task_id": item.task_id, "execution_id": item.execution_id,
            "endpoint": unit.action_endpoint, "goal_id": goal_id, "result_received_at": received,
            "evidence_file": result.evidence_file, "result": "SUCCEEDED", "plan_updated": changed,
            "formation_geometry": evidence.get("formation_geometry")})
        self.metrics["results_received"].append(item.task_id)
        self._receive_observations(item, evidence)
        # No finally-discard: every exceptional/unknown exit keeps the reservation.
        self.active_executor_ids.remove(unit.executor_id)
        self.metrics["current_action"] = None
        self._refresh_executor_timing(received - self.epoch)
        self._save_executor()

    def _receive_observations(self, item, evidence):
        from qn_aav_simulator.observation_coverage import ObservationSample, evaluate_coverage, record_delivery
        task = self.observation_tasks.get(item.task_id)
        if task is None:
            return
        # Only this execution's actual successful holding interval is eligible;
        # no old trajectory, inferred target position or previous-task observation.
        hold = evidence.get("successful_hold_window") or {}
        if hold.get("start") is None or hold.get("end") is None:
            raise RuntimeError("successful holding interval missing")
        samples = []
        for member, rows in evidence.get("member_samples", {}).items():
            previous = None
            for row in rows:
                stamp = row["state_stamp_s"]
                if stamp is None or not hold["start"] <= stamp <= hold["end"]:
                    continue
                if stamp == previous:
                    continue
                previous = stamp
                samples.append(ObservationSample("drone_" + member, stamp, tuple(row["position"])))
        coverage = evaluate_coverage(samples, {p: self.points[p] for p in task.covers},
            self.request.requirement, self.obstacles, sample_timeout_s=evidence["sample_timeout_s"])
        for point_id, observation in coverage.points.items():
            if observation.observed or point_id not in self.coverage.points:
                self.coverage.points[point_id] = observation
        # The local Result carries access to the observation evidence. Receipt is
        # explicit and zero-latency in this simulator; no radio link is claimed.
        record_delivery(self.coverage, task.covers)

    def _run_executor(self):
        from qn_aav_simulator.monitoring_request import expand
        from qn_aav_simulator.task_line import retest_tasks
        from qn_aav_simulator.observation_coverage import ObstacleBox
        self.points = {p.point_id: p.position for r in self.request.regions for p in r.interest_points}
        self.weights = {p.point_id: p.weight for r in self.request.regions for p in r.interest_points}
        scene = rospy.get_param("/scene", {})
        self.obstacles = ([ObstacleBox(tuple(scene["obstacle_center"]), tuple(scene["obstacle_size"]))]
                          if scene.get("obstacle_present", False) else [])
        self._save_executor()
        try:
            for unit in self.units:
                self._wait_executor_ready(unit)
            observations = expand(self.request, {c for u in self.units for c in u.capabilities})
            self.plan = self._executor_plan(observations, include_formation=True)
            self.metrics["initial_plan"] = asdict(self.plan)
            self.metrics["status"] = "AWAITING_CONFIRMATION"
            self._save_executor()
            print(json.dumps({"request_id": self.request.request_id, "tasks": asdict(self.plan),
                "targets": self.centers, "endpoints": {u.executor_id: u.action_endpoint for u in self.units},
                "limits": ["declared geometric visibility/dwell only; image quality unverified",
                           "zero-latency local result delivery; no USV/UUV online execution",
                           "formation transfer has no business shape/corridor pass threshold",
                           "one optional retest of uncovered points; no new batch during execution"]},
                 indent=2, default=json_default), flush=True)
            try:
                answer = input("Confirm this request and at most one retest? Type yes to dispatch: ")
            except (EOFError, KeyboardInterrupt):
                answer = ""
            if answer.strip().lower() != "yes":
                self.metrics["status"] = "NOT_CONFIRMED"
                self._save_executor()
                return
            self.metrics["confirmation"] = {"answer": "yes", "at_ros_s": rospy.Time.now().to_sec()}
            self.epoch = rospy.Time.now().to_sec()
            self.metrics["mission_epoch"] = self.epoch
            self.metrics["status"] = "RUNNING"
            self._refresh_executor_timing(0.0)
            retested = False
            while not rospy.is_shutdown():
                pending = next((i for i in self.plan.items if i.status == "PLANNED"), None)
                if pending is None:
                    extra = retest_tasks(self.request, observations, self.coverage, self.weights,
                        delivery_recorded=all(t.task_id in self.metrics["results_received"] for t in observations),
                        already_retested=retested)
                    if extra:
                        retested = True
                        self.plan.items.extend(self._executor_plan(extra, release=rospy.Time.now().to_sec()-self.epoch).items)
                        self.plan_revision += 1
                        continue
                    break
                while rospy.Time.now().to_sec() - self.epoch < pending.planned_start:
                    if rospy.is_shutdown():
                        raise RuntimeError("shutdown before dispatch")
                    rospy.sleep(.1)
                self._dispatch_executor_item(pending)
            if rospy.is_shutdown():
                raise RuntimeError("runner shutdown")
            fraction = (self.coverage.delivered_fraction(self.weights) if self.request.delivery_required
                        else self.coverage.observed_fraction(self.weights))
            elapsed = rospy.Time.now().to_sec() - self.epoch
            self.metrics["deadline_met"] = elapsed <= self.request.deadline_s
            self.metrics["status"] = ("FAIL_COVERAGE" if fraction != 1.0 else
                                      "PASS_GEOMETRIC_PROXY" if self.metrics["deadline_met"] else "FAIL_DEADLINE")
            self.metrics["formation_motion_complete"] = all(
                i.status == "COMPLETED" for i in self.plan.items if i.task_id not in self.observation_tasks)
            if fraction != 1.0:
                self.metrics["failure_reason"] = "points remain unobserved after the permitted retest"
            self._save_executor()
        except BaseException as error:
            self.metrics["failure_reason"] = str(error)
            self.metrics["status"] = "UNKNOWN_LOCKED" if self.active_executor_ids else "FAIL"
            if self.metrics.get("current_action"):
                self.metrics["current_action"]["phase"] = self.metrics["status"]
            if self.plan:
                for item in self.plan.items:
                    if item.status == "RUNNING":
                        item.status = "UNKNOWN_LOCKED"
            self._save_executor()
            raise

    # ------------------------------------------------------------ callbacks
    def on_goal(self, message):
        with self.condition:
            self.goal_ids.setdefault(message.goal.task_id, set()).add(
                message.goal_id.id)
            self.condition.notify_all()

    def on_result(self, message):
        with self.condition:
            self.native_results[message.status.goal_id.id] = message
            self.condition.notify_all()

    def on_command(self, message, agent_id):
        with self.lock_condition():
            self.command_trajectory.setdefault(agent_id, []).append(
                (int(message.trajectory_id), float(message.header.stamp.to_sec()),
                 rospy.Time.now().to_sec()))

    def on_diagnostics(self, message, agent_id):
        values = {}
        for status in message.status:
            for entry in status.values:
                values[entry.key] = entry.value
        if "source_trajectory_id" not in values:
            return
        with self.lock_condition():
            self.qn_source.setdefault(agent_id, []).append(
                (int(values["source_trajectory_id"]),
                 int(values.get("used_outer_step", -1)),
                 rospy.Time.now().to_sec()))

    def lock_condition(self):
        return self.condition

    # ------------------------------------------------------------- helpers
    def native_result(self, task_id):
        deadline = time.monotonic() + 10.0
        with self.condition:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                ids = self.goal_ids.get(task_id, set())
                if len(ids) > 1:
                    raise RuntimeError(
                        "more than one Action GoalID for {}".format(task_id))
                if ids:
                    goal_id = next(iter(ids))
                    if goal_id in self.native_results:
                        return goal_id, self.native_results[goal_id]
                self.condition.wait(0.05)
        raise RuntimeError(
            "native Action goal/result envelope missing for {}".format(task_id))

    def action_evidence(self, goal_id):
        index_path = self.output / "action_index.json"
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                index = json.loads(index_path.read_text())
            except (IOError, ValueError):
                index = {}
            name = index.get(goal_id)
            if name:
                try:
                    return json.loads((self.output / name).read_text())
                except ValueError:
                    pass
            rospy.sleep(0.05)
        return None

    def time_alignment_baseline(self):
        """The baseline snapshot that admitted this run."""
        raw = rospy.get_param("/formation_action_server/time_alignment_baseline", None)
        if raw is None:
            return self.time_alignment_session()
        try:
            return json.loads(raw)
        except (TypeError, ValueError):
            return {}

    def time_alignment_session(self):
        raw = rospy.get_param("/formation_action_server/time_alignment_session", None)
        if isinstance(raw, str):
            try:
                return json.loads(raw)
            except ValueError:
                return {}
        return raw if isinstance(raw, dict) else {}

    def save(self):
        summary = self.time_alignment_session()
        if summary:
            self.metrics["time_alignment_summary"] = summary
        self.metrics["final_plan"] = asdict(self.plan)
        self.metrics["planned_makespan"] = self.plan.makespan
        self.metrics["events"] = [asdict(e) for e in self.final_events.values()]
        self.metrics["processed_events"] = sorted(
            e.event_id for e in self.final_events.values())
        save_json(self.output / "metrics.json", self.metrics)

    def wait_ready(self, timeout_s=300.0):
        """Block until the Action server advertises READY_IDLE.

        Used before the first dispatch (30 s baseline qualification) and before
        every later dispatch: a resource that is still being released, or that
        lost a run-time health prerequisite, must not be handed a new task.
        """
        deadline = time.monotonic() + float(timeout_s)
        if not self.client.wait_for_server(rospy.Duration(min(120.0, float(timeout_s)))):
            raise RuntimeError("FormationAction server unavailable")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (rospy.get_param("/formation_action_server/ready", False)
                    and self.goal_sub.get_num_connections() > 0
                    and self.result_sub.get_num_connections() > 0):
                return True
            rospy.sleep(0.1)
        return False

    def wait_ready_or_raise(self):
        if not self.wait_ready(300.0):
            raise RuntimeError(
                "FormationAction never reached READY_IDLE: {}".format(
                    rospy.get_param("/formation_action_server/readiness_reason", "unknown")))
        self.metrics["ready_ros_time_s"] = rospy.Time.now().to_sec()
        self.metrics["baseline_qualification"] = self.time_alignment_baseline()
        save_json(self.output / "config.json", {
            "runner": rospy.get_param(rospy.get_name()),
            "monitor": rospy.get_param("/formation_action_server"),
            "agents": [asdict(a) for a in self.agents],
            "tasks": [asdict(t) for t in self.tasks],
            "use_sim_time": rospy.get_param("/use_sim_time", False),
            "repair_mode": self.repair_mode,
            "planner_parameters": {
                str(i): rospy.get_param("/drone_{}_ego_planner_node".format(i))
                for i in range(7)},
        })

    # ----------------------------------------------------------- execution
    def dispatch_goal_with_retry(self, goal, task):
        """Send one goal, waiting for READY_IDLE before every retry.

        A rejection while the resource is still owned by the previous task (or
        while the server is re-qualifying) is expected P1 behaviour, not an
        experiment failure.  Each retry is recorded; the adoption/dispatch
        evidence uses the attempt that was actually accepted.
        """
        timeout = rospy.get_param(
            "/formation_action_server/execution_timeout", 180.0) + 30.0
        attempts = 0
        while True:
            if attempts and not self.wait_ready(60.0):
                return None, None, rospy.Time.now().to_sec(), False, attempts
            dispatch_time = rospy.Time.now().to_sec()
            self.client.send_goal(goal)
            if not self.client.wait_for_result(rospy.Duration(timeout)):
                return None, None, dispatch_time, True, attempts
            state = self.client.get_state()
            result = self.client.get_result()
            if state == GoalStatus.REJECTED and attempts < 3:
                attempts += 1
                self.metrics.setdefault("dispatch_retries", []).append({
                    "task_id": task.task_id,
                    "attempt": attempts,
                    "ros_time_s": dispatch_time,
                    "server_reason": (result.reason if result is not None else None),
                })
                continue
            self.metrics.setdefault("dispatch_attempts", {})[task.task_id] = attempts + 1
            return state, result, dispatch_time, False, attempts + 1

    def _load_routing(self):
        """Read the static executor routing table from the launch parameters."""
        configured = rospy.get_param("~executors", None)
        return load_routing(
            configured,
            default_members=("drone_{}".format(i) for i in range(7)),
            default_initial_target_ref=self.initial_ref)

    def _online_executors(self):
        """Every unit this process can dispatch to.

        Units without an Action endpoint stay in the routing table - they may be
        planned for - but are never dispatched, so a target-fleet platform with
        no backend cannot produce a completion event.
        """
        online = dispatchable_units(self.routing)
        if not online:
            raise RuntimeError("no executor with a real Action endpoint is configured")
        return online

    def execution_context(self, item, epoch, previous_actual_finish, unit):
        with self.condition:
            pre_trajectory = {
                agent_id: (self.command_trajectory[agent_id][-1][0]
                           if self.command_trajectory.get(agent_id) else None)
                for agent_id in range(7)}
            pre_source = {
                agent_id: (self.qn_source[agent_id][-1][0]
                           if self.qn_source.get(agent_id) else None)
                for agent_id in range(7)}
        return {
            "execution_id": item.execution_id,
            "pre_dispatch_trajectory_ids": pre_trajectory,
            "pre_dispatch_qn_source_trajectory_ids": pre_source,
            "initial_planned_start": item.planned_start,
            "initial_planned_finish": item.planned_finish,
            "previous_actual_finish": previous_actual_finish,
            # Which plan revision this dispatch actually read, together with the
            # planned_start it read.  This is the evidence that the next action
            # used the plan the completion step produced.
            "plan_revision_at_dispatch": self.plan_revision,
            "planned_start_read_at_dispatch": item.planned_start,
            # Routing evidence: which unit the plan selected, which members it
            # owns and which endpoint the task was actually sent to.
            "executor_id": unit.executor_id,
            "action_endpoint": unit.action_endpoint,
            "assigned_members": tuple(item.coalition),
            "executed_members": tuple(unit.physical_agent_ids),
        }

    def collect_post_evidence(self, context, dispatch_ros_time_s, result_finish_s):
        with self.condition:
            post_trajectory = {}
            qn_source = {}
            for agent_id in range(7):
                post_trajectory[agent_id] = sorted({
                    trajectory_id
                    for trajectory_id, _stamp, received in self.command_trajectory.get(agent_id, [])
                    if received >= dispatch_ros_time_s})
                qn_source[agent_id] = [
                    (trajectory_id, step)
                    for trajectory_id, step, received in self.qn_source.get(agent_id, [])
                    if received >= dispatch_ros_time_s]
        context["post_dispatch_trajectory_ids"] = post_trajectory
        context["post_dispatch_qn_source"] = qn_source
        context["new_trajectory_observed"] = {
            agent_id: any(trajectory_id != context["pre_dispatch_trajectory_ids"][agent_id]
                          for trajectory_id in post_trajectory[agent_id])
            for agent_id in range(7)}
        context["qn_adopted_trajectory"] = {
            agent_id: any(trajectory_id != context["pre_dispatch_qn_source_trajectory_ids"][agent_id]
                          for trajectory_id, _step in qn_source[agent_id])
            for agent_id in range(7)}
        context["adoption_confirmed_by_runner"] = all(
            context["new_trajectory_observed"][agent_id]
            and context["qn_adopted_trajectory"][agent_id]
            for agent_id in range(7))

    def run(self):
        if self.planning_mode == "executor":
            return self._run_executor()
        self.save()
        try:
            self.wait_ready_or_raise()
            epoch = rospy.Time.now().to_sec()
            self.metrics["mission_epoch"] = epoch
            self.metrics["status"] = "RUNNING"
            previous_actual_finish = 0.0
            while any(i.status == "PLANNED" for i in self.plan.items):
                item = next(i for i in self.plan.items if i.status == "PLANNED")
                task = self.tasks_by_id[item.task_id]
                while (not rospy.is_shutdown()
                       and rospy.Time.now().to_sec() < epoch + item.planned_start):
                    rospy.sleep(min(0.05, max(
                        0.0, epoch + item.planned_start - rospy.Time.now().to_sec())))
                if rospy.is_shutdown():
                    raise RuntimeError("ROS shutdown before dispatch")
                unit = unit_for_coalition(self.routing, item.coalition)
                if unit is None:
                    raise RuntimeError(
                        "no dispatchable executor owns the coalition the plan "
                        "allocated to {}: {}".format(item.task_id, tuple(item.coalition)))
                conflict = conflicting_active_unit(
                    self.routing, self.active_executor_ids, unit.executor_id)
                if conflict is not None:
                    raise RuntimeError(
                        "{} cannot be dispatched while {} is active: they share "
                        "physical members {}".format(
                            unit.executor_id, conflict.executor_id,
                            sorted(conflict.members() & unit.members())))
                context = self.execution_context(item, epoch, previous_actual_finish, unit)
                dispatched_start, dispatched_finish = item.planned_start, item.planned_finish
                goal = FormationGoal()
                goal.task_id = task.task_id
                goal.formation_center.header.frame_id = "world"
                goal.formation_center.header.stamp = rospy.Time.now()
                point = goal.formation_center.point
                point.x, point.y, point.z = self.centers[task.target_ref]
                goal.hold_duration = rospy.Duration.from_sec(task.service_time)
                item.status = "RUNNING"
                # Unknown/failed outcomes retain ownership of the physical members.
                self.active_executor_ids.add(unit.executor_id)
                state, result, dispatch_ros_time_s, timed_out, attempts = \
                    self.dispatch_goal_with_retry(goal, task)
                if timed_out:
                    item.status = "UNKNOWN_LOCKED"
                    self.metrics["status"] = "UNKNOWN_LOCKED"
                    self.metrics["failure"] = (
                        "Action result timeout for {}".format(task.task_id))
                    break
                if state == GoalStatus.REJECTED:
                    # P1: a busy/not-ready server rejects instead of preempting.
                    # The same task is retried after READY_IDLE; only a server
                    # that never returns to READY_IDLE is a run-level fault.
                    item.status = "UNKNOWN_LOCKED"
                    self.metrics["status"] = "UNKNOWN_LOCKED"
                    self.metrics["failure"] = (
                        "{} rejected after {} attempts; server did not return to "
                        "READY_IDLE".format(task.task_id, attempts))
                    break
                goal_id, envelope = self.native_result(task.task_id)
                evidence = self.action_evidence(goal_id)
                execution = dict(context)
                execution.update(
                    task_id=task.task_id,
                    goal_id=goal_id,
                    terminal_status=state,
                    envelope_status=envelope.status.status,
                    goal_dispatch_ros_time_s=dispatch_ros_time_s,
                    dispatch_ros_time_s=dispatch_ros_time_s - epoch,
                    planned_start_at_dispatch=dispatched_start,
                    planned_finish_at_dispatch=dispatched_finish,
                    action_reason=result.reason if result is not None else None,
                    action_task_outcome=(result.task_outcome if result is not None else None),
                    action_safety_outcome=(result.safety_outcome if result is not None else None),
                    action_experiment_validity=(
                        result.experiment_validity if result is not None else None),
                    model_time_hold_seconds=(
                        result.model_time_hold_seconds if result is not None else None),
                    planner_nominal_finish_times=(
                        evidence.get("planner_nominal_finish_times") if evidence else None),
                    adoption_verdict=(evidence.get("adoption_verdict") if evidence else None),
                    verdict=(evidence.get("verdict") if evidence else None),
                    evidence_file=(evidence.get("evidence_file") if evidence else None),
                )
                result_finish_s = rospy.Time.now().to_sec()
                self.collect_post_evidence(context, dispatch_ros_time_s, result_finish_s)
                # plan.md P2: every action records the trajectory_id set that
                # belongs to it.  It can only be assembled after the
                # post-dispatch observation has been collected.
                execution["trajectory_ids"] = sorted({
                    trajectory_id
                    for ids in context.get("post_dispatch_trajectory_ids", {}).values()
                    for trajectory_id in ids})
                execution["runner_adoption_confirmed"] = context[
                    "adoption_confirmed_by_runner"]
                execution["executor_matches_execution"] = (
                    execution["assigned_members"] == execution["executed_members"])
                if state != envelope.status.status:
                    execution["evidence_conflict"] = "Action status/envelope mismatch"
                if result is None:
                    execution["evidence_conflict"] = "Action returned no result"
                if execution.get("evidence_conflict"):
                    self.metrics["executions"].append(execution)
                    self.metrics["status"] = "INVALID"
                    self.metrics["failure"] = execution["evidence_conflict"]
                    break
                actual_start = result.actual_start_time.to_sec() - epoch
                actual_finish = result.actual_finish_time.to_sec() - epoch
                if not (math.isfinite(actual_start) and math.isfinite(actual_finish)
                        and actual_finish >= actual_start >= 0):
                    execution["evidence_conflict"] = "invalid Action timestamps"
                    self.metrics["executions"].append(execution)
                    self.metrics["status"] = "INVALID"
                    self.metrics["failure"] = execution["evidence_conflict"]
                    break
                nominal = (execution.get("planner_nominal_finish_times") or {})
                execution["planner_nominal_finish"] = (
                    max(nominal.values()) - epoch if nominal else None)
                execution.update(
                    actual_start=actual_start, actual_finish=actual_finish,
                    delay_seconds=actual_finish - dispatched_finish,
                    deadline_lateness=max(0.0, actual_finish - task.deadline),
                    scheduled_start_at_dispatch=dispatched_start,
                    expected_release_without_repair=max(
                        previous_actual_finish, context["initial_planned_start"]),
                )
                # How far the dispatch landed from the release boundary it was
                # waiting for.  This is a schedule-tracking figure, not evidence
                # that repair changed anything.
                execution["release_lag_s"] = (
                    execution["dispatch_ros_time_s"]
                    - execution["expected_release_without_repair"])
                execution["dispatch_changed"] = (
                    abs(execution["release_lag_s"]) > self.tolerance)
                self.metrics["executions"].append(execution)
                if state != GoalStatus.SUCCEEDED or result.reason != 0:
                    item.status = "FAILED"
                    self.metrics["status"] = "FAILED"
                    self.metrics["failure"] = "{} ended with status={} reason={}".format(
                        task.task_id, state, result.reason)
                    break
                if not context["adoption_confirmed_by_runner"]:
                    item.status = "REFERENCE_ADOPTION_UNCONFIRMED"
                    self.metrics["status"] = "INCOMPLETE"
                    self.metrics["failure"] = (
                        "runner could not confirm trajectory ownership for {} at "
                        "goal_dispatch_time={:.3f}".format(
                            task.task_id, dispatch_ros_time_s))
                    break
                previous_actual_finish = actual_finish
                if self.mode == "mission":
                    event = DelayEvent(
                        goal_id + ":completion", item.execution_id, task.task_id,
                        dispatched_finish, actual_finish)
                    before = asdict(self.plan)
                    self.plan, changed = process_completion(
                        self.plan, event, self.final_events, tolerance=self.tolerance,
                        repair_timing=(self.repair_mode == "on"))
                    execution["plan_updated"] = changed
                    self.plan_revision += 1
                    execution["plan_revision_after_completion"] = self.plan_revision
                    self.metrics["plan_history"].append({
                        "event_id": event.event_id, "plan_before": before,
                        "plan": asdict(self.plan), "plan_updated": changed,
                        "plan_revision": self.plan_revision})
                    execution["plan_before"] = before
                    execution["plan_after"] = asdict(self.plan)
                    execution["repair_timing"] = self.repair_mode == "on"
                else:
                    item.status = "COMPLETED"
                    execution["plan_updated"] = False
                    execution["plan_revision_after_completion"] = self.plan_revision
                self.active_executor_ids.discard(unit.executor_id)
                rospy.loginfo(
                    "%s complete actual=%.3f planned=%.3f delay=%.3f "
                    "plan_updated=%s dispatch_changed=%s",
                    task.task_id, actual_finish, dispatched_finish,
                    actual_finish - dispatched_finish,
                    execution.get("plan_updated"), execution.get("dispatch_changed"))
                self.save()
            if self.metrics["status"] == "RUNNING":
                self.metrics["actual_makespan"] = max(
                    (e["actual_finish"] for e in self.metrics["executions"]), default=None)
                self.metrics["status"] = "PASS"
            self._summarise_test_c()
        except Exception as error:
            self.metrics["status"] = self.metrics.get("status", "FAILED")
            if self.metrics["status"] == "RUNNING":
                self.metrics["status"] = "FAILED"
            self.metrics["failure"] = str(error)
            raise
        finally:
            self.save()

    def _plan_in_use(self, dispatched_start):
        """True when the next planned item reflects the updated Plan object."""
        remaining = [i for i in self.plan.items if i.status == "PLANNED"]
        if not remaining:
            return True
        after = [i for i in self.plan.items
                 if i.status == "PLANNED" and i.planned_start >= dispatched_start - 1e-6]
        return len(after) == len(remaining)

    def _summarise_test_c(self):
        executions = self.metrics["executions"]
        # A dispatch proves it used the updated plan only by having read the
        # revision the previous completion produced.
        for index, execution in enumerate(executions):
            if index == 0:
                execution["updated_plan_used"] = True
                continue
            previous = executions[index - 1]
            execution["updated_plan_used"] = (
                execution.get("plan_revision_at_dispatch")
                == previous.get("plan_revision_after_completion"))
        self.metrics["test_c"] = {
            "repair_mode": self.repair_mode,
            "plan_revision": self.plan_revision,
            "plan_updated": any(e.get("plan_updated") for e in executions),
            "plan_updated_actions": [e["task_id"] for e in executions
                                     if e.get("plan_updated")],
            "updated_plan_used": all(e.get("updated_plan_used", True) for e in executions),
            "dispatch_changed": any(e.get("dispatch_changed") for e in executions),
            "dispatch_changed_actions": [e["task_id"] for e in executions
                                         if e.get("dispatch_changed")],
            "release_lag_s": {e["task_id"]: e.get("release_lag_s")
                              for e in executions},
            "notes": (
                "release_lag_s is the dispatch offset from the release boundary "
                "it was waiting for; it is a schedule-tracking figure, not a "
                "measure of repair.  updated_plan_used reports that each "
                "dispatch read the plan revision the completion produced."),
        }


def main():
    rospy.init_node("formation_mission_runner")
    runner = MissionRunner()
    runner.run()
    if runner.metrics["status"] not in ("PASS", "PASS_GEOMETRIC_PROXY", "NOT_CONFIRMED"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
