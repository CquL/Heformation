#!/usr/bin/env python3
"""One native Swarm goal, with completion established by actual qn state.

Design notes (plan.md P0/P1):

* the process starts immediately, but only ``READY_IDLE`` accepts a Goal; the
  goal callback validates, reserves the group resource atomically and hands the
  work to a single execution loop, then returns,
* a running task is never preempted by a new goal, and a cancel request never
  releases resources or ends the physical task,
* odometry loss, execution timeout and result timeout enter ``UNKNOWN_LOCKED``,
* task outcome, safety outcome and experiment validity are reported separately,
* the new task reference must be proven by ``trajectory_id`` ownership.
"""

import csv
from collections import deque
import json
import math
import queue
import re
import threading
import time
import traceback
import xmlrpc.client
from dataclasses import asdict, dataclass
from pathlib import Path

import actionlib
import rosgraph
import rospy
from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

from qn_aav_simulator.action_lifecycle import (
    ACCEPTED, ActionResourceStateMachine, REJECTED_INVALID,
)
from qn_aav_simulator.experiment_verdict import (
    MemberSample, SAFETY_PASS, TASK_PASS, VALIDITY_VALID, box_surface_clearance,
    build_ledger, compute_metrics, decide, evaluate_safety,
)
from qn_aav_simulator.formation_monitor import (
    AGENT_IDS, DEFAULT_RELATIVE_SLOTS, GroupCompletionMonitor, OdometrySample,
    validate_configuration, validate_target,
)
from qn_aav_simulator.odometry import OdometryContractError, parse_standard_odometry
from qn_aav_simulator.readiness import (
    GLOBAL_MAP_TOPIC, ReadinessEvaluator, RuntimeHealthMonitor, SENSOR_BACKEND_CPU,
    SENSOR_BACKEND_CUDA, depth_topic, local_cloud_topic, odometry_topic,
    planner_finish_topic, position_command_topic, qn_diagnostics_topic,
)
from qn_aav_simulator.time_alignment import (
    ModelTimeHistory, ModelTimeSample, TimeAlignmentMonitor,
)
from qn_aav_simulator.trajectory_adoption import TrajectoryAdoptionTracker
from qn_aav_simulator.msg import FormationAction, FormationFeedback, FormationResult

try:  # optional; only accelerates the discrete obstacle-clearance sample
    import numpy
except Exception:  # pragma: no cover - numpy ships with the ROS image
    numpy = None

_MAX_CLOUD_POINTS = 20000
_ACTIVE_STATUS = (GoalStatus.ACTIVE, GoalStatus.PREEMPTING)


def used_reference_pose_topic(agent_id):
    return "/drone_{}_qn/used_reference_pose".format(agent_id)


def used_reference_twist_topic(agent_id):
    return "/drone_{}_qn/used_reference_twist".format(agent_id)


class _TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


@dataclass
class WorkItem:
    goal_handle: object
    goal: object
    received_ros_time_s: float


class FormationActionServer:
    def __init__(self):
        rospy.set_param("~ready", False)
        rospy.set_param("~readiness_reason", "BOOTING")
        self.agent_ids = list(rospy.get_param("~agent_ids", list(AGENT_IDS)))
        self.scale = float(rospy.get_param("~swarm_scale", 2.0))
        self.epsilon_p = float(rospy.get_param("~epsilon_p", 0.5))
        self.epsilon_v = float(rospy.get_param("~epsilon_v", 0.25))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.25))
        self.execution_timeout = float(rospy.get_param("~execution_timeout", 180.0))
        self.monitor_rate = float(rospy.get_param("~monitor_rate", 20.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 120.0))
        self.sensor_backend = str(rospy.get_param("~sensor_backend", SENSOR_BACKEND_CPU)).upper()
        self.known_empty_map = bool(rospy.get_param("~known_empty_map", False))
        self.cloud_timeout = float(rospy.get_param("~cloud_timeout", 2.0))
        self.qn_state_timeout = float(rospy.get_param("~qn_state_timeout", 1.0))
        self.model_time_window = float(rospy.get_param("~model_time_window", 0.06))
        self.time_baseline_seconds = float(rospy.get_param("~time_baseline_seconds", 30.0))
        self.time_rate_lower = float(rospy.get_param("~time_rate_lower", 0.95))
        self.time_rate_upper = float(rospy.get_param("~time_rate_upper", 1.05))
        self.max_model_ros_drift = float(rospy.get_param("~max_model_ros_drift", 0.05))
        self.max_cross_agent_drift = float(rospy.get_param("~max_cross_agent_drift", 0.05))
        self.time_reference_displacement_limit = float(
            rospy.get_param("~time_reference_displacement_limit", 0.5 * self.epsilon_p))
        self.inter_agent_clearance = float(rospy.get_param("~inter_agent_clearance", 0.5))
        self.obstacle_clearance = float(rospy.get_param("~obstacle_clearance", 0.2))
        self.obstacle_sample_period = float(rospy.get_param("~obstacle_sample_period", 1.0))
        # Physical envelope of one platform and the declared surface plane.
        # Surface clearance is a centre distance with both envelopes removed;
        # the AIR floor itself is derived from the qn model, not configured.
        # Declared cruise altitude of the scenario.  The replan FSM honours the
        # goal's z (build-time patch), so this value reaches the planner too.
        self.cruise_altitude_m = float(rospy.get_param("~cruise_altitude_m", 0.5))
        self.platform_radius_m = float(rospy.get_param("~platform_radius_m", 0.25))
        self.surface_plane_m = float(rospy.get_param("~surface_plane_m", 0.0))
        # One scene definition, shared with the publisher, the renderers and the
        # verifier.  The box is the *actual* geometric truth for the clearance
        # check; the point cloud stays a diagnostic.
        scene = rospy.get_param("/scene", {})
        self.map_topic = str(rospy.get_param(
            "~global_map_topic", scene.get("topic", GLOBAL_MAP_TOPIC)))
        self.scene_source = bool(rospy.get_param(
            "~scene_source", bool(scene)))
        self.obstacle_present = bool(rospy.get_param(
            "~obstacle_present", scene.get("obstacle_present", False)))
        self.obstacle_center = [float(value) for value in rospy.get_param(
            "~obstacle_center", scene.get("obstacle_center", [-23.0, 0.0, 0.5]))]
        self.obstacle_size = [float(value) for value in rospy.get_param(
            "~obstacle_size", scene.get("obstacle_size", [1.0, 1.0, 1.2]))]
        # Declared before the run: the reference path must leave this much extra
        # room beyond the required clearance.  It is only used for the reference
        # margin analysis and never added to the actual safety threshold.
        self.tracking_budget_m = float(rospy.get_param("~tracking_budget_m", 0.30))
        self.min_valid_sample_ratio = float(rospy.get_param("~min_valid_sample_ratio", 0.9))
        self.slots = validate_configuration(
            self.agent_ids,
            rospy.get_param("~relative_slots", {
                str(key): list(value) for key, value in DEFAULT_RELATIVE_SLOTS.items()}),
            self.scale, self.epsilon_p, self.epsilon_v,
            self.odom_timeout, self.execution_timeout)
        for name, value in (("monitor_rate", self.monitor_rate),
                            ("startup_timeout", self.startup_timeout),
                            ("cloud_timeout", self.cloud_timeout),
                            ("qn_state_timeout", self.qn_state_timeout),
                            ("model_time_window", self.model_time_window),
                            ("time_baseline_seconds", self.time_baseline_seconds),
                            ("obstacle_sample_period", self.obstacle_sample_period)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        for name, value in (("platform_radius_m", self.platform_radius_m),
                            ("surface_plane_m", self.surface_plane_m)):
            if not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        if self.platform_radius_m < 0.0:
            raise ValueError("platform_radius_m must be non-negative")
        if self.sensor_backend not in (SENSOR_BACKEND_CPU, SENSOR_BACKEND_CUDA):
            raise ValueError("sensor_backend must be {} or {}".format(
                SENSOR_BACKEND_CPU, SENSOR_BACKEND_CUDA))
        self.output_dir = Path(rospy.get_param("~output_dir", "/experiments/current"))
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.RLock()
        self.odom = {}
        # Recent per-member odometry (about 2 s at 100 Hz).  The completion
        # monitor evaluates the settled condition over this window instead of
        # only the newest sample, so a monitor tick cannot hide a transient
        # out-of-tolerance state between ticks.
        self.odom_history = {
            agent_id: deque(maxlen=256) for agent_id in self.agent_ids}
        self.odom_counts = {agent_id: 0 for agent_id in self.agent_ids}
        self.odometry_errors = {}
        self.model_times = ModelTimeHistory(
            ["drone_{}".format(agent_id) for agent_id in self.agent_ids])
        self.used_reference = {}
        self.command_trajectory = {}
        self.qn_source = {}
        self.planner_finish = {}
        self.map_cloud = None
        self.group_goal_messages = 0
        self.active_diagnostics = None
        self.baseline_snapshot = None
        self.air_state = {}
        self.air_domain_violation = False
        self.air_floor_m = None
        self.max_diagnostics_callback_lag_s = 0.0

        self.state_machine = ActionResourceStateMachine(lock=self.lock)
        self.readiness = ReadinessEvaluator(
            self.agent_ids, sensor_backend=self.sensor_backend,
            known_empty_map=self.known_empty_map, scene_source=self.scene_source,
            map_topic=self.map_topic, cloud_timeout_s=self.cloud_timeout,
            odometry_timeout_s=self.odom_timeout,
            diagnostics_timeout_s=self.qn_state_timeout)
        self.health = RuntimeHealthMonitor(self.readiness,
                                           odometry_timeout_s=self.odom_timeout)
        self.session_alignment = self._new_alignment_monitor()
        self._alignment_fed_ros = {
            "drone_{}".format(agent_id): 0.0 for agent_id in self.agent_ids}
        self._readiness_cache = None
        self._readiness_cache_time = 0.0

        self.master = rosgraph.Master(rospy.get_name())
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(self.map_topic, PointCloud2, self._map_callback,
                             queue_size=1),
            rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                             self._group_goal_observer, queue_size=10),
        ]
        sensing_type = (PointCloud2 if self.sensor_backend == SENSOR_BACKEND_CPU
                        else Image)
        for agent_id in self.agent_ids:
            sensing_topic = (local_cloud_topic(agent_id)
                             if self.sensor_backend == SENSOR_BACKEND_CPU
                             else depth_topic(agent_id))
            self.subscribers.extend([
                rospy.Subscriber(odometry_topic(agent_id), Odometry,
                                 self._standard_odom_callback,
                                 callback_args=agent_id, queue_size=1),
                rospy.Subscriber(qn_diagnostics_topic(agent_id), DiagnosticArray,
                                 self._diagnostics_callback,
                                 callback_args=agent_id, queue_size=5),
                rospy.Subscriber(position_command_topic(agent_id), PositionCommand,
                                 self._command_callback,
                                 callback_args=agent_id, queue_size=5),
                rospy.Subscriber(planner_finish_topic(agent_id), Bool,
                                 self._finish_callback,
                                 callback_args=agent_id, queue_size=10),
                rospy.Subscriber(used_reference_pose_topic(agent_id), PoseStamped,
                                 self._used_reference_pose_callback,
                                 callback_args=agent_id, queue_size=5),
                rospy.Subscriber(used_reference_twist_topic(agent_id), TwistStamped,
                                 self._used_reference_twist_callback,
                                 callback_args=agent_id, queue_size=5),
                rospy.Subscriber(sensing_topic, sensing_type,
                                 self._sensing_callback,
                                 callback_args=agent_id, queue_size=1),
            ])
        self.action_server = actionlib.ActionServer(
            "formation_action", FormationAction,
            self._goal_callback, self._cancel_callback, auto_start=False)
        self.work_queue = queue.Queue()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.readiness_thread = threading.Thread(target=self._readiness_loop, daemon=True)
        self.action_server.start()
        self.worker.start()
        self.readiness_thread.start()
        rospy.loginfo("FormationAction started; goals are rejected until READY_IDLE")

    # ------------------------------------------------------------- helpers
    def _new_alignment_monitor(self):
        return TimeAlignmentMonitor(
            ["drone_{}".format(agent_id) for agent_id in self.agent_ids],
            baseline_seconds=self.time_baseline_seconds,
            rate_lower=self.time_rate_lower, rate_upper=self.time_rate_upper,
            max_abs_drift_s=self.max_model_ros_drift,
            max_cross_agent_drift_s=self.max_cross_agent_drift,
            alignment_window_s=self.model_time_window)

    # -------------------------------------------------------- observations
    def _standard_odom_callback(self, message, agent_id):
        agent = "drone_{}".format(agent_id)
        try:
            sample = parse_standard_odometry(message, agent)
        except OdometryContractError as error:
            with self.lock:
                self.odometry_errors[agent_id] = str(error)
            return
        with self.lock:
            self.odom[agent_id] = sample
            self.odom_history[agent_id].append(sample)
            self.odom_counts[agent_id] += 1
            self.odometry_errors.pop(agent_id, None)
            self.readiness.note_message(odometry_topic(agent_id),
                                        rospy.Time.now().to_sec())

    def _diagnostics_callback(self, message, agent_id):
        values = {}
        for status in message.status:
            for entry in status.values:
                values[entry.key] = entry.value
        if "model_time_s" not in values:
            return
        agent = "drone_{}".format(agent_id)
        # The model time in this message belongs to the ROS time stamped by its
        # publisher, not to the moment this server happened to dequeue it.
        # Aligning on receive time would measure this process' scheduling lag.
        received = rospy.Time.now().to_sec()
        stamp = float(message.header.stamp.to_sec())
        if not math.isfinite(stamp) or stamp <= 0.0:
            stamp = received
        model_time = float(values["model_time_s"])
        sample = ModelTimeSample(agent, stamp, model_time, received)
        with self.lock:
            self.model_times.note(sample)
            self.session_alignment.note_sample(sample)
            self.readiness.note_message(qn_diagnostics_topic(agent_id), received)
            self.max_diagnostics_callback_lag_s = max(
                self.max_diagnostics_callback_lag_s, received - stamp)
            self.qn_source[agent_id] = {
                "source_trajectory_id": int(values.get("source_trajectory_id", -1)),
                "used_outer_step": int(values.get("used_outer_step", -1)),
                "source_command_stamp": float(values.get("source_command_stamp", "nan")),
                "model_time_s": model_time,
                "ros_time_s": received,
                "stamp_s": stamp,
                "velocity_directly_consumed": values.get("velocity_directly_consumed"),
                "acceleration_directly_consumed": values.get(
                    "acceleration_directly_consumed"),
                "inner_backend_update_time_s": received,
            }
            self._latch_air_domain(agent_id, values)
            diagnostics = self.active_diagnostics
        if diagnostics is not None:
            diagnostics["last_qn_state_update"] = {
                "agent_id": agent_id, "ros_time_s": received,
                "message_stamp_s": stamp, "model_time_s": model_time}

    def _command_callback(self, message, agent_id):
        ros_time = rospy.Time.now().to_sec()
        with self.lock:
            self.command_trajectory[agent_id] = (
                int(message.trajectory_id), float(message.header.stamp.to_sec()), ros_time)
        self._feed_adoption(agent_id, ros_time)

    def _used_reference_pose_callback(self, message, agent_id):
        with self.lock:
            entry = self.used_reference.setdefault(agent_id, {})
            entry["position"] = (float(message.pose.position.x),
                                 float(message.pose.position.y),
                                 float(message.pose.position.z))
            entry["ros_time_s"] = rospy.Time.now().to_sec()
            entry["stamp_s"] = float(message.header.stamp.to_sec())

    def _used_reference_twist_callback(self, message, agent_id):
        with self.lock:
            entry = self.used_reference.setdefault(agent_id, {})
            entry["velocity"] = (float(message.twist.linear.x),
                                 float(message.twist.linear.y),
                                 float(message.twist.linear.z))

    def _latch_air_domain(self, agent_id: int, values) -> None:
        """Latch the observed AIR-domain state of one member.

        The qn model switches mass, inertia, damping and actuation by
        ``medium_flag``, so any non-zero flag means the run is no longer the AIR
        experiment it claims to be.  The exact duration and minimum are computed
        offline from the recorded diagnostics; what has to stay online is the
        fact that a violation happened, before the next task can be dispatched.
        """
        def number(key, default=None):
            try:
                value = float(values.get(key, "nan"))
            except (TypeError, ValueError):
                return default
            return value if math.isfinite(value) else default

        flag = number("max_medium_flag")
        height = number("min_height_m")
        floor = number("air_floor_m")
        if floor is not None:
            self.air_floor_m = floor
        self.air_state[agent_id] = {
            "max_medium_flag": flag, "min_height_m": height,
            "air_floor_m": floor,
            "air_domain_violation": str(
                values.get("air_domain_violation", "false")).lower() == "true",
        }
        if self.air_state[agent_id]["air_domain_violation"]:
            self.air_domain_violation = True
        if flag is not None and flag > 0.0:
            self.air_domain_violation = True
        if height is not None and floor is not None and height < floor:
            self.air_domain_violation = True

    def _air_domain_summary(self):
        """The latched AIR-domain result of the whole coalition."""
        flags = [entry.get("max_medium_flag") for entry in self.air_state.values()]
        heights = [entry.get("min_height_m") for entry in self.air_state.values()]
        floors = [entry.get("air_floor_m") for entry in self.air_state.values()]
        observed_flags = [value for value in flags if value is not None]
        observed_heights = [value for value in heights if value is not None]
        observed_floors = [value for value in floors if value is not None]
        detail = ""
        if self.air_domain_violation:
            detail = (
                "model domain: the run left the AIR model "
                "(max_medium_flag={}, min_height_m={}, air_floor_m={})".format(
                    max(observed_flags) if observed_flags else None,
                    min(observed_heights) if observed_heights else None,
                    (0.5 * max(observed_floors) if observed_floors
                     else (self.air_floor_m if self.air_floor_m is not None else None))))
        return {
            "ok": not self.air_domain_violation,
            "detail": detail,
            "max_medium_flag": max(observed_flags) if observed_flags else None,
            "min_height_m": min(observed_heights) if observed_heights else None,
            "air_floor_m": self.air_floor_m,
            "members_reporting": len(self.air_state),
        }

    def _feed_adoption(self, agent_id, now_s):
        with self.lock:
            diagnostics = self.active_diagnostics
            if diagnostics is None or "adoption" not in diagnostics:
                return
            tracker = diagnostics["adoption"]
            trajectory = self.command_trajectory.get(agent_id)
            source = self.qn_source.get(agent_id)
        if trajectory is not None:
            tracker.note_position_command(agent_id, trajectory[0], trajectory[1], now_s)
        if source is not None and source["source_trajectory_id"] >= 0:
            tracker.note_qn_source(
                agent_id, source["source_trajectory_id"], source["used_outer_step"],
                source["ros_time_s"], source["source_command_stamp"])

    def _map_callback(self, message):
        points = self._downsample_cloud(message)
        with self.lock:
            self.map_cloud = points
            self.readiness.note_message(
                self.map_topic, rospy.Time.now().to_sec(),
                valid=points is not None, empty=(points is not None and len(points) == 0))

    def _sensing_callback(self, message, agent_id):
        topic = (local_cloud_topic(agent_id)
                 if self.sensor_backend == SENSOR_BACKEND_CPU
                 else depth_topic(agent_id))
        if isinstance(message, PointCloud2):
            try:
                count = int(message.width) * int(message.height)
            except Exception:
                count = -1
            empty = count == 0
            valid = count >= 0
        else:
            empty = False
            valid = True
        with self.lock:
            self.readiness.note_message(
                topic, rospy.Time.now().to_sec(), valid=valid, empty=empty)

    def _group_goal_observer(self, message):
        header = getattr(message, "_connection_header", None) or {}
        publisher = header.get("callerid") if isinstance(header, dict) else None
        with self.lock:
            self.group_goal_messages += 1
            diagnostics = self.active_diagnostics
            tracker = diagnostics.get("adoption") if diagnostics else None
        if tracker is not None:
            # Counted from the observed topic, not from our own publish call:
            # a foreign publisher emitting a group goal must not go unnoticed.
            tracker.note_group_goal(publisher)

    def _finish_callback(self, message, agent_id):
        if not message.data:
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.planner_finish[agent_id] = now
            diagnostics = self.active_diagnostics
            if diagnostics is not None and now >= diagnostics["actual_start_time"]:
                diagnostics["planner_nominal_finish_times"].setdefault(str(agent_id), now)

    @staticmethod
    def _downsample_cloud(message):
        if numpy is None:
            return None
        try:
            count = int(message.width) * int(message.height)
        except Exception:
            return None
        if count <= 0:
            return numpy.zeros((0, 3))
        try:
            point_step = int(message.point_step)
            if point_step <= 0 or len(message.data) < count * point_step:
                return None
            raw = numpy.frombuffer(message.data, dtype=numpy.uint8)
            points = raw[:count * point_step].reshape(count, point_step)
            xyz = points[:, :12].copy().view(numpy.float32).reshape(count, 3)
            step = max(1, count // _MAX_CLOUD_POINTS)
            xyz = xyz[::step]
            finite = numpy.isfinite(xyz).all(axis=1)
            return xyz[finite]
        except Exception:
            return None

    # ---------------------------------------------------------- readiness
    def _planner_configuration_matches(self):
        for agent_id in self.agent_ids:
            prefix = "/drone_{}_ego_planner_node/".format(agent_id)
            configuration = rospy.get_param(prefix.rstrip("/"), {})
            if configuration.get("fsm", {}).get("flight_type") != 3:
                return "planner {} is not in formation goal mode".format(agent_id)
            if configuration.get("manager", {}).get("drone_id") != agent_id:
                return "planner {} has a different drone ID".format(agent_id)
            global_goal = configuration.get("global_goal", {})
            if not math.isclose(float(global_goal.get("swarm_scale", float("nan"))),
                                self.scale, rel_tol=0.0, abs_tol=1e-9):
                return "planner {} swarm_scale differs from monitor".format(agent_id)
            for member_id, slot in self.slots.items():
                for axis, value in zip(("x", "y", "z"), slot):
                    actual = global_goal.get(
                        "relative_pos_{}".format(member_id), {}).get(axis, float("nan"))
                    if not math.isclose(float(actual), value, rel_tol=0.0, abs_tol=1e-9):
                        return "planner {} relative slots differ from monitor".format(agent_id)
        return "OK"

    def _planner_interface_health(self):
        mismatch = self._planner_configuration_matches()
        if mismatch != "OK":
            return mismatch
        try:
            publishers, subscribers, _services = self.master.getSystemState()
        except Exception as error:
            return "cannot read the ROS master state: {}".format(error)
        publishers, subscribers = dict(publishers), dict(subscribers)
        expected = {"/drone_{}_ego_planner_node".format(agent_id)
                    for agent_id in self.agent_ids}
        if not expected <= set(subscribers.get("/move_base_simple/goal", [])):
            return "waiting for all seven planner goal subscriptions"
        for agent_id in self.agent_ids:
            topic = odometry_topic(agent_id)
            if set(publishers.get(topic, [])) != {"/drone_{}_qn_aav".format(agent_id)}:
                return "odometry {} must have exactly its qn publisher".format(topic)
        try:
            uri = self.master.lookupNode(rospy.get_name())
            with xmlrpc.client.ServerProxy(uri, transport=_TimeoutTransport()) as proxy:
                code, _message, connections = proxy.getBusInfo(rospy.get_name())
        except Exception as error:
            return "cannot inspect goal connections: {}".format(error)
        connected = {connection[1] for connection in connections
                     if len(connection) >= 6 and connection[2] == "o"
                     and connection[4] == "/move_base_simple/goal" and connection[5]}
        if code != 1 or not expected <= connected:
            return "waiting for established TCPROS goal connections to all seven planners"
        return "OK"

    def _refresh_topic_existence(self):
        try:
            # get_published_topics() returns [[name, type], ...]; a set of the
            # raw entries would fail because lists are unhashable.
            published = {topic for topic, _type in rospy.get_published_topics()}
        except Exception as error:
            rospy.logwarn_throttle(5.0, "cannot list published topics: %s", error)
            return
        with self.lock:
            for topic in list(self.readiness.topics):
                self.readiness.note_topic_exists(topic, topic in published)

    def _cached_baseline_report(self):
        """The baseline the run was admitted with, or a fresh report."""
        if self.baseline_snapshot is not None:
            return self.baseline_snapshot
        return self._baseline_report()

    def _baseline_report(self):
        # The monitor protects its own history and copies it before computing;
        # holding the server lock here used to block every subscription
        # callback for seconds once the sample history grew.
        return self.session_alignment.report()

    def _readiness_snapshot(self, now=None):
        # Message age must be measured against a clock read *after* the slow
        # planner-health XML-RPC calls below.  Evaluating with a `now` captured
        # before them made every message that arrived during the call look
        # future-stamped (age < 0) and therefore "stale": messages were flowing
        # at ~50-90 Hz while READY_IDLE could never be reached.
        if now is None:
            now = rospy.Time.now().to_sec()
        with self.lock:
            if (self._readiness_cache is not None
                    and now - self._readiness_cache_time < 1.0):
                return self._readiness_cache
        planner_health = self._planner_interface_health()
        baseline = self._baseline_report()
        evaluated_at = rospy.Time.now().to_sec()
        health = self.health.evaluate(evaluated_at)
        with self.lock:
            # The readiness evaluator gates on its own planner_health field:
            # without this wiring every evaluation would report "planner
            # health UNKNOWN" and READY_IDLE could never be reached.
            self.readiness.note_planner_health(planner_health)
            status = self.readiness.evaluate(evaluated_at)
            snapshot = status.as_dict()
            snapshot["odometry_contract_errors"] = dict(self.odometry_errors)
        snapshot["planner_health"] = planner_health
        snapshot["runtime_health"] = health
        reasons = []
        if not status.ready:
            reasons.append(status.reason)
        if planner_health != "OK":
            reasons.append("planner_health: " + planner_health)
        if not baseline.within_thresholds:
            reasons.append("time baseline not qualified: " + "; ".join(baseline.reasons))
        snapshot["ready"] = not reasons
        snapshot["reason"] = "ready" if not reasons else "; ".join(reasons)
        snapshot["time_baseline"] = baseline.as_dict()
        if snapshot["ready"] and self.baseline_snapshot is None:
            # The qualification that admitted this run is the one that counts.
            # The sample history slides, so recomputing it later would report a
            # shorter window than the one the gate actually accepted.
            self.baseline_snapshot = baseline
            rospy.loginfo(
                "time baseline qualified: %.1f s, max_abs_drift=%.6f s, "
                "max_cross_agent_drift=%.6f s",
                baseline.duration_s or 0.0,
                baseline.max_abs_model_ros_drift_s or 0.0,
                baseline.max_cross_agent_drift_s or 0.0)
        with self.lock:
            snapshot["topic_ages"] = {
                topic: (None if topic_health.last_stamp_s is None
                        else evaluated_at - topic_health.last_stamp_s)
                for topic, topic_health in sorted(self.readiness.topics.items())}
            snapshot["topic_message_counts"] = {
                topic: int(topic_health.message_count)
                for topic, topic_health in sorted(self.readiness.topics.items())}
        with self.lock:
            self._readiness_cache = snapshot
            self._readiness_cache_time = evaluated_at
        return snapshot

    def _readiness_loop(self):
        deadline = time.monotonic() + self.startup_timeout
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            try:
                self._refresh_topic_existence()
                snapshot = self._readiness_snapshot(now)
            except Exception as error:
                snapshot = {"ready": False,
                            "reason": "readiness check: {}".format(error)}
            self.state_machine.ros_time_s = now
            try:
                self.state_machine.note_readiness(bool(snapshot["ready"]))
                rospy.set_param("~ready", bool(snapshot["ready"]))
                rospy.set_param("~readiness_reason", str(snapshot["reason"]))
                rospy.set_param("~sensor_backend",
                                str(snapshot.get("sensor_backend", "")))
                rospy.set_param("~missing_topics",
                                list(snapshot.get("missing_topics", [])))
                rospy.set_param("~stale_topics",
                                list(snapshot.get("stale_topics", [])))
                rospy.set_param("~invalid_inputs",
                                list(snapshot.get("invalid_inputs", [])))
                rospy.set_param("~planner_health", str(snapshot.get("planner_health")))
                rospy.set_param("~runtime_health", snapshot.get("runtime_health", {}))
                rospy.set_param("~topic_ages", json.dumps(
                    self._json_safe(snapshot.get("topic_ages", {}))))
                rospy.set_param("~topic_message_counts", json.dumps(
                    self._json_safe(snapshot.get("topic_message_counts", {}))))
                # XML-RPC cannot marshal None, so the baseline report is carried
                # as JSON text.  A failure here must never kill readiness.
                # Two different facts, kept apart: the whole-run live summary
                # (which keeps measuring reference speed) and the baseline that
                # admitted this run (a one-off snapshot; re-measuring it from the
                # sliding history can read just under the threshold even though
                # the gate accepted it).
                rospy.set_param("~time_alignment_session", json.dumps(
                    self._json_safe(snapshot.get("time_baseline", {}))))
                rospy.set_param("~time_alignment_baseline", json.dumps(
                    self._json_safe(self._cached_baseline_report().as_dict())))
            except Exception as error:
                rospy.logwarn_throttle(
                    5.0, "could not publish readiness parameters: %s", error)
            if not snapshot["ready"]:
                rospy.loginfo_throttle(5.0, "FormationAction not ready: %s",
                                       snapshot["reason"])
                if time.monotonic() >= deadline:
                    rospy.logerr("FormationAction startup timed out: %s",
                                 snapshot["reason"])
                    rospy.signal_shutdown("formation startup timeout: " +
                                          snapshot["reason"])
                    return
            else:
                rospy.loginfo_throttle(30.0, "FormationAction %s",
                                       self.state_machine.state)
            rospy.rostime.wallsleep(0.25)

    # ----------------------------------------------------------- goal entry
    def _validate_goal(self, goal):
        if not str(goal.task_id).strip():
            raise ValueError("task_id must not be empty")
        return validate_target(goal.formation_center.header.frame_id,
                               (goal.formation_center.point.x,
                                goal.formation_center.point.y,
                                goal.formation_center.point.z),
                               goal.hold_duration.to_sec(),
                               self.cruise_altitude_m)

    def _goal_callback(self, goal_handle):
        """Validate, atomically reserve the group resource, hand over, return."""
        goal = goal_handle.get_goal()
        received = rospy.Time.now().to_sec()
        validation_error = None
        try:
            self._validate_goal(goal)
        except (ValueError, TypeError) as error:
            validation_error = str(error)
        status, reason, _ = self.state_machine.request_goal(
            goal_handle.get_goal_id().id, validation_error=validation_error)
        if status == ACCEPTED:
            goal_handle.set_accepted()
            self.work_queue.put(WorkItem(goal_handle, goal, received))
            rospy.loginfo("FormationAction accepted %s (%s)",
                          goal.task_id, goal_handle.get_goal_id().id)
        else:
            result = FormationResult()
            result.task_id = str(goal.task_id)
            result.goal_id = goal_handle.get_goal_id().id
            result.reason = (FormationResult.INVALID_TARGET
                             if status == REJECTED_INVALID
                             else FormationResult.UNKNOWN_LOCKED)
            goal_handle.set_rejected(result, "{}: {}".format(status, reason))
            rospy.logwarn("FormationAction rejected %s: %s %s",
                          goal.task_id, status, reason)

    def _cancel_callback(self, goal_handle):
        """A cancel request never releases resources or ends the physics."""
        self.state_machine.ros_time_s = rospy.Time.now().to_sec()
        running = goal_handle.get_goal_status().status in _ACTIVE_STATUS
        self.state_machine.note_cancel_request()
        rospy.logwarn(
            "FormationAction cancel request observed for %s; running=%s "
            "(actionlib may show PREEMPTING, the physical task is not cancelled)",
            goal_handle.get_goal_id().id, running)

    # --------------------------------------------------------------- worker
    def _worker_loop(self):
        while not rospy.is_shutdown():
            try:
                work = self.work_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._run_task(work)
            except Exception as error:
                rospy.logerr("FormationAction execution failed: %s\n%s",
                             error, traceback.format_exc())
                try:
                    self._fail_task(work, "execution error: {}".format(error))
                except Exception:
                    rospy.logerr("could not report failure: %s", traceback.format_exc())
            finally:
                self.work_queue.task_done()

    def _fail_task(self, work, text):
        diagnostics = self.active_diagnostics
        if diagnostics is None:
            diagnostics = self._new_diagnostics(work, rospy.Time.now())
        self.state_machine.fault(text)
        diagnostics["reason_text"] = text
        self._write_diagnostics(diagnostics)
        result = self._result_message(diagnostics, FormationResult.UNKNOWN_LOCKED)
        work.goal_handle.set_aborted(result, text)

    def _new_diagnostics(self, work, start):
        goal_id = work.goal_handle.get_goal_id().id
        stem = "{}_{}".format(
            re.sub(r"[^A-Za-z0-9_-]", "_", work.goal.task_id)[:80] or "task",
            start.to_nsec())
        path = self.output_dir / (stem + ".diagnostics.json")
        return {
            "task_id": work.goal.task_id,
            "goal_id": goal_id,
            "evidence_file": path.name,
            "diagnostics_path": path,
            "formation_center": [work.goal.formation_center.point.x,
                                 work.goal.formation_center.point.y,
                                 work.goal.formation_center.point.z],
            "hold_duration": work.goal.hold_duration.to_sec(),
            "actual_start_time": start.to_sec(),
            "goal_publish_count": 0,
            "topic_group_goal_count": 0,
            "group_goal_publisher": None,
            "dispatch_ros_time_s": None,
            "planner_nominal_finish_times": {},
            "max_position_error": None,
            "max_velocity": None,
            "min_inter_agent_distance": None,
            "position_not_settled": False,
            "velocity_not_settled": False,
            "successful_hold_window": None,
            "model_hold": None,
            "run_state": "ACTIVE",
        }

    def _result_message(self, diagnostics, reason):
        result = FormationResult()
        result.task_id = diagnostics["task_id"]
        result.goal_id = diagnostics["goal_id"]
        result.evidence_file = diagnostics.get("evidence_file", "")
        result.actual_start_time = rospy.Time.from_sec(diagnostics["actual_start_time"])
        result.actual_finish_time = rospy.Time.from_sec(
            diagnostics.get("actual_finish_time", diagnostics["actual_start_time"]))
        result.reason = reason
        verdict = diagnostics.get("verdict") or {}
        result.task_outcome = (FormationResult.TASK_PASS
                               if verdict.get("task_outcome") == TASK_PASS
                               else FormationResult.TASK_FAIL)
        result.safety_outcome = {
            "PASS": FormationResult.SAFETY_PASS,
            "FAIL": FormationResult.SAFETY_FAIL,
        }.get(verdict.get("safety_outcome"), FormationResult.SAFETY_NOT_VERIFIED)
        result.experiment_validity = {
            "VALID": FormationResult.VALIDITY_VALID,
            "INVALID": FormationResult.VALIDITY_INVALID,
        }.get(verdict.get("experiment_validity"), FormationResult.VALIDITY_INCOMPLETE)
        result.model_time_hold_seconds = float(
            (diagnostics.get("model_hold") or {}).get("min_model_hold_s") or 0.0)
        return result

    @staticmethod
    def _json_safe(value):
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: FormationActionServer._json_safe(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [FormationActionServer._json_safe(item) for item in value]
        if hasattr(value, "as_dict"):
            return value.as_dict()
        return value

    def _write_diagnostics(self, diagnostics):
        payload = {key: value for key, value in diagnostics.items()}
        payload.pop("adoption", None)
        path = payload.pop("diagnostics_path")
        path.write_text(json.dumps(self._json_safe(payload), indent=2,
                                   allow_nan=False) + "\n")
        index_path = self.output_dir / "action_index.json"
        try:
            index = json.loads(index_path.read_text()) if index_path.exists() else {}
        except ValueError:
            index = {}
        index[diagnostics["goal_id"]] = path.name
        temporary = index_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(index, indent=2) + "\n")
        temporary.replace(index_path)

    # ------------------------------------------------------------ execution
    def _run_task(self, work):
        goal = work.goal
        start = rospy.Time.now()
        diagnostics = self._new_diagnostics(work, start)
        monitor = GroupCompletionMonitor(
            diagnostics["formation_center"], goal.hold_duration.to_sec(), start.to_sec(),
            agent_ids=self.agent_ids, relative_slots=self.slots, swarm_scale=self.scale,
            epsilon_p=self.epsilon_p, epsilon_v=self.epsilon_v,
            odom_timeout=self.odom_timeout, execution_timeout=self.execution_timeout,
            platform_radius_m=self.platform_radius_m,
            target_z=self.cruise_altitude_m)
        alignment = self._new_alignment_monitor()
        with self.lock:
            self._alignment_fed_ros = {
                "drone_{}".format(agent_id): start.to_sec()
                for agent_id in self.agent_ids}
        adoption = TrajectoryAdoptionTracker(
            self.agent_ids, authorized_goal_publishers=(rospy.get_name(),))
        diagnostics["adoption"] = adoption
        with self.lock:
            counts_start = dict(self.odom_counts)
            group_goal_start = self.group_goal_messages
            for agent_id in self.agent_ids:
                trajectory = self.command_trajectory.get(agent_id)
                source = self.qn_source.get(agent_id)
                if trajectory is not None:
                    adoption.note_position_command(agent_id, trajectory[0], trajectory[1],
                                                   rospy.Time.now().to_sec())
                if source is not None and source["source_trajectory_id"] >= 0:
                    adoption.note_qn_source(
                        agent_id, source["source_trajectory_id"],
                        source["used_outer_step"], source["ros_time_s"],
                        source["source_command_stamp"])
            self.active_diagnostics = diagnostics

        command = PoseStamped()
        command.header.frame_id = "world"
        command.header.stamp = start
        command.pose.position = goal.formation_center.point
        command.pose.orientation.w = 1.0
        dispatch_time = rospy.Time.now().to_sec()
        adoption.begin_dispatch(goal.task_id, diagnostics["goal_id"], dispatch_time)
        self.goal_pub.publish(command)
        diagnostics["goal_publish_count"] = 1
        diagnostics["dispatch_ros_time_s"] = dispatch_time
        diagnostics["group_goal_publisher"] = rospy.get_name()

        fields = ["ros_time", "elapsed", "phase", "max_position_error", "max_velocity",
                  "min_inter_agent_distance", "fresh_agent_count", "stale_agent_ids",
                  "hold_elapsed", "model_hold_elapsed", "model_hold_pending"]
        member_samples = {agent_id: [] for agent_id in self.agent_ids}
        obstacle_clearances = []
        reference_missing = 0
        previous_phase = None
        last_obstacle_sample = -1.0
        monitor_path = self.output_dir / (
            diagnostics["evidence_file"].replace(".diagnostics.json", ".monitor.csv"))
        try:
            with monitor_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                rate = rospy.Rate(round(self.monitor_rate))
                while not rospy.is_shutdown():
                    with self.lock:
                        samples = {agent_id: list(self.odom_history[agent_id])
                                   for agent_id in self.agent_ids}
                        references = dict(self.used_reference)
                    now = rospy.Time.now()
                    now_s = now.to_sec()
                    snapshot = monitor.evaluate(
                        now_s, samples,
                        model_hold_satisfied=lambda a, b: self._model_hold_elapsed(
                            a, b, diagnostics),
                        hold_not_before_s=adoption.adopted_at_s(),
                        reference_confirmed=(
                            adoption.verdict().state == "ADOPTED"))
                    status = self.state_machine.note_phase(snapshot.phase)
                    diagnostics["run_state"] = status
                    if snapshot.phase != previous_phase:
                        feedback = FormationFeedback()
                        feedback.phase = (FormationFeedback.HOLDING
                                          if snapshot.phase == "HOLDING"
                                          else FormationFeedback.MOVING)
                        work.goal_handle.publish_feedback(feedback)
                        previous_phase = snapshot.phase
                    for name in ("max_position_error", "max_velocity"):
                        value = getattr(snapshot, name)
                        if value is not None:
                            diagnostics[name] = max(diagnostics[name] or 0.0, value)
                    distance = snapshot.min_inter_agent_distance
                    if distance is not None:
                        old = diagnostics["min_inter_agent_distance"]
                        diagnostics["min_inter_agent_distance"] = (
                            distance if old is None else min(old, distance))
                    diagnostics["position_not_settled"] |= (
                        snapshot.max_position_error is not None
                        and snapshot.max_position_error > self.epsilon_p)
                    diagnostics["velocity_not_settled"] |= (
                        snapshot.max_velocity is not None
                        and snapshot.max_velocity > self.epsilon_v)
                    if now_s - last_obstacle_sample >= self.obstacle_sample_period:
                        clearance = self._obstacle_clearance(samples)
                        if clearance is not None:
                            obstacle_clearances.append(clearance)
                            diagnostics["min_obstacle_clearance"] = min(
                                diagnostics.get("min_obstacle_clearance", clearance),
                                clearance)
                        last_obstacle_sample = now_s
                    # Whole-run latches: the plane envelope and the declared box
                    # are checked over the entire task, not only at the end.
                    box_clearance = self._box_clearance(samples)
                    if box_clearance is not None:
                        previous = diagnostics.get("min_box_surface_clearance_m")
                        diagnostics["min_box_surface_clearance_m"] = (
                            box_clearance if previous is None
                            else min(previous, box_clearance))
                    if snapshot.min_height_m is not None:
                        previous = diagnostics.get("min_member_height_m")
                        diagnostics["min_member_height_m"] = (
                            snapshot.min_height_m if previous is None
                            else min(previous, snapshot.min_height_m))
                    diagnostics["stale_local_scans"] = self._stale_local_scans(now_s)
                    violation = self._violation_reason(diagnostics, snapshot)
                    if violation:
                        diagnostics["violation_latched_at_s"] = now_s
                        diagnostics["violation_reason"] = violation
                        for agent_id in self.agent_ids:
                            self._feed_adoption(agent_id, now_s)
                        break
                    self._record_member_samples(
                        member_samples, samples, references, monitor, now_s)
                    reference_missing += self._count_missing_references(references)
                    for agent_id in self.agent_ids:
                        self._feed_adoption(agent_id, now_s)
                    self._note_alignment(alignment)
                    row = {key: value for key, value in asdict(snapshot).items()
                           if key in fields}
                    row["model_hold_elapsed"] = snapshot.model_hold_elapsed_s
                    row.update(ros_time=now_s, elapsed=(now - start).to_sec(),
                               stale_agent_ids=";".join(map(str, snapshot.stale_agent_ids)))
                    writer.writerow(row)
                    stream.flush()
                    if snapshot.terminal_state:
                        break
                    rate.sleep()
        finally:
            with self.lock:
                self.active_diagnostics = None

        finish = rospy.Time.now()
        self._finalize(diagnostics, work, monitor, adoption, alignment,
                       member_samples, obstacle_clearances, reference_missing,
                       start, finish, counts_start, group_goal_start)

    # ------------------------------------------------------------- sampling
    @staticmethod
    def _latest_sample(window):
        """Newest state of one member out of a monitor sample window.

        ``GroupCompletionMonitor`` consumes every state received since the
        previous tick so a transient excursion between ticks cannot be hidden.
        The per-tick sample ledger and the sampled obstacle check need exactly
        one state per member, so they take the newest one.
        """
        if window is None:
            return None
        if isinstance(window, OdometrySample):
            return window
        states = list(window)
        return states[-1] if states else None

    def _record_member_samples(self, member_samples, samples, references, monitor,
                               now_s):
        with self.lock:
            model_times = {
                agent_id: self.model_times.model_time_at(
                    "drone_{}".format(agent_id), now_s, self.model_time_window)
                for agent_id in self.agent_ids}
        for agent_id in self.agent_ids:
            sample = self._latest_sample(samples.get(agent_id))
            if sample is None:
                continue
            reference = references.get(agent_id)
            used_position = used_velocity = None
            if (reference is not None and "position" in reference
                    and abs(reference.get("ros_time_s", -1e9) - now_s) <= 0.1):
                used_position = reference["position"]
                used_velocity = reference.get("velocity")
            with self.lock:
                source = self.qn_source.get(agent_id) or {}
            model_step = source.get("used_outer_step")
            member_samples[agent_id].append(MemberSample(
                agent_id=agent_id, ros_time_s=now_s, position=sample.position,
                world_velocity=sample.velocity,
                used_reference_position=used_position,
                used_reference_velocity=used_velocity,
                target_position=monitor.targets[agent_id],
                model_time_s=model_times.get(agent_id),
                state_stamp_s=sample.stamp,
                model_step=None if model_step is None else int(model_step)))

    def _count_missing_references(self, references):
        missing = 0
        for agent_id in self.agent_ids:
            entry = references.get(agent_id)
            if entry is None or "position" not in entry or "velocity" not in entry:
                missing += 1
        return missing

    def _box_clearance(self, samples):
        """Actual surface clearance to the declared obstacle box.

        ``None`` means no box was declared: the check is not applicable, which is
        a different statement from a zero clearance or a missing measurement.
        """
        if not self.obstacle_present:
            return None
        worst = None
        for agent_id in self.agent_ids:
            sample = self._latest_sample(samples.get(agent_id))
            if sample is None:
                continue
            value = box_surface_clearance(
                sample.position, self.obstacle_center, self.obstacle_size,
                self.platform_radius_m)
            worst = value if worst is None else min(worst, value)
        return worst

    def _stale_local_scans(self, now_s):
        """Local scan topics that stopped delivering while a task is running.

        A stop in the scan chain is a perception failure, not a quiet
        environment: the scan had been arriving and then stopped, which the
        readiness health reports as stale after ``cloud_timeout``.
        """
        stale = []
        with self.lock:
            topics = dict(self.readiness.topics)
        for topic, health in topics.items():
            if not self._is_local_scan_topic(topic):
                continue
            if health.last_stamp_s is None:
                continue
            if now_s - health.last_stamp_s > self.cloud_timeout:
                stale.append((topic, now_s - health.last_stamp_s))
        return stale

    @staticmethod
    def _is_local_scan_topic(topic):
        return topic.endswith("pcl_render_node/cloud") or \
            topic.endswith("pcl_render_node/depth")

    def _violation_reason(self, diagnostics, snapshot):
        """The first observed violation, or None.

        Violations are latched as soon as they are seen and end the task; waiting
        for the final summary would let a task keep flying after a failure it has
        already detected.
        """
        stale = diagnostics.get("stale_local_scans")
        if stale:
            return "local scan stopped delivering: {}".format(", ".join(
                "{} ({:.1f} s)".format(topic, age) for topic, age in stale))
        box = diagnostics.get("min_box_surface_clearance_m")
        if box is not None and box < self.obstacle_clearance:
            return "box surface clearance {:.3f} m below {:.2f} m".format(
                box, self.obstacle_clearance)
        height = diagnostics.get("min_member_height_m")
        if height is not None and height - self.platform_radius_m < self.surface_plane_m:
            return ("member envelope reached {:.3f} m, below the declared "
                    "surface {:.3f} m".format(
                        height - self.platform_radius_m, self.surface_plane_m))
        if snapshot is not None and snapshot.min_inter_agent_surface_distance is not None \
                and snapshot.min_inter_agent_surface_distance < self.inter_agent_clearance:
            return "inter-agent surface clearance {:.3f} m below {:.2f} m".format(
                snapshot.min_inter_agent_surface_distance, self.inter_agent_clearance)
        if self.air_domain_violation:
            return "the run left the AIR model"
        return None

    def _obstacle_clearance(self, samples):
        with self.lock:
            cloud = self.map_cloud
        if cloud is None or numpy is None or len(cloud) == 0:
            return None
        points = []
        for agent_id in self.agent_ids:
            sample = self._latest_sample(samples.get(agent_id))
            if sample is not None:
                points.append(sample.position)
        if not points:
            return None
        positions = numpy.asarray(points, dtype=float)
        differences = cloud[None, :, :] - positions[:, None, :]
        return float(numpy.sqrt((differences ** 2).sum(axis=2)).min())

    def _note_alignment(self, alignment):
        """Feed every new model-time sample, not only the latest per tick.

        The monitor loop runs at ~20 Hz while diagnostics arrive at ~100 Hz;
        feeding only the newest sample per tick leaves 50 ms holes in the
        common grid and shows up as spurious "missing agents" grid points in
        the task-scope time gate.
        """
        with self.lock:
            entries = []
            for agent_id in self.agent_ids:
                agent = "drone_{}".format(agent_id)
                since = self._alignment_fed_ros.get(agent, 0.0)
                samples = self.model_times.samples_after(agent, since)
                reference = self.used_reference.get(agent_id) or {}
                entries.append((agent, samples, reference.get("velocity")))
        for agent, samples, speed in entries:
            for sample in samples:
                alignment.note_sample(sample)
            if samples:
                self._alignment_fed_ros[agent] = samples[-1].ros_time_s
            if speed is not None:
                magnitude = math.sqrt(sum(value * value for value in speed))
                alignment.note_reference_speed(magnitude)
                if alignment is not self.session_alignment:
                    # The whole-run summary needs the measured reference speed
                    # too, otherwise its induced-displacement gate is unset.
                    self.session_alignment.note_reference_speed(magnitude)

    def _model_hold_elapsed(self, hold_start_ros, now_ros, diagnostics):
        """min_i(tau_i(hold_end) - tau_i(hold_start)) enforced in model time."""
        elapsed = []
        with self.lock:
            for agent_id in self.agent_ids:
                start = self.model_times.model_time_at(
                    "drone_{}".format(agent_id), hold_start_ros,
                    self.model_time_window)
                end = self.model_times.model_time_at(
                    "drone_{}".format(agent_id), now_ros, self.model_time_window)
                if start is None or end is None:
                    diagnostics["model_hold"] = {
                        "min_model_hold_s": None,
                        "missing_agents": [agent_id],
                        "required_s": diagnostics["hold_duration"],
                    }
                    return None
                elapsed.append(end - start)
        minimum = min(elapsed)
        diagnostics["model_hold"] = {
            "min_model_hold_s": minimum,
            "per_agent_s": {str(agent_id): value
                            for agent_id, value in zip(self.agent_ids, elapsed)},
            "required_s": diagnostics["hold_duration"],
        }
        return minimum

    # ------------------------------------------------------------ finalize
    def _finalize(self, diagnostics, work, monitor, adoption, alignment,
                  member_samples, obstacle_clearances, reference_missing,
                  start, finish, counts_start, group_goal_start):
        snapshot = monitor.snapshot
        diagnostics["actual_finish_time"] = finish.to_sec()
        diagnostics["final_phase"] = snapshot.phase if snapshot else "MOVING"
        diagnostics["failed_agent_ids"] = (
            list(snapshot.stale_agent_ids) if snapshot else [])
        diagnostics["planner_finish_to_actual_seconds"] = {
            key: finish.to_sec() - nominal
            for key, nominal in diagnostics["planner_nominal_finish_times"].items()}
        with self.lock:
            counts = {str(agent_id): self.odom_counts[agent_id] - counts_start[agent_id]
                      for agent_id in self.agent_ids}
            diagnostics["topic_group_goal_count"] = (
                self.group_goal_messages - group_goal_start)
        elapsed = finish.to_sec() - diagnostics["actual_start_time"]
        diagnostics["odom_sample_counts"] = counts
        diagnostics["odom_sample_rates"] = {
            key: value / elapsed if elapsed > 0.0 else 0.0
            for key, value in counts.items()}
        diagnostics["reference_missing_samples"] = reference_missing
        diagnostics["odometry_contract_errors"] = dict(self.odometry_errors)
        diagnostics["max_diagnostics_callback_lag_s"] = (
            self.max_diagnostics_callback_lag_s)

        motion_completed = (snapshot is not None
                            and snapshot.terminal_state == "SUCCEEDED")
        if self.state_machine.state == "UNKNOWN_LOCKED":
            motion_completed = False
        adoption_verdict = adoption.verdict()
        diagnostics["adoption_verdict"] = adoption_verdict.as_dict()
        diagnostics["state_machine"] = self.state_machine.snapshot()

        alignment_report = alignment.report(require_baseline=False, min_duration_s=1.0)
        diagnostics["time_alignment"] = alignment_report.as_dict()
        baseline_report = self._cached_baseline_report()
        diagnostics["time_alignment_baseline"] = baseline_report.as_dict()
        time_ok = (baseline_report.within_thresholds
                   and alignment_report.within_thresholds
                   and not alignment_report.alignment_failure_count)
        if (alignment_report.induced_reference_displacement_m is not None
                and alignment_report.induced_reference_displacement_m
                > self.time_reference_displacement_limit):
            time_ok = False
            diagnostics.setdefault("time_alignment_reasons", []).append(
                "time-drift reference displacement {:.4f} m exceeds {:.4f} m".format(
                    alignment_report.induced_reference_displacement_m,
                    self.time_reference_displacement_limit))

        # The expected grid is generated from the task window and the nominal
        # monitor period.  It must not be derived from the samples that were
        # received, otherwise a missing interval would leave no trace.
        period = 1.0 / self.monitor_rate
        window_start = start.to_sec()
        window_end = max(finish.to_sec(), window_start)
        count = int(math.floor((window_end - window_start) / period)) + 1
        grid = [window_start + index * period for index in range(max(count, 1))]
        ledger = build_ledger(self.agent_ids, grid, member_samples,
                              expected_period_s=period)
        hold_started = snapshot.hold_started if snapshot else None
        hold_finished = finish.to_sec()
        metrics = compute_metrics(
            member_samples, scale=self.scale, relative_slots=self.slots,
            hold_start_s=hold_started, hold_end_s=hold_finished)
        air = self._air_domain_summary()
        diagnostics["air_domain"] = air
        # The plane envelope is judged over the whole task, not on the final
        # monitor window.
        lowest_height = diagnostics.get(
            "min_member_height_m",
            snapshot.min_height_m if snapshot is not None else None)
        diagnostics["min_member_height_m"] = lowest_height
        surface_violation = (
            lowest_height is not None
            and lowest_height - self.platform_radius_m < self.surface_plane_m)
        box_clearance = diagnostics.get("min_box_surface_clearance_m")
        if box_clearance is None:
            box_clearance = self._box_clearance({
                agent_id: [member.position for member in series]
                for agent_id, series in member_samples.items()})
            diagnostics["min_box_surface_clearance_m"] = box_clearance
        safety = evaluate_safety(
            member_samples, obstacle_clearances=obstacle_clearances or None,
            required_inter_agent_clearance_m=self.inter_agent_clearance,
            required_obstacle_clearance_m=self.obstacle_clearance,
            platform_radius_m=self.platform_radius_m,
            surface_clearance_violation=surface_violation,
            surface_detail=(
                "member envelope reached {:.3f} m, below the declared surface "
                "{:.3f} m".format(lowest_height - self.platform_radius_m,
                                  self.surface_plane_m)
                if surface_violation else ""),
            box_clearance_m=box_clearance,
            box_center=self.obstacle_center if self.obstacle_present else None,
            box_size=self.obstacle_size if self.obstacle_present else None,
            required_box_clearance_m=self.obstacle_clearance,
            obstacle_check_expected=self.obstacle_present)
        model_hold = diagnostics.get("model_hold") or {}
        min_model_hold = model_hold.get("min_model_hold_s")
        model_hold_satisfied = (
            motion_completed and min_model_hold is not None
            and min_model_hold + 1e-9 >= diagnostics["hold_duration"])
        verdict = decide(
            epsilon_p=self.epsilon_p, epsilon_v=self.epsilon_v,
            motion_completed=motion_completed,
            adoption_state=adoption_verdict.state,
            model_hold_satisfied=model_hold_satisfied,
            time_alignment_ok=time_ok, metrics=metrics, ledger=ledger, safety=safety,
            air_domain_ok=air["ok"], air_domain_detail=air["detail"],
            hold_duration_s=diagnostics["hold_duration"],
            min_valid_sample_ratio=self.min_valid_sample_ratio)
        diagnostics["verdict"] = verdict.as_dict()
        diagnostics["task_outcome"] = verdict.task_outcome
        diagnostics["safety_outcome"] = verdict.safety_outcome
        diagnostics["experiment_validity"] = verdict.experiment_validity
        diagnostics["safety_evidence_kind"] = safety.evidence_kind
        if motion_completed:
            diagnostics["successful_hold_window"] = {
                "start": hold_started,
                "end": hold_finished,
                "duration": (None if hold_started is None
                             else hold_finished - hold_started),
                "required_duration": diagnostics["hold_duration"],
                "min_model_hold_s": min_model_hold,
            }

        reason = FormationResult.NONE
        text = "task_outcome={} safety_outcome={} experiment_validity={}".format(
            verdict.task_outcome, verdict.safety_outcome, verdict.experiment_validity)
        if not motion_completed:
            if self.state_machine.unknown_locked_reason:
                reason = FormationResult.UNKNOWN_LOCKED
            elif adoption_verdict.state != "ADOPTED":
                # Waiting for adoption is normal; hitting the execution timeout
                # while still waiting means this task's reference never arrived.
                reason = FormationResult.REFERENCE_ADOPTION_UNCONFIRMED
            elif snapshot is not None and snapshot.reason == monitor.ODOMETRY_TIMEOUT:
                reason = FormationResult.ODOMETRY_TIMEOUT
            else:
                reason = FormationResult.EXECUTION_TIMEOUT
        elif adoption_verdict.state != "ADOPTED":
            reason = FormationResult.REFERENCE_ADOPTION_UNCONFIRMED
        elif (not air["ok"] or verdict.safety_outcome != SAFETY_PASS
              or diagnostics.get("violation_reason")):
            # The task may have reached its slots, but the run violated the
            # operating or safety envelope.  Lock instead of continuing.
            reason = FormationResult.UNKNOWN_LOCKED
        elif not time_ok or verdict.task_outcome != TASK_PASS:
            reason = FormationResult.MODEL_TIME_MISMATCH
        diagnostics["reason"] = reason
        diagnostics["reason_text"] = text

        # The resource is released only for a task the task layer accepts.
        # Reaching the slots is an observation, not a licence to dispatch the
        # next task: an unproven reference, a failed time gate, a safety failure
        # or an AIR-domain violation locks the chain instead.
        accepted = (
            motion_completed
            and adoption_verdict.state == "ADOPTED"
            and time_ok
            and air["ok"]
            and verdict.task_outcome == TASK_PASS
            and verdict.safety_outcome == SAFETY_PASS
            and verdict.experiment_validity == VALIDITY_VALID)
        self.state_machine.ros_time_s = finish.to_sec()
        if accepted:
            if self.state_machine.state == "HOLDING":
                self.state_machine.finish_success()
            if self.state_machine.state == "SUCCEEDED":
                self.state_machine.release()
        else:
            self.state_machine.fault(text)
        diagnostics["accepted_for_dispatch"] = accepted
        # Evidence is written *after* the transition so it records the state the
        # resource is actually in when the Action returns to the caller.
        diagnostics["run_state"] = self.state_machine.state
        diagnostics["state_machine"] = self.state_machine.snapshot()
        diagnostics["resource_released"] = (
            self.state_machine.state == "READY_IDLE")
        self._write_diagnostics(diagnostics)
        result = self._result_message(diagnostics, reason)
        if reason == FormationResult.NONE:
            work.goal_handle.set_succeeded(result, text)
        else:
            work.goal_handle.set_aborted(result, text)
        rospy.loginfo("FormationAction %s: %s", diagnostics["task_id"], text)


def main():
    rospy.init_node("formation_action_server")
    FormationActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
