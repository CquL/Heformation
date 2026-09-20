#!/usr/bin/env python3
"""One native Swarm goal, with completion established by actual qn state.

Design notes (plan.md P0/P1):

* the process starts immediately, but only ``READY_IDLE`` accepts a Goal; the
  goal callback validates, reserves the group resource atomically and hands the
  work to a single execution loop, then returns,
* a running task is never preempted by a new goal; opt-in cancellation requests
  local holding and observes qn before returning a non-success terminal Result,
  while retaining resources (the seven-member default only records cancel),
* odometry loss, execution timeout and result timeout enter ``UNKNOWN_LOCKED``,
* task outcome, safety outcome and experiment validity are reported separately,
* the new task reference must be proven by ``trajectory_id`` ownership.
"""

import csv
from collections import deque
import json
import math
import queue
import subprocess
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
    build_ledger, compute_metrics, decide, evaluate_safety, StaticSceneGeometry,
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


DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"


def goal_topic(template, agent_id):
    """Format a goal topic template.  A template without ``{}`` is a broadcast."""
    return template.format(agent_id) if "{" in template else template


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
        self.safety_agent_ids = list(rospy.get_param("~safety_agent_ids", self.agent_ids))
        if not set(self.agent_ids) <= set(self.safety_agent_ids):
            raise ValueError("safety_agent_ids must include all controlled members")
        self.peer_odom = {}
        self.scale = float(rospy.get_param("~swarm_scale", 2.0))
        self.epsilon_p = float(rospy.get_param("~epsilon_p", 0.5))
        self.epsilon_v = float(rospy.get_param("~epsilon_v", 0.25))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.25))
        self.execution_timeout = float(rospy.get_param("~execution_timeout", 180.0))
        self.safety_hold_enabled = bool(rospy.get_param("~safety_hold_enabled", False))
        self.safety_hold_timeout = float(rospy.get_param("~safety_hold_timeout_s", 180.0))
        if not math.isfinite(self.safety_hold_timeout) or self.safety_hold_timeout <= 0:
            raise ValueError("safety_hold_timeout_s must be finite and positive")
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
        self.static_scene=StaticSceneGeometry.from_mapping(scene)
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
        self.terminal_lock = threading.RLock()
        self.cancelled_goals = set()
        self.terminal_goals = set()
        self.safety_status = {}
        self.latched_members = set()
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
        # Goal routing.  The default reproduces the upstream broadcast exactly:
        # one topic that every planner subscribes to.  A template containing {}
        # is per member, which is what lets a command address a subset.
        self.formation_goal_topic_format = str(
            rospy.get_param("~goal_topic_format", DEFAULT_GOAL_TOPIC))
        self.member_goal_topic_format = str(
            rospy.get_param("~member_goal_topic_format", ""))
        self.formation_goal_topics = {
            agent_id: goal_topic(self.formation_goal_topic_format, agent_id)
            for agent_id in self.agent_ids}
        self.member_goal_topics = (
            {agent_id: goal_topic(self.member_goal_topic_format, agent_id)
             for agent_id in self.agent_ids}
            if self.member_goal_topic_format else {})
        self.goal_publishers = {
            topic: rospy.Publisher(topic, PoseStamped, queue_size=1)
            for topic in sorted(set(self.formation_goal_topics.values()))}
        self.member_goal_publishers = {
            topic: rospy.Publisher(topic, PoseStamped, queue_size=1)
            for topic in sorted(set(self.member_goal_topics.values()))}
        self.goal_topics = sorted(set(self.formation_goal_topics.values())
                                  | set(self.member_goal_topics.values()))
        self.observed_goal_topics = set()
        self.subscribers = [
            rospy.Subscriber(self.map_topic, PointCloud2, self._map_callback,
                             queue_size=1),
        ] + [
            rospy.Subscriber(topic, PoseStamped, self._group_goal_observer,
                             callback_args=topic, queue_size=10)
            for topic in self.goal_topics]
        sensing_type = (PointCloud2 if self.sensor_backend == SENSOR_BACKEND_CPU
                        else Image)
        for agent_id in self.agent_ids:
            if self.safety_hold_enabled:
                self.subscribers.append(rospy.Subscriber(
                    "/drone_{}_planning/safety_status".format(agent_id), DiagnosticArray,
                    self._safety_status_callback, callback_args=agent_id, queue_size=1))
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
        for agent_id in set(self.safety_agent_ids) - set(self.agent_ids):
            self.subscribers.append(rospy.Subscriber(
                odometry_topic(agent_id), Odometry, self._peer_odom_callback,
                callback_args=agent_id, queue_size=1))
        # One node per execution unit, so the Action name has to be a parameter:
        # four units sharing "formation_action" would collide.
        self.action_name = str(rospy.get_param("~action_name", "formation_action"))
        self.action_server = actionlib.ActionServer(
            self.action_name, FormationAction,
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

    def _peer_odom_callback(self, message, agent_id):
        try:
            sample = parse_standard_odometry(message, "drone_{}".format(agent_id))
        except OdometryContractError:
            with self.lock:
                self.peer_odom.pop(agent_id, None)
            return
        with self.lock:
            self.peer_odom[agent_id] = sample

    def _fleet_safety(self, now_s):
        """Discrete fleet clearance, including idle members of other units.

        Reuse odometry freshness and model alignment window. Missing peers cannot
        make a one-member Action report collision safety for the whole fleet.
        """
        with self.lock:
            samples = dict(self.peer_odom)
            samples.update(self.odom)
        fresh = {a: samples[a] for a in self.safety_agent_ids
                 if a in samples and samples[a].is_fresh(now_s, self.odom_timeout)}
        if len(fresh) != len(self.safety_agent_ids):
            return {"ok": False, "reason": "missing or stale fleet odometry",
                    "missing": sorted(set(self.safety_agent_ids) - set(fresh))}
        stamps = [v.stamp for v in fresh.values()]
        if max(stamps) - min(stamps) > self.model_time_window:
            return {"ok": False, "reason": "fleet odometry not time aligned"}
        positions = [fresh[a].position for a in sorted(fresh)]
        clearances = [math.dist(p, q) - 2*self.platform_radius_m
                      for i, p in enumerate(positions) for q in positions[i+1:]]
        minimum = min(clearances, default=None)
        ok = minimum is None or minimum >= self.inter_agent_clearance
        return {"ok": ok, "reason": "" if ok else "fleet inter-agent clearance violated",
                "min_surface_clearance_m": minimum, "agent_ids": self.safety_agent_ids,
                "evidence_kind": "DISCRETE_SAMPLED"}

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
                "reference_source": values.get("reference_source", "AIR_SWARM"),
                "requested_reference_source": values.get("requested_reference_source", "AIR_SWARM"),
                "reference_handover_enabled": values.get("reference_handover_enabled", "false"),
                "reference_generation": values.get("reference_generation", "0"),
                "reference_air_floor_id": values.get("reference_air_floor_id", "-1"),
                "reference_context_ready": values.get("reference_context_ready", "false"),
                "reference_active": values.get("reference_active", "false"),
                "reference_goal_id": values.get("reference_goal_id", ""),
                "platform_action_active": values.get("platform_action_active", "false"),
                "platform_resource_locked": values.get("platform_resource_locked", "false").lower(),
                "actual_mode": values.get("actual_mode", "AIR"),
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

        scoped=values.get("domain_policy_version")=="1"
        flag = number("air_scope_max_medium_flag" if scoped else "max_medium_flag")
        height = number("air_scope_min_height_m" if scoped else "min_height_m")
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
        if (source is not None and source["source_trajectory_id"] >= 0
                and source.get("reference_source", "AIR_SWARM")=="AIR_SWARM"):
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

    def _publish_routed_goal(self, command):
        semantics, route = self.goal_route()
        publishers = self.member_goal_publishers if semantics == "MEMBER_TARGET" else self.goal_publishers
        topics = set()
        for _agent, topic in route:
            if topic not in topics:
                publishers[topic].publish(command)
                topics.add(topic)
        return semantics, route, topics

    def goal_route(self):
        """Where the next dispatch goes and what the position means.

        Returns ``(semantics, [(agent_id, topic), ...])``.  The planner applies
        the matching interpretation, so these two must stay in step:

          MEMBER_TARGET     the position is that member's own world target and
                            must not have a formation slot offset added;
          FORMATION_CENTRE  the position is a formation centre and every member
                            adds its own slot offset.
        """
        member_topics = getattr(self, "member_goal_topics", {})
        if len(self.agent_ids) == 1 and member_topics:
            agent_id = self.agent_ids[0]
            return "MEMBER_TARGET", [(agent_id, member_topics[agent_id])]
        return "FORMATION_CENTRE", [
            (agent_id, self.formation_goal_topics[agent_id])
            for agent_id in self.agent_ids]

    def _member_goal_topics_for(self, agent_id):
        """Topics this member's planner must be subscribed to for this unit."""
        topics = [self.formation_goal_topics[agent_id]]
        if self.member_goal_topics:
            topics.append(self.member_goal_topics[agent_id])
        return topics

    def _group_goal_observer(self, message, topic):
        header = getattr(message, "_connection_header", None) or {}
        publisher = header.get("callerid") if isinstance(header, dict) else None
        with self.lock:
            self.group_goal_messages += 1
            self.observed_goal_topics.add(topic)
            diagnostics = self.active_diagnostics
            tracker = diagnostics.get("adoption") if diagnostics else None
        if tracker is not None:
            # Counted from the observed topic, not from our own publish call:
            # a foreign publisher emitting a group goal must not go unnoticed.
            tracker.note_group_goal(publisher, topic)

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
    def uses_member_entry(self):
        """Whether this unit commands members directly rather than a centre."""
        return len(self.agent_ids) == 1 and bool(self.member_goal_topics)

    def _planner_configuration_matches(self):
        for agent_id in self.agent_ids:
            prefix = "/drone_{}_ego_planner_node/".format(agent_id)
            configuration = rospy.get_param(prefix.rstrip("/"), {})
            if configuration.get("fsm", {}).get("flight_type") != 3:
                return "planner {} is not in formation goal mode".format(agent_id)
            if configuration.get("manager", {}).get("drone_id") != agent_id:
                return "planner {} has a different drone ID".format(agent_id)
            global_goal = configuration.get("global_goal", {})
            if self.uses_member_entry():
                # A single-member unit commands the member's own world target
                # through the member entry, which applies no slot offset at all.
                # The planner's formation slots are then irrelevant to this unit,
                # so requiring them to match would reject a correct configuration:
                # the unit's slot for its only member is necessarily the origin,
                # while the planner keeps the slot it uses for group commands.
                continue
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
        # Every member's planner must be listening on the topics this unit will
        # publish to, so a dispatch cannot silently reach nobody.  The check is
        # per member, not "every planner on every topic": with per-member goal
        # topics each planner subscribes only to its own, while the upstream
        # broadcast has all of them on one topic - both must pass.
        for agent_id in self.agent_ids:
            node = "/drone_{}_ego_planner_node".format(agent_id)
            for topic in self._member_goal_topics_for(agent_id):
                if node not in set(subscribers.get(topic, [])):
                    return ("waiting for {} to subscribe to {}".format(node, topic))
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
        wanted = {topic for agent_id in self.agent_ids
                  for topic in self._member_goal_topics_for(agent_id)}
        connected = {connection[1] for connection in connections
                     if len(connection) >= 6 and connection[2] == "o"
                     and connection[4] in wanted and connection[5]}
        if code != 1 or not expected <= connected:
            return ("waiting for established TCPROS goal connections to all "
                    "members on {}".format(sorted(wanted)))
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
        reference_reason=self._reference_dispatch_block()
        if reference_reason:reasons.append(reference_reason)
        if not status.ready:
            reasons.append(status.reason)
        if getattr(self, "safety_hold_enabled", False):
            safety_reason = self._safety_dispatch_block()
            if safety_reason:
                reasons.append(safety_reason)
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
        startup_complete = False
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
                if getattr(self, "safety_disposition_payload", None) is not None:
                    rospy.set_param("~safety_disposition", self.safety_disposition_payload)
            except Exception as error:
                rospy.logwarn_throttle(
                    5.0, "could not publish readiness parameters: %s", error)
            if not snapshot["ready"]:
                rospy.loginfo_throttle(5.0, "FormationAction not ready: %s",
                                       snapshot["reason"])
                if not startup_complete and time.monotonic() >= deadline:
                    rospy.logerr("FormationAction startup timed out: %s",
                                 snapshot["reason"])
                    rospy.signal_shutdown("formation startup timeout: " +
                                          snapshot["reason"])
                    return
            else:
                startup_complete = True
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
        # The same boundary orders cancellation, local latch and terminal commit.
        with getattr(self, "terminal_lock", self.lock):
            safety_block = None
            reference_block=self._reference_dispatch_block()
            if reference_block:
                self.state_machine.note_readiness(False)
            if getattr(self, "safety_hold_enabled", False):
                safety_block = self._safety_dispatch_block()
                if safety_block:
                    self.state_machine.note_readiness(False)
            status, reason, _ = self.state_machine.request_goal(
                goal_handle.get_goal_id().id, validation_error=validation_error)
            if safety_block and not validation_error and status != ACCEPTED:
                reason = safety_block
            if reference_block and not validation_error and status != ACCEPTED:
                reason = reference_block
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
        """Scope cancellation to its Goal; the worker observes opt-in holding."""
        if getattr(self, "safety_hold_enabled", False):
            goal_id = goal_handle.get_goal_id().id
            with self.terminal_lock:
                if goal_id in self.terminal_goals or goal_id != self.state_machine.goal_id:
                    return
                self.cancelled_goals.add(goal_id)
                self.state_machine.note_cancel_request(goal_id, stop_and_lock=True)
            return
        self.state_machine.ros_time_s = rospy.Time.now().to_sec()
        running = goal_handle.get_goal_status().status in _ACTIVE_STATUS
        self.state_machine.note_cancel_request()
        rospy.logwarn(
            "FormationAction cancel request observed for %s; running=%s "
            "(actionlib may show PREEMPTING, the physical task is not cancelled)",
            goal_handle.get_goal_id().id, running)

    def _safety_status_callback(self, message, agent_id):
        entries = [s for s in message.status if s.name == "local_safety_hold"
                   and s.hardware_id == "drone_{}".format(agent_id)]
        if len(entries) != 1:
            return
        values = {v.key: v.value for v in entries[0].values}
        try:
            if values["latched"] not in ("true", "false"):
                return
            row = {"latched": values["latched"] == "true", "reason": values["reason"],
                   "trajectory_id": int(values["trajectory_id"]),
                   "reference_published": values["reference_published"] == "true",
                   "point_valid": values["point_valid"] == "true",
                   "hold_point": tuple(float(values["hold_"+a]) for a in "xyz"),
                   "latched_at_s": float(values["latched_at_s"]),
                   "last_cloud_stamp_s": float(values["last_cloud_stamp_s"]),
                   "stamp_s": message.header.stamp.to_sec(), "received_monotonic": time.monotonic()}
            if not all(math.isfinite(v) for v in row["hold_point"] + (
                    row["latched_at_s"], row["last_cloud_stamp_s"], row["stamp_s"])):
                return
        except (KeyError, TypeError, ValueError):
            return
        with self.terminal_lock:
            with self.lock:
                newly_latched = row["latched"] and agent_id not in self.latched_members
                self.safety_status[agent_id] = row
                if row["latched"]:
                    self.latched_members.add(agent_id)
                self._readiness_cache = None
            if newly_latched:
                self.state_machine.fault("physical member drone_{} safety hold latched".format(agent_id))
                rospy.set_param("~ready", False)
                rospy.set_param("~safety_latched_members", sorted(self.latched_members))

    def _safety_dispatch_block(self):
        if not getattr(self, "safety_hold_enabled", False):
            return None
        now, wall = rospy.Time.now().to_sec(), time.monotonic()
        with self.lock:
            for a in self.agent_ids:
                if a in self.latched_members:
                    return "physical member drone_{} safety hold latched".format(a)
                status = self.safety_status.get(a)
                if (status is None or wall-status["received_monotonic"] > self.qn_state_timeout
                        or not 0 <= now-status["stamp_s"] <= self.qn_state_timeout):
                    return "drone_{} safety status unavailable/stale".format(a)
        return None

    def _reference_dispatch_block(self):
        with self.lock:
            for member in self.agent_ids:
                row=self.qn_source.get(member,{})
                if row.get('reference_handover_enabled')!='true':continue
                if row.get('platform_resource_locked')=='true':
                    return 'member {} local reference fault locked'.format(member)
                if row.get('platform_action_active')=='true' or row.get('actual_mode')!='AIR':
                    return 'member {} is executing a native segment or outside AIR'.format(member)
                if row.get('reference_active')=='true' and row.get('reference_goal_id')!=self.state_machine.goal_id:
                    return 'member {} reference belongs to another accepted Goal'.format(member)
                if row.get('reference_context_ready')!='true':
                    return 'member {} planner ownership confirmation unavailable'.format(member)
        return None

    def _claim_air_references(self,goal_id):
        """Bounded local handover; ordinary goals are published only after ACK.

        rosservice serializes the standard typed service response as YAML. This
        is the existing bounded CLI-RPC approach used for safety requests, not
        parsing control state from planner log messages.
        """
        import yaml
        evidence=[]
        deadline=time.monotonic()+10.
        with self.lock:
            enabled=[(m,dict(self.qn_source[m])) for m in self.agent_ids
                     if self.qn_source.get(m,{}).get('reference_handover_enabled')=='true']
        for member,row in enabled:
            service='/drone_{}_qn_aav/take_reference'.format(member)
            payload=yaml.safe_dump(dict(goal_id=goal_id,source='AIR_SWARM',
                                       expected_generation=int(row['reference_generation'])))
            process=subprocess.Popen(['rosservice','call',service,payload],
                stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            try:
                output,error=process.communicate(timeout=max(.001,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=1.)
                raise RuntimeError('reference handover RPC timed out: '+service)
            if process.returncode:
                raise RuntimeError('reference handover RPC failed: '+error.decode(errors='replace'))
            response=yaml.safe_load(output.decode())
            if not response or response.get('accepted') is not True:
                raise RuntimeError('reference handover refused: '+str(response))
            generation=str(response['generation'])
            while time.monotonic()<deadline and not rospy.is_shutdown():
                with self.lock:current=dict(self.qn_source.get(member,{}))
                if (current.get('reference_generation')==generation and
                    current.get('requested_reference_source')=='AIR_SWARM' and
                    current.get('reference_context_ready')=='true' and
                    int(current.get('reference_air_floor_id',-1))>=0):
                    evidence.append(dict(member=member,generation=int(generation),
                                         retired_id=int(current['reference_air_floor_id']),
                                         used_outer_step=current['used_outer_step'],
                                         confirmed_at_s=rospy.Time.now().to_sec()))
                    break
                time.sleep(.01)
            else:raise RuntimeError('planner did not acknowledge AIR handover')
        return evidence

    def _safety_trigger(self, goal_id):
        if not getattr(self, "safety_hold_enabled", False):
            return None
        with self.lock:
            if any(s["latched"] and s["reason"] == "LOCAL_CLOUD_STALE"
                   for s in self.safety_status.values()):
                return "GROUP_MEMBER_SENSOR_FAILURE"
        if self._stale_local_scans(rospy.Time.now().to_sec()):
            return "GROUP_MEMBER_SENSOR_FAILURE"
        if goal_id in self.cancelled_goals:
            return "CANCEL_REQUEST"
        return "LOCAL_HOLD_OR_STATUS_FAILURE" if self._safety_dispatch_block() else None

    def _observe_safety_hold(self, work, diagnostics, alignment, member_samples):
        """Bounded observation in this worker; local holding outlives the worker."""
        goal_id = diagnostics["goal_id"]
        existing = diagnostics.get("safety_hold")
        if existing is not None:
            return  # One disposition and one deadline per Goal, including retries.
        started_wall = time.monotonic()
        end_wall = started_wall + self.safety_hold_timeout
        hold = {"reason": self._safety_trigger(goal_id) or "CANCEL_REQUEST",
                "started_ros_s": rospy.Time.now().to_sec(), "timeout_s": self.safety_hold_timeout,
                "deadline_monotonic": end_wall, "verified": False, "members": {},
                "hold_duration": 4.0, "request_results": {}}
        if diagnostics.get("violation_reason"):
            hold["original_violation_reason"] = diagnostics["violation_reason"]
            if hold["reason"] == "CANCEL_REQUEST":
                hold["reason"] = "TASK_FAULT_BEFORE_CANCEL"
        diagnostics["safety_hold"] = hold
        diagnostics["violation_reason"] = hold["reason"]
        self.state_machine.fault(hold["reason"] + "; observing local holding")
        calls = {}
        # Subprocesses provide a cancellable bound for the entire blocking RPC,
        # including services that exist but never return. All requests start first.
        local_faults = {a for a in self.agent_ids
                        if any(topic == local_cloud_topic(a) for topic, _ in
                               self._stale_local_scans(rospy.Time.now().to_sec()))}
        with self.lock:
            local_faults.update(a for a, s in self.safety_status.items()
                                if s["latched"] and s["reason"] == "LOCAL_CLOUD_STALE")
        try:
            for a in self.agent_ids:
                if a in local_faults:
                    hold["request_results"][str(a)] = "LOCAL_WATCHDOG_EXPECTED"
                    continue
                try:
                    calls[a] = subprocess.Popen(
                        ["rosservice", "call", "/drone_{}_planning/safety_hold".format(a)],
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                except OSError as error:
                    hold["request_results"][str(a)] = {"error": str(error)}
            stop_monitor = None
            adopted = {}
            points = {}
            while not rospy.is_shutdown() and time.monotonic() < end_wall:
                now = rospy.Time.now().to_sec()
                trigger = self._safety_trigger(goal_id)
                if trigger == "GROUP_MEMBER_SENSOR_FAILURE":
                    hold["reason"] = trigger  # A sensor fault dominates cancellation.
                for a, process in calls.items():
                    if process.poll() is not None and str(a) not in hold["request_results"]:
                        out, err = process.communicate()
                        hold["request_results"][str(a)] = {
                            "returncode": process.returncode,
                            "response": out.decode(errors="replace"), "error": err.decode(errors="replace")}
                with self.lock:
                    statuses = {a: dict(s) for a, s in self.safety_status.items()}
                    sources = {a: dict(s) for a, s in self.qn_source.items()}
                    commands = dict(self.command_trajectory)
                    references = {a: dict(s) for a, s in self.used_reference.items()}
                    samples = {a: list(self.odom_history[a]) for a in self.agent_ids}
                for a in self.agent_ids:
                    status, source = statuses.get(a), sources.get(a)
                    if not status:
                        continue
                    hold["members"][str(a)] = dict(status)
                    if status["point_valid"] and status["latched"]:
                        points.setdefault(a, status["hold_point"])
                    reliable = (time.monotonic()-status["received_monotonic"] <= self.qn_state_timeout
                                and 0 <= now-status["stamp_s"] <= self.qn_state_timeout)
                    ref = references.get(a) or {}
                    if (reliable and status["latched"] and status["reference_published"]
                            and source and 0 <= now-source["stamp_s"] <= self.qn_state_timeout
                            and source["source_trajectory_id"] == status["trajectory_id"]
                            and commands.get(a, (-1,))[0] == status["trajectory_id"]
                            and source["stamp_s"] >= status["latched_at_s"]
                            and now-ref.get("ros_time_s", 0) <= self.odom_timeout):
                        adopted.setdefault(a, source["stamp_s"])
                    else:
                        adopted.pop(a, None)
                hold["adopted_at_s"] = dict(adopted)
                if len(points) == len(self.agent_ids) and stop_monitor is None:
                    stop_monitor = GroupCompletionMonitor(
                        (0., 0., self.cruise_altitude_m), 4.0, now, agent_ids=self.agent_ids,
                        relative_slots={a: (0., 0., 0.) for a in points}, member_targets=points,
                        swarm_scale=1., epsilon_p=self.epsilon_p, epsilon_v=self.epsilon_v,
                        odom_timeout=self.odom_timeout, execution_timeout=self.safety_hold_timeout,
                        platform_radius_m=self.platform_radius_m, target_z=self.cruise_altitude_m)
                self._note_alignment(alignment)
                fleet = self._fleet_safety(now)
                minimum = fleet.get("min_surface_clearance_m")
                if minimum is not None:
                    old = diagnostics.get("min_inter_agent_distance")
                    distance = minimum + 2*self.platform_radius_m
                    diagnostics["min_inter_agent_distance"] = distance if old is None else min(old, distance)
                if not fleet["ok"]:
                    hold.setdefault("safety_failures", []).append({"time": now, "reason": fleet["reason"]})
                    if minimum is not None and minimum < self.inter_agent_clearance:
                        hold["reason"] = "ACTUAL_SAFETY_VIOLATION"
                if stop_monitor:
                    stop_monitor.start_time = now  # Only the monotonic deadline limits observation.
                    time_ok = alignment.report(require_baseline=False).within_thresholds
                    snapshot = stop_monitor.evaluate(
                        now, samples, reference_confirmed=len(adopted) == len(self.agent_ids) and time_ok,
                        hold_not_before_s=max(adopted.values()) if len(adopted) == len(self.agent_ids) else now,
                        model_hold_satisfied=lambda begin, end: self._model_hold_elapsed(begin, end, hold))
                    self._record_member_samples(member_samples, samples, references, stop_monitor, now)
                    height = snapshot.min_height_m
                    if height is not None:
                        previous = diagnostics.get("min_member_height_m")
                        diagnostics["min_member_height_m"] = height if previous is None else min(previous, height)
                    clearance = self._box_clearance(samples)
                    scene_reason=self._scene_violation(samples)
                    if scene_reason:
                        diagnostics['scene_violation']=scene_reason
                        hold['reason']='ACTUAL_SAFETY_VIOLATION'
                    if clearance is not None:
                        previous = diagnostics.get("min_box_surface_clearance_m")
                        diagnostics["min_box_surface_clearance_m"] = clearance if previous is None else min(previous, clearance)
                    if ((clearance is not None and clearance < self.obstacle_clearance) or
                            (height is not None and height-self.platform_radius_m < self.surface_plane_m)):
                        hold["reason"] = "ACTUAL_SAFETY_VIOLATION"
                    hold["last_snapshot"] = asdict(snapshot)
                    if snapshot.terminal_state == "SUCCEEDED" and time_ok:
                        hold["verified"] = True
                        break
                self.safety_disposition_payload = json.dumps(self._json_safe(hold))
                time.sleep(min(1.0/self.monitor_rate, max(0., end_wall-time.monotonic())))
        finally:
            for a, process in calls.items():
                if process.poll() is None:
                    process.kill()
                    process.communicate()
                    hold["request_results"][str(a)] = "OBSERVATION_DEADLINE_OR_END"
            hold["finished_ros_s"] = rospy.Time.now().to_sec()
            hold["observed_wall_s"] = time.monotonic()-started_wall
            last = hold.get("last_snapshot") or {}
            observed_not_settled = (time.monotonic() >= end_wall
                                   and len(hold.get("adopted_at_s", {})) == len(self.agent_ids)
                                   and last.get("fresh_agent_count") == len(self.agent_ids)
                                   and (last.get("max_position_error", 0) > self.epsilon_p
                                        or last.get("max_velocity", 0) > self.epsilon_v))
            hold["outcome"] = ("VERIFIED" if hold["verified"] else
                               "FAILED" if observed_not_settled else "UNVERIFIED")
            diagnostics["violation_reason"] = hold["reason"]
            self.safety_disposition_payload = json.dumps(self._json_safe(hold))

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
        with self.terminal_lock:
            goal_id = work.goal_handle.get_goal_id().id
            if goal_id in self.terminal_goals:
                return
            diagnostics = self.active_diagnostics or getattr(self, "executing_diagnostics", None)
            if diagnostics is None or diagnostics["goal_id"] != goal_id:
                diagnostics = self._new_diagnostics(work, rospy.Time.now())
            self.state_machine.fault(text)
            diagnostics["reason_text"] = text
            diagnostics["run_state"] = "UNKNOWN_LOCKED"
            diagnostics["state_machine"] = self.state_machine.snapshot()
            diagnostics["resource_released"] = False
            diagnostics["accepted_for_dispatch"] = False
            diagnostics["actual_finish_time"] = rospy.Time.now().to_sec()
            if diagnostics.get("safety_hold"):
                diagnostics["safety_hold"].update(verified=False, outcome="UNVERIFIED", error=text)
            self._write_diagnostics(diagnostics)
            result = self._result_message(diagnostics, FormationResult.UNKNOWN_LOCKED)
            self.terminal_goals.add(goal_id)
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
        handover=self._claim_air_references(work.goal_handle.get_goal_id().id)
        start = rospy.Time.now()
        diagnostics = self._new_diagnostics(work, start)
        diagnostics['reference_handover']=handover
        self.executing_diagnostics = diagnostics
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
                if (source is not None and source["source_trajectory_id"] >= 0 and
                    source.get('reference_source','AIR_SWARM')=='AIR_SWARM'):
                    adoption.note_qn_source(
                        agent_id, source["source_trajectory_id"],
                        source["used_outer_step"], source["ros_time_s"],
                        source["source_command_stamp"])
            self.active_diagnostics = diagnostics

        for boundary in handover:
            adoption.note_reference_handover(boundary['member'],boundary['retired_id'],boundary['used_outer_step'])

        command = PoseStamped()
        command.header.frame_id = "world"
        command.header.stamp = start
        command.pose.position = goal.formation_center.point
        command.pose.orientation.w = 1.0
        dispatch_time = rospy.Time.now().to_sec()
        adoption.begin_dispatch(goal.task_id, diagnostics["goal_id"], dispatch_time)
        # The meaning of the received position depends on the unit:
        #   one member   -> that member's own world target   (no slot offset)
        #   several      -> a formation centre               (each member offsets)
        # The planner enforces the same split on its side, so the two entries must
        # stay in step; see plan_manage_member_goal_entry.patch.
        with self.terminal_lock:
            if self._safety_trigger(diagnostics["goal_id"]):
                semantics, route, published_topics = "SAFETY_BEFORE_DISPATCH", [], set()
            else:
                semantics, route, published_topics = self._publish_routed_goal(command)
        diagnostics["goal_semantics"] = semantics
        diagnostics["goal_publish_count"] = 1
        diagnostics["goal_messages_published"] = len(published_topics)
        diagnostics["goal_topics"] = sorted(published_topics)
        diagnostics["dispatching_members"] = [agent for agent, _topic in route]
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
                    if self._safety_trigger(diagnostics["goal_id"]):
                        self._observe_safety_hold(work, diagnostics, alignment, member_samples)
                        break
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
                    scene_reason=self._scene_violation(samples)
                    if scene_reason:diagnostics['scene_violation']=scene_reason
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
                    if set(self.safety_agent_ids) != set(self.agent_ids):
                        fleet = self._fleet_safety(now_s)
                        previous = diagnostics.get("fleet_safety", {})
                        distances = [v for v in (fleet.get("min_surface_clearance_m"),
                                     previous.get("min_surface_clearance_m")) if v is not None]
                        fleet["min_surface_clearance_m"] = min(distances, default=None)
                        diagnostics["fleet_safety"] = fleet
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
                    # Persist the terminating observation before ending the loop.
                    # Otherwise the native Result can omit the very safety
                    # sample that caused its abort.
                    violation = self._violation_reason(diagnostics, snapshot)
                    if violation:
                        diagnostics["violation_latched_at_s"] = now_s
                        diagnostics["violation_reason"] = violation
                        if self._safety_trigger(diagnostics["goal_id"]):
                            self._observe_safety_hold(work, diagnostics, alignment, member_samples)
                        break
                    if snapshot.terminal_state:
                        break
                    rate.sleep()
        finally:
            with self.lock:
                self.active_diagnostics = None

        finish = rospy.Time.now()
        committed = self._finalize(diagnostics, work, monitor, adoption, alignment,
                                  member_samples, obstacle_clearances, reference_missing,
                                  start, finish, counts_start, group_goal_start)
        if not committed:
            self._observe_safety_hold(work, diagnostics, alignment, member_samples)
            self._finalize(diagnostics, work, monitor, adoption, alignment,
                           member_samples, obstacle_clearances, reference_missing,
                           start, rospy.Time.now(), counts_start, group_goal_start)

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

    def _scene_violation(self,samples):
        scene=getattr(self,'static_scene',None)
        if scene is None:return ''
        for agent in self.agent_ids:
            sample=self._latest_sample(samples.get(agent))
            if sample is not None:
                reason=scene.violation(sample.position,self.platform_radius_m)
                if reason:return 'member {} {}'.format(agent,reason)
        return ''

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
        fleet = diagnostics.get("fleet_safety")
        if diagnostics.get('scene_violation'):return diagnostics['scene_violation']
        if fleet is not None and not fleet["ok"]:
            return fleet["reason"]
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

    def _merge_online_safety(self, safety, diagnostics):
        """Retain online extrema when the coarser final ledger misses a sample."""
        fleet = diagnostics.get("fleet_safety") or {}
        if diagnostics.get('scene_violation'):
            safety.outcome='FAIL'
            safety.reasons+=(diagnostics['scene_violation'],)
        clearances = []
        distance = diagnostics.get("min_inter_agent_distance")
        if distance is not None:
            clearances.append(distance - 2 * self.platform_radius_m)
        if fleet.get("min_surface_clearance_m") is not None:
            clearances.append(fleet["min_surface_clearance_m"])
        if safety.min_inter_agent_surface_clearance_m is not None:
            clearances.append(safety.min_inter_agent_surface_clearance_m)
        if clearances:
            minimum = min(clearances)
            safety.min_inter_agent_surface_clearance_m = minimum
            safety.min_inter_agent_distance_m = minimum + 2 * self.platform_radius_m
            if minimum < self.inter_agent_clearance:
                safety.outcome = "FAIL"
                safety.reasons += ("online inter-agent surface clearance {:.3f} m below {:.2f} m".format(
                    minimum, self.inter_agent_clearance),)
        if fleet.get("ok") is False and fleet.get("reason") in (
                "missing or stale fleet odometry", "fleet odometry not time aligned"):
            if safety.outcome != "FAIL":
                safety.outcome = "NOT_VERIFIED"
            safety.reasons += (fleet["reason"],)

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
        self._merge_online_safety(safety, diagnostics)
        hold_gaps = [row["reason"] for row in (diagnostics.get("safety_hold") or {}).get("safety_failures", [])
                     if row["reason"] in ("missing or stale fleet odometry", "fleet odometry not time aligned")]
        if hold_gaps:
            if safety.outcome != "FAIL":
                safety.outcome = "NOT_VERIFIED"
            safety.reasons += tuple(sorted(set(hold_gaps)))
        # Persist the existing sample ledger inputs, not a second telemetry protocol.
        diagnostics["member_samples"] = {
            str(a): [asdict(row) for row in rows] for a, rows in member_samples.items()}
        diagnostics["sample_timeout_s"] = self.odom_timeout
        if len(self.agent_ids) > 1:
            from qn_aav_simulator.task_line import formation_interval_metrics
            times = sorted({row.ros_time_s for rows in member_samples.values() for row in rows})
            by_time = {t: {} for t in times}
            for a, rows in member_samples.items():
                for row in rows:
                    by_time[row.ros_time_s][str(a)] = row.position
            try:
                diagnostics["formation_geometry"] = formation_interval_metrics(
                    list(by_time.items()), {str(a): p for a, p in self.slots.items()},
                    sample_timeout_s=self.odom_timeout)
            except ValueError as error:
                diagnostics["formation_geometry"] = {"data_valid": False, "reason": str(error)}
        if diagnostics.get("safety_hold"):
            motion_completed = False
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
        if diagnostics.get("violation_reason"):
            reason = FormationResult.UNKNOWN_LOCKED
            text += "; " + diagnostics["violation_reason"]
        elif not motion_completed:
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
            and reason == FormationResult.NONE
            and adoption_verdict.state == "ADOPTED"
            and time_ok
            and air["ok"]
            and verdict.task_outcome == TASK_PASS
            and verdict.safety_outcome == SAFETY_PASS
            and verdict.experiment_validity == VALIDITY_VALID)
        with self.terminal_lock:
            if diagnostics["goal_id"] in self.terminal_goals:
                return True
            final_trigger = self._safety_trigger(diagnostics["goal_id"])
            if final_trigger and not diagnostics.get("safety_hold"):
                return False
            if final_trigger == "GROUP_MEMBER_SENSOR_FAILURE" and diagnostics["safety_hold"]["reason"] == "CANCEL_REQUEST":
                diagnostics["safety_hold"]["reason"] = final_trigger
                diagnostics["violation_reason"] = final_trigger
                text += "; " + final_trigger
                diagnostics["reason_text"] = text
            self.state_machine.ros_time_s = finish.to_sec()
            if accepted:
                if self.state_machine.state == "HOLDING":
                    self.state_machine.finish_success()
                if self.state_machine.state == "SUCCEEDED":
                    self.state_machine.release()
            else:
                self.state_machine.fault(text)
            diagnostics["accepted_for_dispatch"] = accepted
            diagnostics["run_state"] = self.state_machine.state
            diagnostics["state_machine"] = self.state_machine.snapshot()
            diagnostics["resource_released"] = self.state_machine.state == "READY_IDLE"
            self._write_diagnostics(diagnostics)
            result = self._result_message(diagnostics, reason)
            hold = diagnostics.get("safety_hold") or {}
            self.terminal_goals.add(diagnostics["goal_id"])
            if reason == FormationResult.NONE:
                work.goal_handle.set_succeeded(result, text)
            elif hold.get("reason") == "CANCEL_REQUEST" and hold.get("verified"):
                work.goal_handle.set_canceled(result, text)
            else:
                work.goal_handle.set_aborted(result, text)
        rospy.loginfo("FormationAction %s: %s", diagnostics["task_id"], text)
        return True


def main():
    rospy.init_node("formation_action_server")
    FormationActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
