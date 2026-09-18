#!/usr/bin/env python3
"""One native Swarm goal, with completion established by actual qn odometry."""

import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import re
import threading
import time
import xmlrpc.client

import actionlib
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from qn_aav_simulator.formation_monitor import (
    AGENT_IDS, DEFAULT_RELATIVE_SLOTS, GroupCompletionMonitor, OdometrySample,
    validate_configuration, validate_target,
)
from qn_aav_simulator.msg import FormationAction, FormationFeedback, FormationResult


class _TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


class FormationActionServer:
    def __init__(self):
        rospy.set_param("~ready", False)
        self.agent_ids = list(rospy.get_param("~agent_ids", list(AGENT_IDS)))
        self.scale = float(rospy.get_param("~swarm_scale", 2.0))
        self.epsilon_p = float(rospy.get_param("~epsilon_p", 0.5))
        self.epsilon_v = float(rospy.get_param("~epsilon_v", 0.25))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.25))
        self.execution_timeout = float(rospy.get_param("~execution_timeout", 180.0))
        self.monitor_rate = float(rospy.get_param("~monitor_rate", 20.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 60.0))
        self.slots = validate_configuration(
            self.agent_ids, rospy.get_param(
                "~relative_slots", {str(key): list(value)
                                    for key, value in DEFAULT_RELATIVE_SLOTS.items()}),
            self.scale, self.epsilon_p, self.epsilon_v,
            self.odom_timeout, self.execution_timeout)
        if not math.isfinite(self.monitor_rate) or self.monitor_rate <= 0.0:
            raise ValueError("monitor_rate must be finite and positive")
        if not math.isfinite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError("startup_timeout must be finite and positive")
        self.output_dir = Path(rospy.get_param("~output_dir", "/experiments/current"))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.odom = {}
        self.odom_counts = {agent_id: 0 for agent_id in self.agent_ids}
        self.map_seen = False
        self.depth_streams_seen = set()
        self.lock = threading.Lock()
        self.active_diagnostics = None
        self.ready = False
        self.master = rosgraph.Master(rospy.get_name())
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.subscribers = [rospy.Subscriber(
            "/map_generator/global_cloud", rospy.AnyMsg, self._map_callback, queue_size=1)]
        for agent_id in self.agent_ids:
            self.subscribers.extend([
                rospy.Subscriber("/drone_{}_visual_slam/odom".format(agent_id),
                                 Odometry, self._odom_callback,
                                 callback_args=agent_id, queue_size=1),
                rospy.Subscriber("/drone_{}_planning/finish".format(agent_id),
                                 Bool, self._finish_callback,
                                 callback_args=agent_id, queue_size=10),
                rospy.Subscriber("/drone_{}_pcl_render_node/depth".format(agent_id),
                                 rospy.AnyMsg, self._depth_callback,
                                 callback_args=agent_id, queue_size=1),
            ])
        self.server = actionlib.SimpleActionServer(
            "formation_action", FormationAction, self._execute, auto_start=False)
        self.ready_thread = threading.Thread(target=self._wait_until_ready, daemon=True)
        self.ready_thread.start()

    def _odom_callback(self, message, agent_id):
        p, v = message.pose.pose.position, message.twist.twist.linear
        sample = OdometrySample(message.header.stamp.to_sec(),
                                (p.x, p.y, p.z), (v.x, v.y, v.z),
                                message.header.frame_id)
        with self.lock:
            self.odom[agent_id] = sample
            self.odom_counts[agent_id] += 1

    def _map_callback(self, _message):
        with self.lock:
            self.map_seen = True

    def _depth_callback(self, _message, agent_id):
        with self.lock:
            self.depth_streams_seen.add(agent_id)

    def _finish_callback(self, message, agent_id):
        if not message.data:
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            diagnostics = self.active_diagnostics
            if diagnostics is not None and now >= diagnostics["actual_start_time"]:
                diagnostics["planner_nominal_finish_times"].setdefault(str(agent_id), now)

    def _planner_configuration_matches(self):
        for agent_id in self.agent_ids:
            prefix = "/drone_{}_ego_planner_node/".format(agent_id)
            configuration = rospy.get_param(prefix.rstrip("/"), {})
            if configuration.get("fsm", {}).get("flight_type") != 3:
                return "planner {} is not in formation goal mode".format(agent_id)
            if configuration.get("manager", {}).get("drone_id") != agent_id:
                return "planner {} has a different drone ID".format(agent_id)
            global_goal = configuration.get("global_goal", {})
            if not math.isclose(float(global_goal.get("swarm_scale", float("nan"))), self.scale,
                    rel_tol=0.0, abs_tol=1e-9):
                return "planner {} swarm_scale differs from monitor".format(agent_id)
            for member_id, slot in self.slots.items():
                for axis, value in zip(("x", "y", "z"), slot):
                    actual = global_goal.get("relative_pos_{}".format(member_id), {}).get(
                        axis, float("nan"))
                    if not math.isclose(float(actual), value, rel_tol=0.0, abs_tol=1e-9):
                        return "planner {} relative slots differ from monitor".format(agent_id)
        return None

    def _readiness_reason(self):
        with self.lock:
            samples = dict(self.odom)
            maps_ready = self.map_seen and set(self.agent_ids) <= self.depth_streams_seen
        now = rospy.Time.now().to_sec()
        if not all(agent_id in samples and samples[agent_id].is_fresh(now, self.odom_timeout)
                   for agent_id in self.agent_ids):
            return "waiting for fresh world-frame qn odometry from all seven members"
        if not maps_ready:
            return "waiting for the global map and seven local sensing depth streams"
        mismatch = self._planner_configuration_matches()
        if mismatch:
            return mismatch
        publishers, subscribers, _services = self.master.getSystemState()
        publishers, subscribers = dict(publishers), dict(subscribers)
        expected = {"/drone_{}_ego_planner_node".format(agent_id)
                    for agent_id in self.agent_ids}
        if not expected <= set(subscribers.get("/move_base_simple/goal", [])):
            return "waiting for all seven planner goal subscriptions"
        for agent_id in self.agent_ids:
            topic = "/drone_{}_visual_slam/odom".format(agent_id)
            if set(publishers.get(topic, [])) != {"/drone_{}_qn_aav".format(agent_id)}:
                return "odometry {} must have exactly its qn publisher".format(agent_id)
        uri = self.master.lookupNode(rospy.get_name())
        with xmlrpc.client.ServerProxy(uri, transport=_TimeoutTransport()) as proxy:
            code, _message, connections = proxy.getBusInfo(rospy.get_name())
        connected = {connection[1] for connection in connections
                     if len(connection) >= 6 and connection[2] == "o"
                     and connection[4] == "/move_base_simple/goal" and connection[5]}
        if code != 1 or not expected <= connected:
            return "waiting for established TCPROS goal connections to all seven planners"
        return None

    def _wait_until_ready(self):
        deadline = time.monotonic() + self.startup_timeout
        while not rospy.is_shutdown():
            try:
                reason = self._readiness_reason()
            except (OSError, ValueError, TypeError, xmlrpc.client.Error,
                    rosgraph.MasterException, rosgraph.MasterError) as error:
                reason = "readiness check: {}".format(error)
            if reason is None:
                self.server.start()
                self.ready = True
                rospy.set_param("~ready", True)
                rospy.set_param("~readiness_reason", "ready")
                rospy.loginfo("FormationAction ready: seven qn states and planner connections verified")
                return
            rospy.set_param("~readiness_reason", reason)
            rospy.loginfo_throttle(5.0, reason)
            if time.monotonic() >= deadline:
                rospy.logerr("FormationAction startup timed out: %s", reason)
                rospy.signal_shutdown("formation startup timeout: " + reason)
                return
            rospy.rostime.wallsleep(0.5)

    def _new_diagnostics(self, goal, start):
        return {
            "task_id": goal.task_id,
            "goal_id": self.server.current_goal.get_goal_id().id,
            "formation_center": [goal.formation_center.point.x,
                                 goal.formation_center.point.y,
                                 goal.formation_center.point.z],
            "hold_duration": goal.hold_duration.to_sec(),
            "actual_start_time": start.to_sec(),
            "goal_publish_count": 0,
            "planner_nominal_finish_times": {},
            "max_position_error": None,
            "max_velocity": None,
            "min_inter_agent_distance": None,
            "position_not_settled": False,
            "velocity_not_settled": False,
            "planner_finished_early": False,
            "successful_hold_window": None,
        }

    def _complete(self, diagnostics, path, state, reason, finish, counts_start,
                  snapshot=None, text=""):
        with self.lock:
            counts = {str(agent_id): self.odom_counts[agent_id] - counts_start[agent_id]
                      for agent_id in self.agent_ids}
            self.active_diagnostics = None
        elapsed = finish.to_sec() - diagnostics["actual_start_time"]
        diagnostics.update(
            terminal_state=state, reason=reason, actual_finish_time=finish.to_sec(),
            final_phase=snapshot.phase if snapshot else "MOVING",
            failed_agent_ids=list(snapshot.stale_agent_ids) if snapshot else [],
            odom_sample_counts=counts,
            odom_sample_rates={key: value / elapsed if elapsed > 0.0 else 0.0
                               for key, value in counts.items()},
            diagnostic_message=text)
        if state == "SUCCEEDED":
            diagnostics["successful_hold_window"] = {
                "start": snapshot.hold_started, "end": finish.to_sec(),
                "duration": snapshot.hold_elapsed,
                "required_duration": diagnostics["hold_duration"],
            }
        diagnostics["planner_finish_to_actual_seconds"] = {
            key: finish.to_sec() - nominal
            for key, nominal in diagnostics["planner_nominal_finish_times"].items()}
        # Invalid coordinates can be NaN/Inf; diagnostics remain portable JSON.
        def finite_json(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: finite_json(item) for key, item in value.items()}
            if isinstance(value, list):
                return [finite_json(item) for item in value]
            return value
        path.write_text(json.dumps(finite_json(diagnostics), indent=2, allow_nan=False) + "\n")
        result = FormationResult()
        result.actual_start_time = rospy.Time.from_sec(diagnostics["actual_start_time"])
        result.actual_finish_time = finish
        result.reason = reason
        if state == "SUCCEEDED":
            self.server.set_succeeded(result, text)
        elif state == "PREEMPTED":
            self.server.set_preempted(result, text)
        else:
            self.server.set_aborted(result, text)

    def _execute(self, goal):
        start = rospy.Time.now()
        diagnostics = self._new_diagnostics(goal, start)
        stem = "{}_{}".format(re.sub(r"[^A-Za-z0-9_-]", "_", goal.task_id)[:80] or "task",
                              start.to_nsec())
        diagnostic_path = self.output_dir / (stem + ".diagnostics.json")
        with self.lock:
            counts_start = dict(self.odom_counts)
        try:
            center = validate_target(goal.formation_center.header.frame_id,
                                     diagnostics["formation_center"],
                                     goal.hold_duration.to_sec())
            if not goal.task_id.strip():
                raise ValueError("task_id must not be empty")
        except (ValueError, TypeError) as error:
            self._complete(diagnostics, diagnostic_path, "ABORTED",
                           GroupCompletionMonitor.INVALID_TARGET, rospy.Time.now(),
                           counts_start, text=str(error))
            return
        monitor = GroupCompletionMonitor(
            center, goal.hold_duration.to_sec(), start.to_sec(),
            agent_ids=self.agent_ids, relative_slots=self.slots, swarm_scale=self.scale,
            epsilon_p=self.epsilon_p, epsilon_v=self.epsilon_v,
            odom_timeout=self.odom_timeout, execution_timeout=self.execution_timeout)
        with self.lock:
            samples = dict(self.odom)
        now = rospy.Time.now()
        initial = monitor.evaluate(now.to_sec(), samples,
                                   cancelled=self.server.is_preempt_requested())
        if initial.terminal_state in ("ABORTED", "PREEMPTED"):
            self._complete(diagnostics, diagnostic_path, initial.terminal_state,
                           initial.reason, now, counts_start, initial)
            return

        command = PoseStamped()
        command.header.frame_id = "world"
        command.header.stamp = start
        command.pose.position = goal.formation_center.point
        command.pose.orientation.w = 1.0
        with self.lock:
            self.active_diagnostics = diagnostics
        self.goal_pub.publish(command)
        diagnostics["goal_publish_count"] = 1
        previous_phase = None
        fields = ["ros_time", "elapsed", "phase", "max_position_error", "max_velocity",
                  "min_inter_agent_distance", "fresh_agent_count", "stale_agent_ids",
                  "hold_elapsed"]
        with (self.output_dir / (stem + ".monitor.csv")).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            rate = rospy.Rate(self.monitor_rate)
            while not rospy.is_shutdown():
                with self.lock:
                    samples = dict(self.odom)
                now = rospy.Time.now()
                snapshot = monitor.evaluate(now.to_sec(), samples,
                                            cancelled=self.server.is_preempt_requested())
                if snapshot.phase != previous_phase:
                    feedback = FormationFeedback()
                    feedback.phase = (FormationFeedback.HOLDING if snapshot.phase == "HOLDING"
                                      else FormationFeedback.MOVING)
                    self.server.publish_feedback(feedback)
                    previous_phase = snapshot.phase
                for name in ("max_position_error", "max_velocity"):
                    value = getattr(snapshot, name)
                    if value is not None:
                        diagnostics[name] = max(diagnostics[name] or 0.0, value)
                distance = snapshot.min_inter_agent_distance
                if distance is not None:
                    old = diagnostics["min_inter_agent_distance"]
                    diagnostics["min_inter_agent_distance"] = distance if old is None else min(old, distance)
                diagnostics["position_not_settled"] |= (
                    snapshot.max_position_error is not None and snapshot.max_position_error > self.epsilon_p)
                diagnostics["velocity_not_settled"] |= (
                    snapshot.max_velocity is not None and snapshot.max_velocity > self.epsilon_v)
                with self.lock:
                    if diagnostics["planner_nominal_finish_times"] and snapshot.phase != "HOLDING":
                        diagnostics["planner_finished_early"] = True
                row = {key: value for key, value in asdict(snapshot).items() if key in fields}
                row.update(ros_time=now.to_sec(), elapsed=(now - start).to_sec(),
                           stale_agent_ids=";".join(map(str, snapshot.stale_agent_ids)))
                writer.writerow(row)
                stream.flush()
                if snapshot.terminal_state:
                    stream.close()
                    self._complete(diagnostics, diagnostic_path, snapshot.terminal_state,
                                   snapshot.reason, now, counts_start, snapshot)
                    return
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
            stream.close()
            self._complete(diagnostics, diagnostic_path, "PREEMPTED", 0,
                           rospy.Time.now(), counts_start, monitor.snapshot,
                           "node shutdown; the native motion backend is not stopped by this action")


def main():
    rospy.init_node("formation_action_server")
    FormationActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
