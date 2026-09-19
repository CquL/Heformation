#!/usr/bin/env python3
"""Live view of a running mission: allocation -> dispatch -> flight -> control.

A passive observer.  It subscribes to the topics the mission already publishes
and never publishes anything the control chain consumes: the only outputs are a
matplotlib window (optional) and an image topic for RViz / rqt_image_view.  If
this node dies or is never started, the mission is unaffected.

Panels, all live:
  scene        the seven members, their trails, the slot layout around the
               formation centre currently commanded, that centre, the obstacle
  status       the task in flight, its phase, reference adoption per member,
               liveness, and the verdicts as they arrive
  slot error   distance from each member to its slot, against epsilon_p, with
               the hold window marked
  envelope     altitude against the AIR floor, and the closest approach between
               members against the required clearance

The offline renderers (plot_mission_overview.py, animate_mission_replay.py) show
what the recording settles as.  This shows what is true right now.
"""

from __future__ import annotations

import math
import json
import sys
import threading
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_mission_overview import AIR_FLOOR_M_FALLBACK as AIR_FLOOR_M, slot_target  # noqa: E402

AGENTS = tuple(range(7))
COLOURS = matplotlib.pyplot.get_cmap("tab10")
PHASES = {0: "MOVING", 1: "HOLDING"}
OUTCOMES = {0: "-", 1: "PASS", 2: "FAIL"}
SAFETY = {0: "-", 1: "PASS", 2: "FAIL", 3: "NOT_VERIFIED"}
VALIDITY = {0: "-", 1: "VALID", 2: "INVALID", 3: "INCOMPLETE"}


class MissionDashboard:
    def __init__(self):
        self.lock = threading.Lock()
        self.history_s = float(rospy.get_param("~history_seconds", 180.0))
        self.rate = float(rospy.get_param("~rate", 5.0))
        self.show_window = bool(rospy.get_param("~window", False))
        self.publish_raw = bool(rospy.get_param("~publish_raw", False))
        if self.show_window:
            matplotlib.use("TkAgg", force=True)

        self.planning_mode = rospy.get_param("~planning_mode", "fixed_coalition")
        if self.planning_mode == "executor":
            import matplotlib.pyplot as plt
            self.figure, self.task_axis = plt.subplots(figsize=(13, 8))
            if self.show_window:
                self.figure.canvas.manager.set_window_title("Monitoring request — live task status")
                window = self.figure.canvas.manager.window
                width = min(1300, window.winfo_screenwidth() // 2 - 40)
                window.geometry("{}x850+{}+70".format(width, window.winfo_screenwidth() // 2 + 20))
            self.image_publisher = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)
            self.raw_publisher = None
            return
        self.epsilon_p = float(rospy.get_param("/formation_action_server/epsilon_p", 0.5))
        self.platform_radius = float(
            rospy.get_param("/formation_action_server/platform_radius_m", 0.25))
        self.inter_agent_clearance = float(
            rospy.get_param("/formation_action_server/inter_agent_clearance", 0.5))
        self.odom_timeout = float(
            rospy.get_param("/formation_action_server/odom_timeout", 0.25))
        self.obstacle = bool(rospy.get_param("/formation_mission_runner/obstacle_scenario",
                                             False))
        self.obstacle_center = rospy.get_param("/scene/obstacle_center",
                                               rospy.get_param("~obstacle_center",
                                                               [-23.0, 0.0, 0.5]))
        self.obstacle_size = rospy.get_param("/scene/obstacle_size",
                                             rospy.get_param("~obstacle_size",
                                                             [1.0, 1.0, 1.2]))
        slots = rospy.get_param("/formation_action_server/relative_slots", None)
        if not slots:
            # Without the slot layout every member would be measured against its
            # own origin and the panel would show a flat, meaningless error.
            rospy.logwarn("relative_slots missing on /formation_action_server: the slot "
                          "layout is degenerate. Start this node from "
                          "formation_air.launch, or load config/formation_air.yaml first.")
            slots = {str(a): [0.0, 0.0, 0.0] for a in AGENTS}
        self.monitor = {
            "relative_slots": slots,
            "swarm_scale": float(rospy.get_param("/formation_action_server/swarm_scale", 2.0)),
            "start_center": rospy.get_param("~start_center", [-26.0, 0.0, 0.8]),
        }

        span = int(self.history_s * 20.0)
        self.odometry = {a: deque(maxlen=span) for a in AGENTS}
        self.model_time = {a: deque(maxlen=span) for a in AGENTS}
        self.latest = {a: None for a in AGENTS}
        self.diagnostic = {a: {} for a in AGENTS}
        self.centre = None
        self.centre_stamp = None
        # The commanded centre over time.  Measuring every past sample against the
        # centre that is current *now* would silently rewrite history: the panel
        # would show a step change at each dispatch instead of the error that was
        # actually there at the time.
        self.centres = deque(maxlen=512)
        self.hold_until = None
        self.task_id = "-"
        self.hold_duration = None
        self.phase = None
        self.status_text = "-"
        self.results = []
        self.started = None
        self.peak_drift = 0.0
        self.min_height = None
        self.max_medium = 0.0
        self.adoption = {}
        self.clearance_history = deque(maxlen=int(self.history_s * 5.0))

        self.figure, self.axes = self.build_figure()
        self.image_publisher = rospy.Publisher("~image/compressed", CompressedImage,
                                               queue_size=1)
        self.raw_publisher = (rospy.Publisher("~image", CompressedImage, queue_size=1)
                              if self.publish_raw else None)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_centre)
        rospy.Subscriber("/formation_action/status", GoalStatusArray, self.on_status)
        for agent in AGENTS:
            rospy.Subscriber("/drone_{}_qn/odometry".format(agent), Odometry,
                             self.on_odometry, callback_args=agent, queue_size=20)
            rospy.Subscriber("/drone_{}_qn/diagnostics".format(agent), DiagnosticArray,
                             self.on_diagnostics, callback_args=agent, queue_size=20)
        self.subscribe_action()

    def subscribe_action(self):
        try:
            from qn_aav_simulator.msg import FormationFeedback
        except ImportError:
            rospy.logwarn("Formation messages unavailable; phase will not be shown")
            return
        rospy.Subscriber("/formation_action/feedback", FormationFeedback, self.on_feedback)
        try:
            from qn_aav_simulator.msg import FormationActionResult
        except ImportError:
            return
        rospy.Subscriber("/formation_action/result", FormationActionResult, self.on_result)
        from qn_aav_simulator.msg import FormationActionGoal
        rospy.Subscriber("/formation_action/goal", FormationActionGoal, self.on_action_goal)

    # ---- callbacks: store only, never draw and never block -------------------
    def on_odometry(self, message, agent):
        stamp = message.header.stamp.to_sec()
        point = message.pose.pose.position
        with self.lock:
            self.odometry[agent].append((stamp, point.x, point.y, point.z))
            self.latest[agent] = (stamp, point.x, point.y, point.z)
            if self.started is None:
                self.started = stamp

    def on_diagnostics(self, message, agent):
        values = {entry.key: entry.value
                  for status in message.status for entry in status.values}
        if "model_time_s" not in values:
            return
        stamp = message.header.stamp.to_sec()
        model = float(values["model_time_s"])
        height = float(values.get("position_z", values.get("min_height_m", 0.0)))
        with self.lock:
            self.model_time[agent].append((stamp, model))
            self.diagnostic[agent] = values
            self.adoption[agent] = int(values.get("source_trajectory_id", -1))
            self.min_height = height if self.min_height is None else min(self.min_height,
                                                                        height)
            self.max_medium = max(self.max_medium, float(values.get("medium_flag", 0.0)))
            if len(self.model_time[agent]) > 1:
                first_stamp, first_model = self.model_time[agent][0]
                drift = (model - first_model) - (stamp - first_stamp)
                self.peak_drift = max(self.peak_drift, abs(drift))

    def on_centre(self, message):
        with self.lock:
            centre = (message.pose.position.x, message.pose.position.y,
                      message.pose.position.z)
            self.centre = centre
            self.centre_stamp = message.header.stamp.to_sec()
            self.centres.append((self.centre_stamp, centre))
            self.started = self.started if self.started is not None else self.centre_stamp

    def on_action_goal(self, message):
        with self.lock:
            self.task_id = message.goal.task_id
            self.hold_duration = message.goal.hold_duration.to_sec()
            self.hold_until = None
            self.phase = None
            point = message.goal.formation_center.point
            if self.centre is None:
                self.centre = (point.x, point.y, point.z)
                self.centre_stamp = message.header.stamp.to_sec()
                self.centres.append((self.centre_stamp, self.centre))

    def on_feedback(self, message):
        with self.lock:
            self.phase = message.phase
            now = rospy.Time.now().to_sec()
            if message.phase == 1 and self.hold_duration:
                self.hold_until = now + self.hold_duration

    def on_result(self, message):
        with self.lock:
            status = message.status
            self.results.append((
                self.task_id,
                OUTCOMES.get(status.task_outcome, "?"),
                SAFETY.get(status.safety_outcome, "?"),
                VALIDITY.get(status.experiment_validity, "?"),
                status.model_time_hold_seconds))

    def on_status(self, message):
        with self.lock:
            for status in message.status_list:
                self.status_text = {
                    0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED",
                    4: "ABORTED", 5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING",
                    8: "RECALLED", 9: "LOST"}.get(status.status, str(status.status))

    # ---- rendering ----------------------------------------------------------
    def build_figure(self):
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
        figure = plt.figure(figsize=(14, 7.6))
        grid = GridSpec(2, 2, figure=figure, hspace=0.34, wspace=0.24, top=0.90,
                        bottom=0.09, left=0.06, right=0.97)
        scene, status = figure.add_subplot(grid[0, 0]), figure.add_subplot(grid[0, 1])
        slot, envelope = figure.add_subplot(grid[1, 0]), figure.add_subplot(grid[1, 1])
        status.axis("off")
        scene.set_aspect("equal", adjustable="box")
        scene.set_xlabel("x [m]")
        scene.set_ylabel("y [m]")
        scene.set_title("scene: members, trails and the commanded slot layout")
        scene.grid(alpha=0.3)
        if self.obstacle:
            from matplotlib.patches import Rectangle
            centre, size = self.obstacle_center, self.obstacle_size
            scene.add_patch(Rectangle((centre[0] - size[0] / 2.0, centre[1] - size[1] / 2.0),
                                      size[0], size[1], facecolor="#c0392b", alpha=0.3,
                                      edgecolor="#c0392b", linewidth=1.4,
                                      label="declared obstacle"))
        self.trail = {}
        self.dot = {}
        for agent in AGENTS:
            line, = scene.plot([], [], linewidth=1.4, alpha=0.9, color=COLOURS(agent % 10),
                               label="drone_{}".format(agent))
            self.trail[agent] = line
            self.dot[agent] = scene.scatter([], [], s=110, color=COLOURS(agent % 10),
                                            edgecolor="k", linewidth=0.8, zorder=5)
        self.centre_mark = scene.scatter([], [], marker="*", s=280, color="k", zorder=6)
        self.slot_line, = scene.plot([], [], linestyle="--", linewidth=1.4, color="0.35",
                                     zorder=4)
        scene.legend(fontsize=7, ncol=4, loc="upper center", framealpha=0.9)

        slot.set_ylabel("slot error [m]")
        slot.set_xlabel("time since mission start [s]")
        slot.set_title("control: distance to slot   epsilon_p {:.2f} m".format(self.epsilon_p))
        slot.grid(alpha=0.3)
        self.slot_lines = {}
        for agent in AGENTS:
            line, = slot.plot([], [], linewidth=0.9, color=COLOURS(agent % 10))
            self.slot_lines[agent] = line
        slot.axhline(self.epsilon_p, color="#c0392b", linestyle="--", linewidth=1.1)
        self.hold_band = slot.axvspan(0, 0, color="#cfe0f5", zorder=0, alpha=0.0)

        envelope.set_ylabel("altitude [m]")
        envelope.set_xlabel("time since first odometry sample [s]")
        envelope.set_title("safety: altitude and closest approach")
        envelope.grid(alpha=0.3)
        self.altitude_lines = {}
        for agent in AGENTS:
            line, = envelope.plot([], [], linewidth=0.9, color=COLOURS(agent % 10))
            self.altitude_lines[agent] = line
        envelope.axhline(AIR_FLOOR_M, color="#c0392b", linestyle="--", linewidth=1.1,
                         label="AIR floor hg/2 = {:.3f} m".format(AIR_FLOOR_M))
        self.clearance_line, = envelope.plot([], [], linewidth=1.6, color="#1f6f3f",
                                             label="min inter-agent surface clearance")
        envelope.axhline(self.inter_agent_clearance, color="#1f6f3f", linestyle=":",
                         linewidth=1.1, label="required {:.2f} m".format(
                             self.inter_agent_clearance))
        envelope.legend(fontsize=7, loc="lower right")
        self.figure = figure
        self.ax_scene, self.ax_status = scene, status
        self.ax_slot, self.ax_envelope = slot, envelope
        return figure, (scene, status, slot, envelope)

    def snapshot(self):
        with self.lock:
            return {
                "odometry": {a: list(self.odometry[a]) for a in AGENTS},
                "model_time": {a: list(self.model_time[a]) for a in AGENTS},
                "latest": dict(self.latest),
                "centre": self.centre,
                "centre_stamp": self.centre_stamp,
                "centres": list(self.centres),
                "task_id": self.task_id,
                "hold_until": self.hold_until,
                "hold_duration": self.hold_duration,
                "phase": self.phase,
                "status_text": self.status_text,
                "results": list(self.results),
                "started": self.started,
                "peak_drift": self.peak_drift,
                "min_height": self.min_height,
                "max_medium": self.max_medium,
                "adoption": dict(self.adoption),
            }

    def draw(self, now):
        if self.planning_mode == "executor":
            return self.draw_task_authority(now)
        snapshot = self.snapshot()
        started = snapshot["started"]
        if started is None:
            self.ax_scene.set_title("waiting for odometry ...")
            return self.render()
        elapsed = now - started
        centre = snapshot["centre"] or self.monitor["start_center"]
        self.ax_scene.set_title(
            "scene: task {}, centre ({:.2f}, {:.2f}, {:.2f})".format(
                snapshot["task_id"], centre[0], centre[1], centre[2]))

        for agent in AGENTS:
            rows = snapshot["odometry"][agent]
            if rows:
                self.trail[agent].set_data([row[1] for row in rows], [row[2] for row in rows])
                self.dot[agent].set_offsets([[rows[-1][1], rows[-1][2]]])
        self.centre_mark.set_offsets([[centre[0], centre[1]]])
        targets = [slot_target(self.monitor, centre, agent) for agent in AGENTS]
        loop = list(range(len(AGENTS))) + [0]
        self.slot_line.set_data([targets[a][0] for a in loop], [targets[a][1] for a in loop])
        self.ax_scene.set_xlim(centre[0] - 9.0, centre[0] + 9.0)
        self.ax_scene.set_ylim(centre[1] - 9.0, centre[1] + 9.0)

        timeline = snapshot["centres"]
        worst = 0.0
        for agent in AGENTS:
            rows = np.array(snapshot["odometry"][agent], dtype=float)
            if rows.size:
                slot = np.array(self.monitor["relative_slots"][str(agent)], dtype=float)
                scale = float(self.monitor["swarm_scale"])
                # Samples taken before the first dispatch belong to the start
                # formation, not to the first commanded centre.
                centres = np.array([self.monitor["start_center"]]
                                   + [row[1] for row in timeline])
                when = [-1e18] + [row[0] for row in timeline]
                index = np.searchsorted(when, rows[:, 0], side="right") - 1
                index = np.clip(index, 0, len(centres) - 1)
                targets = centres[index] + scale * slot
                stamps = rows[:, 0] - started
                errors = np.linalg.norm(rows[:, 1:4] - targets, axis=1)
                self.slot_lines[agent].set_data(stamps, errors)
                self.altitude_lines[agent].set_data(stamps, rows[:, 3])
                worst = max(worst, float(errors.max()))
            else:
                self.slot_lines[agent].set_data([], [])
                self.altitude_lines[agent].set_data([], [])
        current = [row[1:4] for row in snapshot["latest"].values() if row]
        if len(current) > 1:
            closest = min(math.dist(one, other) for index, one in enumerate(current)
                          for other in current[index + 1:])
            self.clearance_history.append((elapsed,
                                           closest - 2.0 * self.platform_radius))
        if self.clearance_history:
            self.clearance_line.set_data([row[0] for row in self.clearance_history],
                                         [row[1] for row in self.clearance_history])

        self.ax_slot.set_xlim(max(0.0, elapsed - self.history_s), max(elapsed, 10.0))
        self.ax_slot.set_ylim(0.0, max(self.epsilon_p * 2.0, worst * 1.15))
        self.ax_envelope.set_xlim(self.ax_slot.get_xlim())
        ceiling = 1.4
        if self.clearance_history:
            ceiling = max(ceiling, max(row[1] for row in self.clearance_history) * 1.1)
        self.ax_envelope.set_ylim(0.0, ceiling)
        hold = snapshot["hold_until"]
        if hold:
            self.hold_band.set_xy([[hold - (snapshot["hold_duration"] or 0) - started, 0],
                                   [hold - started, self.ax_slot.get_ylim()[1]]])
            self.hold_band.set_alpha(0.35)
        self.draw_status(snapshot, elapsed, current, centre)
        return self.render()

    def draw_status(self, snapshot, elapsed, current, commanded):
        self.ax_status.clear()
        self.ax_status.axis("off")
        adoption = [value for value in snapshot["adoption"].values() if value >= 0]
        fresh = [agent for agent, row in snapshot["latest"].items()
                 if row and rospy.Time.now().to_sec() - row[0] <= self.odom_timeout]
        hold_left = max(0.0, (snapshot["hold_until"] or 0.0) - rospy.Time.now().to_sec())
        lines = [
            "elapsed            {:.1f} s".format(elapsed),
            "task in flight     {}  ({})".format(snapshot["task_id"],
                                                 PHASES.get(snapshot["phase"], "-")),
            "action state       {}".format(snapshot["status_text"]),
            "commanded centre   ({:.2f}, {:.2f}, {:.2f})".format(*commanded),
            "hold remaining     {:.1f} s".format(hold_left) if snapshot["hold_until"]
            else "hold remaining     -",
            "",
            "qn trajectory_id   {}".format(
                "{} (all adopted)".format(adoption[0])
                if len(set(adoption)) == 1 and len(adoption) == len(AGENTS)
                else sorted(set(adoption)) or "-"),
            "fresh odometry     {}/7".format(len(fresh)),
            "peak model-ROS     {:.5f} s".format(snapshot["peak_drift"]),
            "min altitude       {}".format(
                "-" if snapshot["min_height"] is None
                else "{:.3f} m".format(snapshot["min_height"])),
            "max medium flag    {:.0f}".format(snapshot["max_medium"]),
            "",
            "task   task/safety/validity   model hold",
        ]
        for task, task_out, safety, validity, hold in snapshot["results"]:
            lines.append("{:<6} {:>4} / {:<12} / {:<10} {:.2f} s".format(
                task, task_out, safety, validity, hold))
        if not snapshot["results"]:
            lines.append("(no result yet)")
        self.ax_status.text(0.0, 1.0, "\n".join(lines), transform=self.ax_status.transAxes,
                            fontsize=9, va="top", family="monospace")
        self.ax_status.set_title("live status: allocation, adoption, liveness and verdicts")

    def draw_task_authority(self, now):
        raw = rospy.get_param("/formation_mission_runner/task_state", "{}")
        state = json.loads(raw)
        axis = self.task_axis
        axis.clear()
        axis.axis("off")
        age = now - state.get("updated_at_ros_s", now)
        action = state.get("current_action") or {}
        lines = ["TASK AUTHORITY — " + state.get("request_id", "waiting for runner"),
                 "Status: {}   update age: {:.1f}s".format(state.get("status", "UNAVAILABLE"), age),
                 "Current action: {} / {}".format(action.get("task_id", "none"), action.get("phase", "-")),
                 "Selected endpoint: " + action.get("endpoint", "-"),
                 "Occupied / locked units: " + str(state.get("resource_locks", [])),
                 "Observed geometry: {}   Received observations: {}".format(
                     state.get("observed_fraction", "unknown"), state.get("delivered_fraction", "unknown")),
                 "Received results: " + str(state.get("results_received", [])),
                 "Failure: " + state.get("failure_reason", ""),
                 "Image/payload quality: " + state.get("payload_quality", "UNVERIFIED"), "",
                 "Task                        Executor             Status          Planned start / finish"]
        for item in (state.get("plan") or {}).get("items", []):
            lines.append("{:<27} {:<20} {:<15} {:.1f} / {:.1f}".format(
                item["task_id"], item["executor_id"], item["status"], item["planned_start"], item["planned_finish"]))
        lines.extend(["", "Point observations (geometric surrogate, no camera-quality claim):"])
        coverage = state.get("coverage") or {}
        received = coverage.get("delivered_point_ids", [])
        for point, observation in coverage.get("points", {}).items():
            lines.append("{}: observed={} received={} member={} dwell={:.2f}s".format(
                point, observation["observed"], point in received,
                observation["member_id"], observation["dwell_s"]))
        axis.text(.01, .99, "\n".join(lines), transform=axis.transAxes, va="top",
                  fontsize=10, family="monospace", wrap=True)
        self.figure.tight_layout()
        return self.render()

    def render(self):
        self.figure.canvas.draw()
        return np.asarray(self.figure.canvas.buffer_rgba())[:, :, :3].copy()

    def spin(self):
        import cv2
        import matplotlib.pyplot as plt
        rate = rospy.Rate(self.rate)
        if self.show_window:
            try:
                plt.ion()
                plt.show(block=False)
            except Exception as error:  # pragma: no cover - depends on the display
                rospy.logwarn("no GUI backend (%s); publishing images only", error)
                self.show_window = False
        while not rospy.is_shutdown():
            rgb = self.draw(rospy.Time.now().to_sec())
            ok, buffer = cv2.imencode(".jpg", rgb[:, :, ::-1],
                                      [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                message = CompressedImage()
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = "world"
                message.format = "jpeg"
                message.data = buffer.tobytes()
                self.image_publisher.publish(message)
                if self.raw_publisher is not None:
                    self.raw_publisher.publish(message)
            if self.show_window:
                plt.pause(0.001)
            rate.sleep()


def main():
    rospy.init_node("mission_dashboard")
    dashboard = MissionDashboard()
    rospy.loginfo("mission dashboard publishing on %s", rospy.resolve_name("~image/compressed"))
    dashboard.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
