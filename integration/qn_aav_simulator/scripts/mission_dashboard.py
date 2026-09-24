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
import re
import sys
import threading
import textwrap
import time
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
        if self.planning_mode in ('five_qualification', 'water_cooperation'):
            self.init_five_view()
            return
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
        if self.planning_mode == 'water_cooperation':
            return self.draw_water_cooperation(now)
        if self.planning_mode == 'five_qualification':
            return self.draw_five_view()
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

    def init_five_view(self):
        """Read native messages only; no generated business-completion state."""
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        from qn_aav_simulator.msg import (FormationActionGoal,FormationActionResult,
                                        PlatformTaskActionGoal,PlatformTaskActionResult)
        font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        self.five_font = font_manager.FontProperties(fname=font)
        self.figure,self.task_axis=plt.subplots(figsize=(8,7))
        self.figure.canvas.manager.set_window_title('五平台实验 · 动作状态')
        if self.show_window:
            self.figure.canvas.manager.window.geometry('640x700+1480+50')
        self.image_publisher=rospy.Publisher('~image/compressed',CompressedImage,queue_size=1)
        self.raw_publisher=None
        self.five_actions={}
        self.five_status={}
        self.five_active={}
        self.five_subs=[]
        self.delivery_progress={}
        if self.planning_mode == 'water_cooperation':
            from std_msgs.msg import String
            def progress(msg):
                with self.lock:self.delivery_progress=json.loads(msg.data)
            self.five_subs.append(rospy.Subscriber('/scene/delivery_progress',String,progress,queue_size=1))
            self.figure.canvas.manager.set_window_title('水下协作 · 任务与母船接收')
            if self.show_window:
                window=self.figure.canvas.manager.window
                window.geometry('760x760+{}+30'.format(max(0,window.winfo_screenwidth()-780)))
        endpoints=[('/aav_'+str(i+1)+'/formation_action',i,FormationActionGoal,FormationActionResult) for i in range(3)]
        endpoints += [('/drone_0_qn_aav/platform_task',0,PlatformTaskActionGoal,PlatformTaskActionResult),
                      ('/usv/platform_task',3,PlatformTaskActionGoal,PlatformTaskActionResult),
                      ('/uuv/platform_task',4,PlatformTaskActionGoal,PlatformTaskActionResult)]
        def goal(msg, endpoint, member):
            with self.lock:
                self.five_actions[(endpoint,msg.goal_id.id)]=dict(member=member,task=msg.goal.task_id,
                    state='已发送',result=False,at=msg.goal_id.stamp.to_sec())
        def status(msg, endpoint, member):
            names={0:'待接纳',1:'执行中',2:'已取消',3:'结束，等待结果',4:'失败，等待结果',5:'已拒绝',6:'取消处理中',7:'撤回中',8:'已撤回',9:'状态未知'}
            with self.lock:
                self.five_active[endpoint]=(member,any(s.status in (0,1,6,7) for s in msg.status_list),time.monotonic())
                for s in msg.status_list:
                    row=self.five_actions.get((endpoint,s.goal_id.id))
                    if row and not row['result']:row['state']=names.get(s.status,'未知')
        def result(msg, endpoint, member):
            names={2:'已取消',3:'动作完成',4:'动作失败',5:'已拒绝',8:'已撤回'}
            with self.lock:
                # Goal is not latched: a passive display can miss it while the
                # Action client/server still work correctly. Result carries its
                # own task identity; never invent completion from position.
                row=self.five_actions.setdefault((endpoint,msg.status.goal_id.id),
                    dict(member=member,task=msg.result.task_id,at=msg.status.goal_id.stamp.to_sec()))
                row.update(state=names.get(msg.status.status,'未知终态'),result=True)
        def diagnostic(msg, member):
            values={v.key:v.value for s in msg.status for v in s.values}
            with self.lock:self.five_status[member]=(values,time.monotonic())
        for endpoint,member,G,R in endpoints:
            self.five_subs.extend([
                rospy.Subscriber(endpoint+'/goal',G,lambda m,e=endpoint,i=member:goal(m,e,i),queue_size=20),
                rospy.Subscriber(endpoint+'/status',GoalStatusArray,lambda m,e=endpoint,i=member:status(m,e,i),queue_size=10),
                rospy.Subscriber(endpoint+'/result',R,lambda m,e=endpoint,i=member:result(m,e,i),queue_size=20)])
        for i,prefix in enumerate(['/drone_0_qn','/drone_1_qn','/drone_2_qn','/usv','/uuv']):
            self.five_subs.append(rospy.Subscriber(prefix+'/diagnostics',DiagnosticArray,
                lambda m,i=i:diagnostic(m,i),queue_size=1))

    def draw_five_view(self):
        axis=self.task_axis
        axis.clear();axis.axis('off')
        def text(*args, **kwargs):
            return axis.text(*args, fontproperties=self.five_font, **kwargs)
        with self.lock:
            rows=[dict(row) for row in self.five_actions.values()]
            diagnostics=dict(self.five_status)
            active=list(self.five_active.values())
        titles={'air-before':'入水前转场','native-roundtrip':'入水→水下航行→出水',
                'air-after':'返回空中目标','aav_2-air':'空中转场','aav_3-air':'空中转场',
                'usv-pass':'水面航行与减速','uuv-pass':'水下通过与减速',
                'forbidden-during-native':'交接互斥检查','reject-rock':'岩石入口检查',
                'reject-exclusion':'禁入区入口检查'}
        text(0,1,'五平台联合运动验证',fontsize=19,weight='bold',va='top')
        text(0,.92,'空中转场 · 跨介质往返 · 水面与水下航行',fontsize=12,va='top')
        text(0,.86,'动作状态来自执行端；此窗口不判定业务任务完成。',fontsize=10,color='#626a73',va='top')
        for i,name in enumerate(['无人机1','无人机2','无人机3','无人船','潜航器']):
            actions=sorted([r for r in rows if r['member']==i],key=lambda r:r['at'])
            # Entry rejection checks never replace the actual accepted work row.
            normal=[r for r in actions if r['task'] in titles and r['task'] not in
                    ('forbidden-during-native','reject-rock','reject-exclusion')]
            current=normal[-1] if normal else None
            y=.75-i*.125
            text(0,y,name,fontsize=13,weight='bold',va='top')
            action_text=(titles.get(current['task'],'原生动作')+'  ·  '+current['state']) if current else '尚未收到动作'
            if any(member==i and busy and time.monotonic()-stamp<1. for member,busy,stamp in active) and (not current or current.get('result')):
                action_text='原生动作执行中（名称待同步）'
            values,stamp=diagnostics.get(i,({},0.))
            if not stamp or time.monotonic()-stamp>1.:
                action_text+='  / 状态更新中断'
            if values.get('resource_locked',values.get('platform_resource_locked','')).lower()=='true':
                action_text+='  / 资源锁定'
            text(0,y-.047,action_text,fontsize=11,va='top')
        text(0,.08,'场景：码头、岩石、禁入区；曲线为实际运动轨迹。',fontsize=10,color='#626a73')
        text(0,.035,'观测点与母船为预设位置；监测交付、中继、复查尚未接通。',fontsize=10,color='#626a73')
        self.figure.tight_layout()
        return self.render()

    def draw_water_cooperation(self, now):
        """Show the selected AIR/WATER activities from the one task authority."""
        state=json.loads(rospy.get_param('/formation_mission_runner/task_state','{}'))
        with self.lock:
            progress=dict(self.delivery_progress)
        axis=self.task_axis
        axis.clear();axis.axis('off')
        def line(y,content,size=12,colour='#263238'):
            axis.text(0,y,content,fontproperties=self.five_font,fontsize=size,
                      color=colour,va='top',wrap=True)
        def member_name(executor):
            parts=executor.split('_')
            if len(parts)>1 and parts[0]=='aav' and parts[1].isdigit():
                return '无人机'+parts[1]
            return {'uuv':'潜航器','usv':'无人船'}.get(executor,executor)
        rows=(state.get('plan') or {}).get('items',[])
        air=any(r['executor_id'].startswith('aav_') for r in rows)
        cross=any(r['executor_id'].startswith('aav_') and
                  any(segment.get('operation') in ('ENTER_WATER','EXIT_WATER')
                      for step in r.get('execution_steps',())
                      for segment in (step.get('native_action') or {}).get('segments',())) for r in rows)
        water=any(r['executor_id']=='uuv' for r in rows) or cross
        names={'PLANNING':'正在比较候选计划','PLANNING_DIAGNOSTIC':'隔离诊断：正在比较候选',
               'AWAITING_CONFIRMATION':'等待终端确认，尚未派发',
               'RUNNING':'协作执行中','RUNNING_DIAGNOSTIC':'同请求诊断执行中',
               'PASS_JOINT_NO_RETURN_DIAGNOSTIC':'空中／水下交付完成（无返回诊断）',
               'PASS_AAV_CROSS_MEDIUM_DIAGNOSTIC':'跨介质观测与返回完成（诊断）',
               'PASS_GEOMETRIC_PROXY_QUALIFICATION':'规定作业、收件与返回完成（几何代理）',
               'PASS_WATER_GEOMETRIC_PROXY':'水下阶段完成（几何观测代理）',
               'PASS_AIR_SUPPORT_COMPONENT':'空中与无人船组件完成（几何代理）',
               'FAILED':'失败，请查看原因与资源锁定',
               'FAIL':'请求未完成，请查看原因',
               'UNKNOWN_LOCKED':'结果未知，成员保持锁定',
               'NOT_CONFIRMED':'未确认，未派发'}
        title=('近岸联合观测' if not rows else '近岸空中＋水下联合观测' if air and water else
               '空中观测＋无人船射频支援' if air else '水下观测＋无人船支援')
        line(1.,title,18)
        status=state.get('status','STANDBY')
        line(.93,names.get(status,'等待任务程序'),14)
        age=now-state.get('updated_at_ros_s',now)
        line(.875,'任务记录年龄 {:.1f}s · 当前计划 {} 项活动'.format(age,len(rows)),10)
        statuses={'PLANNED':'等待派发','RUNNING':'执行中','COMPLETED':'动作完成',
                  'UNKNOWN_LOCKED':'未知，保持锁定','FAILED':'失败'}
        phases={'PREPARING':'正在预装载','PREPARED':'等待共同启动','AIR_MOVE':'空中转场',
                'ENTER_WATER':'入水中','EXIT_WATER':'出水中',
                'MOVING':'空中转场','HOLDING':'稳定确认',
                'WATER_PATH':'水下航行／观测','SURFACE_PATH':'驶往支援区',
                'PRECOMMITTED_WAIT':'等待预定会合',
                'COAST_STOP':'滑行终端','TRIM_PROPULSION':'配平保持',
                'UNKNOWN_LOCKED':'未知，保持锁定'}
        for index,row in enumerate(rows[:6]):
            executor=row['executor_id']
            member=member_name(executor)
            role=('通信支援' if not row['fulfills_task'] else
                  '跨介质观测' if cross and executor.endswith('_native') else
                  '空中观测' if executor.startswith('aav_') else '水下观测' if executor=='uuv' else '作业')
            phase=(state.get('current_actions',{}).get(row['execution_id']) or {}).get('phase','')
            label=phases.get(phase,statuses.get(row['status'],row['status']))
            steps=row.get('execution_steps') or ()
            native=steps[0].get('native_action') if steps else None
            wait=0. if native is None else native.get('terminal_wait_s',0.)
            detail=(' · 交付等待{:.1f}s'.format(wait) if wait else '')
            line(.81-index*.06,'{} · {}  {}  {:.0f}–{:.0f}s{}'.format(
                member,role,label,row['planned_start'],row['planned_finish'],detail),11)
        if len(rows)>6:line(.45,'另有 {} 项活动；完整记录见任务结果'.format(len(rows)-6),10)
        if not rows:line(.79,'正在规划；确认前不派发',12)
        locks=state.get('resource_locks',[])
        line(.44,'占用／锁定：'+('、'.join(member_name(k) for k in locks) or '无'),12)
        commands=state.get('command_progress',{})
        command_status=(' · 指令已达 {}/{}'.format(commands['delivered'],commands['requested'])
                        if commands.get('requested') else '')
        line(.38,'母船结果 {:.0%} · 业务 {} 项{}'.format(
            state.get('delivered_fraction',0.),len(state.get('results_received',[])),command_status),12)
        line(.32,'传输过程（独立仿真视图，不作为任务完成判定）',11)
        if not progress or now-progress.get('at_ros_s',0.)>2.:
            detail='传输状态缺失／过期，不能推断接收进度'
        elif not progress.get('products'):
            detail='等待平台产生观测摘要'
        else:
            products=progress['products']
            detail='{} 份摘要 · 无人船已收 {:.1f} KiB · 母船已收 {:.1f} KiB'.format(
                len(products),sum(p['relay_bytes'] for p in products)/1024.,
                sum(p['mother_bytes'] for p in products)/1024.)
        line(.27,detail,12)
        line(.21,'声明链路：水下 8 m / 2 KiB/s；射频 30 m / 32 KiB/s',10)
        reason=state.get('failure_reason','')
        if reason:
            if reason=='dependent work precedes its support launch':
                reason='协同作业启动早于支援启动，后续派发已阻断'
            timing=re.fullmatch(
                r'endpoint not ready: (\S+): time baseline not qualified: model/ROS drift ([0-9.]+) s exceeds ([0-9.]+) s',
                reason)
            if timing:
                reason='空中执行端 {} 未就绪：模型/ROS时间偏差 {} 秒超过 {} 秒'.format(*timing.groups())
            line(.16,'失败：'+textwrap.fill(reason,62),10,'#b71c1c')
        elif state.get('return_completion'):
            parts=[]
            for member,row in state['return_completion'].items():
                name={'uuv':'潜航器','usv':'无人船'}.get(member,member_name(member))
                method='通过式' if row.get('evidence')=='NATIVE_REENTRY_AND_COAST_RESULT' else '回区'
                parts.append(name+method+('完成' if row.get('completed') else '未证实'))
            line(.16,'返回：'+'、'.join(parts),10)
        else:
            line(.16,'实际收件与 Action 结果分别判断。',10)
        if status=='PASS_GEOMETRIC_PROXY_QUALIFICATION':
            conclusion='规定作业、实际收件与参与成员返回已完成；复查以任务记录为准。'
        elif status in ('FAILED','FAIL','UNKNOWN_LOCKED'):
            conclusion='本请求未完成；请查看失败原因与成员占用。'
        else:
            conclusion='当前为声明初态资格仿真；实际Action与收件决定任务结果。'
        line(.09,conclusion,10)
        line(.045,'关闭窗口不停止物理执行；几何代理不等于真实载荷质量。',10)
        self.figure.tight_layout()
        return self.render()

    def draw_task_authority(self, now):
        raw = rospy.get_param("/formation_mission_runner/task_state", "{}")
        state = json.loads(raw)
        axis = self.task_axis
        axis.clear()
        axis.axis("off")
        age = now - state.get("updated_at_ros_s", now)
        action = state.get("current_action") or {}
        disposition = state.get("safety_disposition") or {}
        lines = ["TASK AUTHORITY — " + state.get("request_id", "waiting for runner"),
                 "Status: {}   update age: {:.1f}s".format(state.get("status", "UNAVAILABLE"), age),
                 "Current action: {} / {}".format(action.get("task_id", "none"), action.get("phase", "-")),
                 "Selected endpoint: " + action.get("endpoint", "-"),
                 "Safety disposition: {} / {} (original task remains unsuccessful)".format(
                     disposition.get("reason", "none"), disposition.get("outcome", "observing")
                     ) if disposition else "Safety disposition: none",
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
        wrapped = "\n".join(textwrap.fill(line, width=115, subsequent_indent="  ") for line in lines)
        axis.text(.01, .99, wrapped, transform=axis.transAxes, va="top",
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
