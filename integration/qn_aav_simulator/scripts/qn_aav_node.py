#!/usr/bin/env python3
"""Execute Swarm-Formation PositionCommand through the original qn closed loop.

Two odometry outputs are derived from one qn state snapshot:

* ``~odometry`` is a standard ROS ``nav_msgs/Odometry``: world position and
  attitude with **body-frame** linear and angular velocity.
* ``~odometry_swarm_compat`` is a Swarm compatibility input.  It preserves the
  world-frame linear velocity that the Swarm-Formation planners actually
  consume and is not a general-purpose ROS Odometry.

The control loop freezes one immutable command snapshot before every
``backend.step()`` and publishes the reference it actually used together with
model-time bookkeeping on ``~used_reference_pose``, ``~used_reference_twist``
and ``~diagnostics``.  No qn controller, actuator or 6DOF equation is changed.
"""

import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import rospy
from std_msgs.msg import Float64

from qn_aav_simulator.contracts import (
    AgentState,
    CommandMode,
    ControlCmd,
    PlantStepInput,
    PlatformAdapterCmd,
)
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
from qn_aav_simulator.qn_telemetry import (
    CommandSnapshot,
    ModelClock,
    ReferenceUsageTracker,
    vector3,
)


class QnAavNode:
    def __init__(self):
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        self.agent_id = "drone_{}".format(self.drone_id)
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        if not self.rate_hz > 0.0:
            raise ValueError("~rate must be positive")
        self.dt = 1.0 / self.rate_hz
        # The outer step is driven by the observed ROS elapsed time so that the
        # accumulated *model* time tracks planner time (plan.md P0.4).  It is
        # quantised to whole qn integration sub-steps.
        self.max_outer_dt = float(rospy.get_param("~max_outer_dt", 0.1))
        self.min_outer_dt = float(rospy.get_param("~min_outer_dt", 0.001))
        self.clamped_outer_steps = 0
        self.max_observed_outer_dt_s = 0.0
        self.outer_dt_residual_s = 0.0
        self.last_step_ros_time_s = None
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 2.0))
        self.max_acc_mps2 = float(rospy.get_param("~max_acc_mps2", 8.0))
        position = (
            float(rospy.get_param("~init_x", 0.0)),
            float(rospy.get_param("~init_y", 0.0)),
            float(rospy.get_param("~init_z", 0.5)),
        )
        self.lock = threading.Lock()
        self.pending_command = None
        self.latest_command = None
        self.step_index = 0
        self.usage = ReferenceUsageTracker(self.agent_id)
        self.clock = ModelClock()
        self.state = AgentState(
            agent_id=self.agent_id,
            type="AAV",
            timestamp_s=rospy.Time.now().to_sec(),
            position=position,
            velocity=(0.0, 0.0, 0.0),
        )
        self.backend = QnPythonClosedLoopBackend(
            {
                "initialization_mode": "STATIC_TRIM",
                "model_step_s": 0.001,
                "reference_mode": "ROUTE_POSITION",
                "water_guidance_mode": "QN_ORIGINAL_POSITION",
                "water_horizontal_controller_mode": "QN_ORIGINAL_RBF_PD",
            }
        )
        self.backend.reset(self.state)
        self.usage.reset(position)

        self.odom_pub = rospy.Publisher("~odometry", Odometry, queue_size=1)
        self.compat_pub = rospy.Publisher(
            "~odometry_swarm_compat", Odometry, queue_size=1)
        self.used_pose_pub = rospy.Publisher(
            "~used_reference_pose", PoseStamped, queue_size=1)
        self.used_twist_pub = rospy.Publisher(
            "~used_reference_twist", TwistStamped, queue_size=1)
        self.diagnostics_pub = rospy.Publisher(
            "~diagnostics", DiagnosticArray, queue_size=1)
        self.medium_pub = rospy.Publisher("~medium_flag", Float64, queue_size=1)
        self.command_sub = rospy.Subscriber(
            "~command", PositionCommand, self.command_callback, queue_size=1
        )
        rospy.loginfo(
            "QN AAV %s ready: original RBF/PD + actuators + 6DOF "
            "(standard odometry + Swarm compatibility input separated)",
            self.agent_id,
        )

    # -- command capture ---------------------------------------------------
    def command_callback(self, message):
        snapshot_fields = {
            "stamp_s": message.header.stamp.to_sec(),
            "trajectory_id": int(message.trajectory_id),
            "trajectory_flag": int(message.trajectory_flag),
            "position": vector3((message.position.x, message.position.y,
                                 message.position.z)),
            "velocity": vector3((message.velocity.x, message.velocity.y,
                                 message.velocity.z)),
            "acceleration": vector3((message.acceleration.x, message.acceleration.y,
                                     message.acceleration.z)),
            "yaw_rad": float(message.yaw),
            "received_ros_time_s": rospy.Time.now().to_sec(),
        }
        with self.lock:
            self.pending_command = snapshot_fields
            self.latest_command = snapshot_fields

    def _freeze_snapshot(self):
        """Copy the pending command into an immutable per-step snapshot."""
        with self.lock:
            pending = self.pending_command
            self.pending_command = None
        if pending is None:
            return None
        return CommandSnapshot(
            agent_id=self.agent_id,
            outer_step_index=self.usage.used_outer_step_count,
            received_ros_time_s=pending["received_ros_time_s"],
            stamp_s=pending["stamp_s"],
            trajectory_id=pending["trajectory_id"],
            trajectory_flag=pending["trajectory_flag"],
            position=pending["position"],
            velocity=pending["velocity"],
            acceleration=pending["acceleration"],
            yaw_rad=pending["yaw_rad"],
        )

    # -- control loop ------------------------------------------------------
    def _hold_reference(self):
        """A synthetic hold-at-current-position before the first command.

        The standard Odometry, the diagnostics and the model clock must exist
        before the first task is dispatched, otherwise the readiness check and
        the model-time baseline could never start.  This does not create a new
        control layer: it is the same ``ROUTE_POSITION`` reference the qn
        backend already tracks.
        """
        state = self.state
        return {
            "stamp_s": rospy.Time.now().to_sec(),
            "trajectory_id": 0,
            "trajectory_flag": 0,
            "position": tuple(float(value) for value in state.position),
            "velocity": (0.0, 0.0, 0.0),
            "acceleration": (0.0, 0.0, 0.0),
            "yaw_rad": 0.0,
            "received_ros_time_s": rospy.Time.now().to_sec(),
        }

    def _outer_dt(self, elapsed_s):
        """Quantise the observed ROS elapsed time to integration sub-steps.

        The qn backend integrates whole ``model_step_s`` sub-steps, so the outer
        step is a multiple of that sub-step.  The leftover below one sub-step is
        carried into the next step instead of being dropped: dropping it turns
        the model/ROS clock difference into an unbiased random walk (~27 ms per
        30 s at 100 Hz, 1 ms sub-steps), which is the same order as the P0.4
        drift gate.  A *clamped* step is different: a real stall must show up as
        accumulated model/ROS drift, so its excess is not carried.
        """
        substep = float(self.backend.model_step_s)
        stalled = float(elapsed_s) > self.max_outer_dt
        target = float(elapsed_s) + self.outer_dt_residual_s
        if target > self.max_outer_dt:
            target = self.max_outer_dt
        if target < self.min_outer_dt:
            target = self.min_outer_dt
        substeps = max(1, int(round(target / substep)))
        dt = substeps * substep
        self.outer_dt_residual_s = 0.0 if stalled else (target - dt)
        if stalled:
            self.clamped_outer_steps += 1
        self.max_observed_outer_dt_s = max(self.max_observed_outer_dt_s, dt)
        return dt

    def step(self, dt_s=None):
        dt_s = self.dt if dt_s is None else float(dt_s)
        now = rospy.Time.now()
        snapshot = self._freeze_snapshot()
        if snapshot is None:
            with self.lock:
                latest = self.latest_command
            if latest is None:
                latest = self._hold_reference()
            # No new command arrived in this outer tick: the frozen reference is
            # the previous command position, re-stamped locally.
            snapshot = CommandSnapshot(
                agent_id=self.agent_id,
                outer_step_index=self.usage.used_outer_step_count,
                received_ros_time_s=now.to_sec(),
                stamp_s=latest["stamp_s"],
                trajectory_id=latest["trajectory_id"],
                trajectory_flag=latest["trajectory_flag"],
                position=latest["position"],
                velocity=latest["velocity"],
                acceleration=latest["acceleration"],
                yaw_rad=latest["yaw_rad"],
            )
        usage = self.usage.record(snapshot, dt_s, self.clock.model_time_s)
        command = ControlCmd(
            cmd_id="swarm-formation:{}:{}".format(self.agent_id, self.step_index),
            agent_id=self.agent_id,
            timestamp_s=usage.model_interval_start_s,
            command_mode=CommandMode.DESIRED_POSITION,
            desired_position=usage.position,
            desired_velocity=usage.velocity,
            desired_acceleration=snapshot.acceleration,
            desired_yaw_rad=snapshot.yaw_rad,
        )
        result = self.backend.step(
            PlantStepInput(
                state=self.state,
                control_cmd=command,
                platform_adapter=PlatformAdapterCmd(self.agent_id),
                dt_s=dt_s,
                max_speed_mps=self.max_speed_mps,
                max_acc_mps2=self.max_acc_mps2,
            )
        )
        self.step_index += 1
        substeps = int(result.diagnostics.get("substeps", 0))
        integration_step_s = float(
            result.diagnostics.get("model_step_s", self.backend.model_step_s))
        model_time_s = self.clock.advance(dt_s, integration_step_s, substeps)
        self.state = AgentState(
            agent_id=self.agent_id,
            type="AAV",
            timestamp_s=now.to_sec(),
            position=result.position,
            velocity=result.velocity,
            acceleration=result.acceleration,
            orientation_quat_wxyz=result.orientation_quat_wxyz,
            body_linear_velocity_mps=result.body_linear_velocity_mps,
            body_angular_velocity_radps=result.body_angular_velocity_radps,
            medium_flag=result.medium_flag,
        )
        self.publish(now, usage, result, model_time_s, integration_step_s)

    # -- publication -------------------------------------------------------
    def _fill_pose(self, message, stamp):
        state = self.state
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.pose.pose.position.x = state.position[0]
        message.pose.pose.position.y = state.position[1]
        message.pose.pose.position.z = state.position[2]
        w, x, y, z = state.orientation_quat_wxyz
        message.pose.pose.orientation.w = w
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z

    def publish(self, stamp, usage, result, model_time_s, integration_step_s):
        state = self.state

        standard = Odometry()
        self._fill_pose(standard, stamp)
        standard.child_frame_id = self.agent_id + "/base_link"
        standard.twist.twist.linear.x = state.body_linear_velocity_mps[0]
        standard.twist.twist.linear.y = state.body_linear_velocity_mps[1]
        standard.twist.twist.linear.z = state.body_linear_velocity_mps[2]
        standard.twist.twist.angular.x = state.body_angular_velocity_radps[0]
        standard.twist.twist.angular.y = state.body_angular_velocity_radps[1]
        standard.twist.twist.angular.z = state.body_angular_velocity_radps[2]
        self.odom_pub.publish(standard)

        compat = Odometry()
        self._fill_pose(compat, stamp)
        compat.child_frame_id = self.agent_id + "/swarm_compat"
        compat.twist.twist.linear.x = state.velocity[0]
        compat.twist.twist.linear.y = state.velocity[1]
        compat.twist.twist.linear.z = state.velocity[2]
        compat.twist.twist.angular = standard.twist.twist.angular
        self.compat_pub.publish(compat)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = usage.position
        pose.pose.orientation.w = 1.0
        self.used_pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "world"
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = usage.velocity
        self.used_twist_pub.publish(twist)

        self.diagnostics_pub.publish(self._diagnostics(
            stamp, usage, result, model_time_s, integration_step_s))
        self.medium_pub.publish(Float64(data=float(state.medium_flag or 0.0)))

    def _diagnostics(self, stamp, usage, result, model_time_s, integration_step_s):
        state = self.state
        medium_active = float(state.medium_flag or 0.0) > 0.0
        entries = [
            ("agent_id", self.agent_id),
            ("source_trajectory_id", str(usage.source_trajectory_id)),
            ("source_command_stamp", repr(usage.source_command_stamp)),
            ("used_outer_step", str(usage.used_outer_step)),
            ("outer_dt_s", repr(usage.outer_dt_s)),
            ("nominal_outer_dt_s", repr(self.dt)),
            ("max_observed_outer_dt_s", repr(self.max_observed_outer_dt_s)),
            ("clamped_outer_steps", str(self.clamped_outer_steps)),
            ("outer_dt_residual_s", repr(self.outer_dt_residual_s)),
            ("integration_step_s", repr(integration_step_s)),
            ("outer_step_count", str(self.clock.outer_step_count)),
            ("integration_step_count", str(self.clock.integration_step_count)),
            ("model_time_s", repr(model_time_s)),
            ("model_interval_start", repr(usage.model_interval_start_s)),
            ("model_interval_end", repr(usage.model_interval_end_s)),
            # PositionCommand.velocity/acceleration are not fed to the qn inner
            # loop in ROUTE_POSITION mode; the used reference velocity is always
            # derived from consecutive used positions.
            ("velocity_directly_consumed", "false"),
            ("acceleration_directly_consumed", "false"),
            ("derived_reference_velocity_role",
             "WATER_PATH_HEADING" if medium_active else "AIR_GATED_OFF"),
            ("used_reference_valid", "true" if usage.velocity_was_derived else "false"),
            ("reference_position_x", repr(usage.position[0])),
            ("reference_position_y", repr(usage.position[1])),
            ("reference_position_z", repr(usage.position[2])),
            ("used_reference_velocity_x", repr(usage.velocity[0])),
            ("used_reference_velocity_y", repr(usage.velocity[1])),
            ("used_reference_velocity_z", repr(usage.velocity[2])),
            ("position_x", repr(state.position[0])),
            ("position_y", repr(state.position[1])),
            ("position_z", repr(state.position[2])),
            ("world_velocity_x", repr(state.velocity[0])),
            ("world_velocity_y", repr(state.velocity[1])),
            ("world_velocity_z", repr(state.velocity[2])),
            ("body_velocity_x", repr(state.body_linear_velocity_mps[0])),
            ("body_velocity_y", repr(state.body_linear_velocity_mps[1])),
            ("body_velocity_z", repr(state.body_linear_velocity_mps[2])),
            ("medium_flag", repr(float(state.medium_flag or 0.0))),
        ]
        status = DiagnosticStatus()
        status.name = "{}/qn_reference".format(self.agent_id)
        status.hardware_id = self.agent_id
        status.level = DiagnosticStatus.OK
        status.message = "reference in use"
        status.values = [KeyValue(key=key, value=value) for key, value in entries]
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [status]
        return array

    def run(self):
        rate = rospy.Rate(round(self.rate_hz))
        self.last_step_ros_time_s = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            dt_s = self._outer_dt(now_s - self.last_step_ros_time_s)
            self.last_step_ros_time_s = now_s
            self.step(dt_s)
            rate.sleep()


def main():
    rospy.init_node("qn_aav_simulator")
    QnAavNode().run()


if __name__ == "__main__":
    main()
