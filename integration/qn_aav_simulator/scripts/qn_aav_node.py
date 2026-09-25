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

import math
import hashlib
import json
import os
import threading
import time

# Opt-in diagnostic: the existing private query build compiles these exact qn
# sources and checks their source hashes/ABI before import. Keep the ordinary
# online path unchanged unless an experiment explicitly selects it.
if os.environ.get('QN_SAME_SOURCE_ACCELERATION')=='true':
    from mrta_python.query_worker import _enable_query_extensions
    if not _enable_query_extensions():
        raise RuntimeError('same-source qn accelerator is absent or differs from mounted source')

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger,TriggerResponse

from qn_aav_simulator.contracts import (
    AgentState,
    CommandMode,
    ControlCmd,
    PlantStepInput,
    PlatformAdapterCmd,
)
from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
from qn_aav_simulator.odometry import ros_odometry_fields
from qn_aav_simulator.platform_execution import DomainHistory,actual_mode
from qn_aav_simulator.qn_telemetry import (
    CommandAdoptionBuffer,
    CommandSnapshot,
    ModelClock,
    ReferenceUsageTracker,
    vector3,
)


class QnAavNode:
    def __init__(self):
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        self.agent_id = str(rospy.get_param("~agent_id", "drone_{}".format(self.drone_id)))
        self.platform_type = "UUV" if self.agent_id == "uuv" else "AAV"
        if self.platform_type == "UUV" and float(rospy.get_param("~init_z", 0.5)) >= 0.0:
            raise ValueError("the qn UUV proxy must start submerged")
        self.world_frame = str(rospy.get_param("~world_frame", "world"))
        if not self.world_frame:
            raise ValueError("world_frame must be declared")
        self.domain_history = DomainHistory()
        # One authoritative outer-step configuration.  The control rate is
        # derived from it, so no second rate parameter can disagree with it.
        self.outer_dt_s = float(rospy.get_param("~outer_dt_s", 0.01))
        if not self.outer_dt_s > 0.0:
            raise ValueError("~outer_dt_s must be positive")
        self.rate_hz = 1.0 / self.outer_dt_s
        # The model clock is driven by completed outer steps only.  A stalled
        # loop therefore shows up as model/ROS lag instead of being hidden by
        # replaying history, and a later command is never applied to an earlier
        # model interval.
        self.command_buffer_size = int(rospy.get_param("~command_buffer_size", 32))
        if self.command_buffer_size < 1:
            raise ValueError("~command_buffer_size must be positive")
        self.commands = CommandAdoptionBuffer(self.command_buffer_size)
        self.min_height_m = float("inf")
        self.max_medium_flag = 0.0
        self.air_domain_violation = False
        self.max_loop_ros_gap_s = 0.0
        self.last_step_ros_time_s = None
        self.worst_step_timing = None
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 2.0))
        self.max_acc_mps2 = float(rospy.get_param("~max_acc_mps2", 8.0))
        position = (
            float(rospy.get_param("~init_x", 0.0)),
            float(rospy.get_param("~init_y", 0.0)),
            float(rospy.get_param("~init_z", 0.5)),
        )
        self.lock = threading.RLock()
        self.platform_action = None
        self.latest_command = None
        self.step_index = 0
        self.usage = ReferenceUsageTracker(self.agent_id)
        self.clock = ModelClock()
        self.state = AgentState(
            agent_id=self.agent_id,
            type=self.platform_type,
            timestamp_s=rospy.Time.now().to_sec(),
            position=position,
            velocity=(0.0, 0.0, 0.0),
        )
        self.backend = QnPythonClosedLoopBackend(
            {
                "initialization_mode": "STATIC_TRIM",
                "model_step_s": 0.001,
                "reference_mode": "ROUTE_POSITION",
                "water_guidance_mode": rospy.get_param(
                    "~water_guidance_mode", "QN_ORIGINAL_POSITION"),
                "water_horizontal_controller_mode": rospy.get_param(
                    "~water_horizontal_controller_mode", "QN_ORIGINAL_RBF_PD"),
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
        self.state_digest_service=rospy.Service('~state_digest',Trigger,self.state_digest)
        if rospy.get_param("~enable_platform_action", False):
            from qn_aav_simulator.platform_action import LocalPlatformAction
            self.platform_action = LocalPlatformAction(self)
        self.command_sub = rospy.Subscriber(
            "~command", PositionCommand, self.command_callback, queue_size=1
        )
        rospy.loginfo(
            "QN AAV %s ready: original RBF/PD + actuators + 6DOF "
            "(standard odometry + Swarm compatibility input separated)",
            self.agent_id,
        )

    def state_digest(self,_request):
        """Return one local full-state claim, never a robot motion command."""
        try:
            with self.lock:
                report=dict(agent_id=self.agent_id,model_time_s=self.clock.model_time_s,
                    outer_step=self.step_index,ros_stamp_s=rospy.Time.now().to_sec(),
                    digest=hashlib.sha256(self.backend.execution_state_bytes()).hexdigest())
            return TriggerResponse(True,json.dumps(report,allow_nan=False))
        except (TypeError,ValueError) as error:
            return TriggerResponse(False,str(error))

    # -- command capture ---------------------------------------------------
    def command_callback(self, message):
        with self.lock:
            self._command_callback_locked(message)

    def _command_callback_locked(self, message):
        if self.platform_action is not None and not self.platform_action.accepts_air(int(message.trajectory_id)):
            return
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
            "reference_source": "AIR_SWARM",
            "reference_generation": self.platform_action.owner.generation if self.platform_action else 0,
        }
        self.commands.note(snapshot_fields)
        with self.lock:
            self.latest_command = snapshot_fields

    def _freeze_snapshot(self):
        """Copy the command adopted for this step into an immutable snapshot.

        The adopted command is the newest one that had already arrived when the
        step began; it is then held for the whole step.  Nothing is adopted from
        the future and no past interval is recomputed.
        """
        adopted = self.commands.adopt()
        if adopted is None:
            return None
        return CommandSnapshot(
            agent_id=self.agent_id,
            outer_step_index=self.usage.used_outer_step_count,
            received_ros_time_s=adopted["received_ros_time_s"],
            stamp_s=adopted["stamp_s"],
            trajectory_id=adopted["trajectory_id"],
            trajectory_flag=adopted["trajectory_flag"],
            position=adopted["position"],
            velocity=adopted["velocity"],
            acceleration=adopted["acceleration"],
            yaw_rad=adopted["yaw_rad"],
            reference_source=adopted.get("reference_source", "AIR_SWARM"),
            reference_generation=adopted.get("reference_generation", 0),
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

    def step(self):
        # Ownership change, snapshot adoption and model integration share one
        # local boundary. No in-flight normal snapshot crosses a handover.
        entered = time.monotonic()
        if self.platform_action is None:
            with self.lock:self._step_locked(entered)
        else:
            # actionlib callbacks already own server.lock before taking the
            # model lock. Terminal feedback/Result in tick uses the same order.
            with self.platform_action.server.lock,self.lock:self._step_locked(entered)

    def _step_locked(self, entered):
        acquired = time.monotonic()
        dt_s = self.outer_dt_s
        now = rospy.Time.now()
        if self.platform_action is not None:
            native = self.platform_action.tick()
            if native is not None:
                self.commands.note(native)
                self.latest_command = native
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
                reference_source=latest.get("reference_source", "INITIAL_HOLD"),
                reference_generation=latest.get("reference_generation", 0),
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
            frame_id=self.world_frame,
        )
        prepared = time.monotonic()
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
        integrated = time.monotonic()
        self.step_index += 1
        substeps = int(result.diagnostics.get("substeps", 0))
        integration_step_s = float(
            result.diagnostics.get("model_step_s", self.backend.model_step_s))
        model_time_s = self.clock.advance(dt_s, integration_step_s, substeps)
        self.state = AgentState(
            agent_id=self.agent_id,
            type=self.platform_type,
            timestamp_s=now.to_sec(),
            position=result.position,
            velocity=result.velocity,
            acceleration=result.acceleration,
            orientation_quat_wxyz=result.orientation_quat_wxyz,
            body_linear_velocity_mps=result.body_linear_velocity_mps,
            body_angular_velocity_radps=result.body_angular_velocity_radps,
            medium_flag=result.medium_flag,
        )
        self._latch_air_domain()
        if self.platform_action is not None:
            self.platform_action.note_adopted(snapshot)
        committed = time.monotonic()
        self.publish(now, usage, result, model_time_s, integration_step_s)
        published = time.monotonic()
        elapsed = published-entered
        if self.worst_step_timing is None or elapsed>self.worst_step_timing[0]:
            # Passive durations for the same worst cycle. No timestamps, model
            # steps, control/reference ownership or acceptance gates change.
            self.worst_step_timing=(elapsed,now.to_sec(),acquired-entered,
                prepared-acquired,integrated-prepared,committed-integrated,published-committed)

    def _latch_air_domain(self):
        """Latch the observed AIR-domain violation instead of only averaging it.

        The model switches its mass, inertia, damping and actuation by
        ``medium_flag``, so any non-zero flag means the run left the AIR model.
        The latch is cheap, stays online, and the exact duration and minimum are
        computed offline from the recorded diagnostics.
        """
        height = float(self.state.position[2])
        flag = float(self.state.medium_flag or 0.0)
        allowed = (self.platform_action.allowed_modes() if self.platform_action
                   else frozenset({'AIR'}))
        self.domain_history.observe(height, flag, allowed, self.air_floor_m)
        self.min_height_m = self.domain_history.min_height
        self.max_medium_flag = self.domain_history.max_medium
        self.air_domain_violation = self.domain_history.air_violation

    @property
    def air_floor_m(self):
        """The AIR boundary of the qn model: ``hg_m / 2``, derived not stored."""
        return 0.5 * float(self.backend.constants.hg_m)

    # -- publication -------------------------------------------------------
    def _fill_pose(self, message, stamp, fields):
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        message.pose.pose.position.x = fields.position[0]
        message.pose.pose.position.y = fields.position[1]
        message.pose.pose.position.z = fields.position[2]
        w, x, y, z = fields.orientation_quat_wxyz
        message.pose.pose.orientation.w = w
        message.pose.pose.orientation.x = x
        message.pose.pose.orientation.y = y
        message.pose.pose.orientation.z = z

    def publish(self, stamp, usage, result, model_time_s, integration_step_s):
        state = self.state

        # One state snapshot, one coordinate mapping, two topics.  The mapping
        # lives in qn_aav_simulator.odometry so no consumer repeats it.
        fields = ros_odometry_fields(
            position=state.position,
            quaternion=state.orientation_quat_wxyz,
            world_velocity=state.velocity,
            body_angular_velocity=state.body_angular_velocity_radps,
        )

        standard = Odometry()
        self._fill_pose(standard, stamp, fields)
        standard.child_frame_id = self.agent_id + "/base_link"
        standard.twist.twist.linear.x = fields.linear_body_velocity[0]
        standard.twist.twist.linear.y = fields.linear_body_velocity[1]
        standard.twist.twist.linear.z = fields.linear_body_velocity[2]
        standard.twist.twist.angular.x = fields.angular_body_velocity[0]
        standard.twist.twist.angular.y = fields.angular_body_velocity[1]
        standard.twist.twist.angular.z = fields.angular_body_velocity[2]
        self.odom_pub.publish(standard)

        compat = Odometry()
        self._fill_pose(compat, stamp, fields)
        compat.child_frame_id = self.agent_id + "/swarm_compat"
        compat.twist.twist.linear.x = fields.world_velocity[0]
        compat.twist.twist.linear.y = fields.world_velocity[1]
        compat.twist.twist.linear.z = fields.world_velocity[2]
        compat.twist.twist.angular = standard.twist.twist.angular
        self.compat_pub.publish(compat)

        # Telemetry records the reference the backend actually used, taken from
        # its own result.  It is not a second derivation of the same quantity.
        used = dict(result.diagnostics.get("reference_used") or {})
        reference_position = tuple(
            used.get("position_m") or result.diagnostics.get(
                "reference_position_m") or usage.position)
        reference_velocity = tuple(
            used.get("velocity_mps") or result.diagnostics.get(
                "reference_velocity_mps") or usage.velocity)
        reference_yaw = float(used.get("yaw_rad", 0.0))

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = reference_position
        # The used reference attitude is a yaw-only rotation: the task boundary
        # commands yaw and holds roll/pitch at zero.
        pose.pose.orientation.z = math.sin(0.5 * reference_yaw)
        pose.pose.orientation.w = math.cos(0.5 * reference_yaw)
        self.used_pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.world_frame
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = reference_velocity
        self.used_twist_pub.publish(twist)

        self.diagnostics_pub.publish(self._diagnostics(
            stamp, usage, result, model_time_s, integration_step_s, reference_yaw))
        self.medium_pub.publish(Float64(data=float(state.medium_flag or 0.0)))

    def _diagnostics(self, stamp, usage, result, model_time_s, integration_step_s,
                     reference_yaw):
        state = self.state
        medium_active = float(state.medium_flag or 0.0) > 0.0
        entries = [
            ("agent_id", self.agent_id),
            ("source_trajectory_id", str(usage.source_trajectory_id)),
            ("reference_source", usage.reference_source),
            ("reference_generation", str(usage.reference_generation)),
            ("actual_mode", actual_mode(state.medium_flag)),
            ("source_command_stamp", repr(usage.source_command_stamp)),
            ("used_outer_step", str(usage.used_outer_step)),
            ("outer_dt_s", repr(usage.outer_dt_s)),
            ("fixed_outer_dt_s", repr(self.outer_dt_s)),
            ("max_loop_ros_gap_s", repr(self.max_loop_ros_gap_s)),
            ("command_dropped_count", str(self.commands.dropped_count)),
            ("command_order_violations", str(self.commands.order_violations)),
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
            ("used_reference_yaw_rad", repr(float(reference_yaw))),
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
            ("min_height_m", repr(self.min_height_m)),
            ("max_medium_flag", repr(self.max_medium_flag)),
            ("air_floor_m", repr(self.air_floor_m)),
            ("air_domain_violation", "true" if self.air_domain_violation else "false"),
        ]
        pause=getattr(self,'gc_pause',(0.,0.,-1))
        entries.extend([('max_gc_pause_s',str(pause[0])),('max_gc_pause_started_ros_s',str(pause[1])),
                        ('max_gc_pause_generation',str(pause[2]))])
        timing=self.worst_step_timing
        if timing is not None:
            entries.extend(zip(('worst_step_total_s','worst_step_started_ros_s','worst_step_lock_wait_s',
                'worst_step_preparation_s','worst_step_backend_s','worst_step_commit_s','worst_step_publish_s'),
                map(str,timing)))
        if self.platform_action is not None:
            entries.extend(self.platform_action.diagnostics())
            entries.extend([
                ("domain_policy_version", "1"),
                ("air_scope_min_height_m", repr(self.domain_history.air_min_height)),
                ("air_scope_max_medium_flag", repr(self.domain_history.air_max_medium)),
                ("domain_violation", str(self.domain_history.violation).lower()),
                ("domain_first_violation", self.domain_history.first_violation)])
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
        if rospy.get_param('/use_sim_time',False):
            raise ValueError('qn wall-time execution requires use_sim_time=false; no external clock adapter is configured')
        # Passive timing evidence; collector policy and model steps are unchanged.
        import gc
        self.gc_pause=(0.,0.,-1)
        gc_start=[None]
        def gc_event(phase,info):
            if phase=='start':gc_start[0]=(time.monotonic(),time.time())
            elif gc_start[0] is not None:
                elapsed=time.monotonic()-gc_start[0][0]
                if elapsed>self.gc_pause[0]:self.gc_pause=(elapsed,gc_start[0][1],info['generation'])
        gc.callbacks.append(gc_event)
        rospy.on_shutdown(lambda:gc.callbacks.remove(gc_event) if gc_event in gc.callbacks else None)
        # Absolute wall-time deadlines do not discard elapsed slots after a
        # slow cycle (rospy.Rate resets its phase after >2 periods). Every
        # iteration still integrates exactly one real fixed model step. No
        # completed interval is replayed; adoption uses inputs available now.
        # Gaps/lag remain in diagnostics and the existing validity gate.
        next_tick = time.monotonic()
        self.last_step_ros_time_s = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            # Diagnostic only: how far ROS wall time ran between two fixed
            # model steps.  A real stall shows up here and in the model/ROS
            # alignment gate. No time is credited without backend.step().
            self.max_loop_ros_gap_s = max(
                self.max_loop_ros_gap_s, now_s - self.last_step_ros_time_s)
            self.last_step_ros_time_s = now_s
            self.step()
            next_tick += self.outer_dt_s
            delay = next_tick-time.monotonic()
            if delay>0:
                time.sleep(delay)


def main():
    rospy.init_node("qn_aav_simulator")
    QnAavNode().run()


if __name__ == "__main__":
    main()
