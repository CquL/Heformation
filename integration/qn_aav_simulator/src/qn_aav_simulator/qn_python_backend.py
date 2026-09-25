"""完全由 Python 执行的 qn.slx 闭环控制器与六自由度模型。

公式、通道参数、饱和范围和控制分配均来自 qn.slx 解包后的 Simulink/Stateflow XML。
本模块不调用 MATLAB。原始控制器与工程 LOS_SURGE_YAW 可选分支明确分开；
后者替换水下水平控制，不能视为原 Simulink 闭环逐采样等价。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Tuple

from .contracts import AgentState, Quaternion, Vector3
from .contracts import PlantStepInput, PlantStepResult
from .qn_dynamics import (
    QnEquationState,
    QnModelConstants,
    body_to_map_velocity,
    buoyancy_wrench,
    map_velocity_to_body,
    medium_flag,
    mass_and_inertia,
    normalize_quaternion,
    rigid_body_derivative,
)

Vector6 = Tuple[float, float, float, float, float, float]
Vector7 = Tuple[float, float, float, float, float, float, float]
Vector10 = Tuple[float, float, float, float, float, float, float, float, float, float]


@dataclass(frozen=True)
class QnControllerParameters:
    """一个 qn 掩码控制器通道的原始参数。"""

    k1: float
    k2: float
    w0: float
    input_gain: float
    adaptation_gain: float


@dataclass(frozen=True)
class QnControllerState:
    """RBF 观测器的二阶状态和七个权重积分状态。"""

    observer: tuple[float, float]
    weights: Vector7


@dataclass(frozen=True)
class QnClosedLoopState:
    """qn 连续状态和两个 Memory 块族的离散主步状态。"""

    plant: QnEquationState
    controllers: tuple[QnControllerState, ...]
    controller_fxp_memory: tuple[float, ...]


# 顺序对应 11/control 中 13 个掩码 PD/RBF 子系统。数值直接来自 MaskParameter。
QN_CONTROLLER_NAMES = (
    "AIR_HEIGHT",
    "AIR_PITCH",
    "AIR_ROLL",
    "AIR_X",
    "AIR_Y",
    "AIR_YAW",
    "WATER_Z",
    "WATER_X",
    "WATER_PITCH",
    "WATER_ROLL",
    "WATER_Y",
    "UNCONNECTED_AAV_X",
    "UNCONNECTED_AAV_Y",
)

QN_CONTROLLER_PARAMETERS = (
    QnControllerParameters(6.0, 9.0, 200.0, 5.0, 100000.0),
    QnControllerParameters(25.0, 15.0, 200.0, 5.0, 100000.0),
    QnControllerParameters(25.0, 10.0, 200.0, 12.0, 100000.0),
    QnControllerParameters(5.0, 10.0, 200.0, 50.0, 0.0),
    QnControllerParameters(5.0, 6.0, 200.0, 50.0, 0.0),
    QnControllerParameters(50.0, 20.0, 200.0, 12.0, 100000.0),
    QnControllerParameters(10.0, 20.0, 200.0, 12.0, 100000.0),
    QnControllerParameters(10.0, 50.0, 200.0, 20.0, 100000.0),
    QnControllerParameters(3.0, 3.0, 200.0, 5.0, 100000.0),
    QnControllerParameters(3.0, 3.0, 200.0, 5.0, 100000.0),
    QnControllerParameters(4.0, 8.0, 200.0, 0.5, 0.0),
    # 这两个水下 PD 块存在于 qn，但顶层未连线；其 k1/k2 由 wc^2/2wc 得到。
    QnControllerParameters(8.0**2, 2.0 * 8.0, 200.0, 5.0, 10000.0),
    QnControllerParameters(2.0**2, 2.0 * 2.0, 200.0, 0.5, 10.0),
)

_AIR_CHANNELS = frozenset(range(0, 6))
_WATER_CHANNELS = frozenset(range(6, 13))
_RBF_CENTERS = tuple(
    (first / 3.0, 2.0 * first / 3.0) for first in (-3, -2, -1, 0, 1, 2, 3)
)


class QnPythonClosedLoopBackend:
    """qn.slx 控制、执行器和 plant 的无 MATLAB 闭环后端。"""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.initialization_mode = str(config.get("initialization_mode", "ZERO_STATE")).upper()
        if self.initialization_mode not in {"ZERO_STATE", "STATIC_TRIM"}:
            raise ValueError("qn initialization_mode must be ZERO_STATE or STATIC_TRIM")
        self.model_step_s = float(config.get("model_step_s", 0.001))
        if self.model_step_s <= 0.0:
            raise ValueError("qn model_step_s must be positive")
        reference_mode = str(
            config.get("reference_mode", "SAFE_VELOCITY_STEP")
        ).strip().upper()
        if reference_mode not in {"SAFE_VELOCITY_STEP", "ROUTE_POSITION"}:
            raise ValueError(
                "qn reference_mode must be SAFE_VELOCITY_STEP or ROUTE_POSITION"
            )
        self.reference_mode = reference_mode
        self.max_reference_lead_m = float(config.get("max_reference_lead_m", 2.0))
        if self.max_reference_lead_m <= 0.0:
            raise ValueError("qn max_reference_lead_m must be positive")
        self.water_guidance_mode = str(
            config.get("water_guidance_mode", "QN_ORIGINAL_POSITION")
        ).strip().upper()
        if self.water_guidance_mode not in {
            "QN_ORIGINAL_POSITION",
            "LOS_VELOCITY_REFERENCE",
        }:
            raise ValueError(
                "qn water_guidance_mode must be QN_ORIGINAL_POSITION or "
                "LOS_VELOCITY_REFERENCE"
            )
        self.water_horizontal_controller_mode = str(
            config.get("water_horizontal_controller_mode", "QN_ORIGINAL_RBF_PD")
        ).strip().upper()
        if self.water_horizontal_controller_mode not in {
            "QN_ORIGINAL_RBF_PD",
            "LOS_SURGE_YAW",
        }:
            raise ValueError(
                "qn water_horizontal_controller_mode must be "
                "QN_ORIGINAL_RBF_PD or LOS_SURGE_YAW"
            )
        self.water_surge_reference_gain_s = float(
            config.get("water_surge_reference_gain_s", 3.0)
        )
        self.water_surge_reference_max_m = float(
            config.get("water_surge_reference_max_m", 10.0)
        )
        self.water_steering_reference_scale_m = float(
            config.get("water_steering_reference_scale_m", 0.12)
        )
        self.water_yaw_rate_damping_s = float(
            config.get("water_yaw_rate_damping_s", 0.25)
        )
        self.water_min_turn_surge_fraction = float(
            config.get("water_min_turn_surge_fraction", 0.15)
        )
        self.water_heading_rate_limit_radps = float(
            config.get("water_heading_rate_limit_radps", 1.0)
        )
        if min(
            self.water_surge_reference_gain_s,
            self.water_surge_reference_max_m,
            self.water_steering_reference_scale_m,
            self.water_yaw_rate_damping_s,
            self.water_heading_rate_limit_radps,
        ) < 0.0:
            raise ValueError("qn water guidance parameters must be non-negative")
        if not 0.0 <= self.water_min_turn_surge_fraction <= 1.0:
            raise ValueError("qn water_min_turn_surge_fraction must be in [0, 1]")
        self.constants = QnModelConstants()
        self._agent_id: str | None = None
        self._platform_type: str | None = None
        self._state: QnClosedLoopState | None = None
        self._reference_position_m: Vector3 | None = None
        self._reference_lead_m = 0.0
        self._reference_lead_clamped = False
        self._water_guidance_active = False
        self._water_heading_error_rad = 0.0
        self._water_target_heading_error_rad = 0.0
        self._water_desired_heading_rad = 0.0
        self._water_steering_signal = 0.0
        self._water_surge_alignment = 1.0
        self._water_horizontal_speed_mps = 0.0
        self._last_model_acceleration = None
        self._reference_yaw_rad = None
        self._reference_input_position_m = None
        self._idle_reference = None

    @property
    def backend_id(self) -> str:
        return "PYTHON_QN_CLOSED_LOOP"

    def reset(self, state: AgentState) -> None:
        self._last_model_acceleration = None
        self._reference_yaw_rad = 0.0
        self._reference_input_position_m = None
        self._idle_reference = None
        quaternion = normalize_quaternion(state.orientation_quat_wxyz)
        body_linear = state.body_linear_velocity_mps
        if _norm3(body_linear) <= 1e-12 and _norm3(state.velocity) > 1e-12:
            body_linear = map_velocity_to_body(state.velocity, quaternion)
        plant = QnEquationState(
            body_twist=body_linear + state.body_angular_velocity_radps,
            quaternion_wxyz=quaternion,
            position_xyz_m=state.position,
            actuator_outputs=(0.0,) * 10,
            buoyancy_memory=(0.0,) * 6,
        )
        controllers = tuple(
            QnControllerState(observer=(0.0, 0.0), weights=(0.0,) * 7)
            for _ in QN_CONTROLLER_NAMES
        )
        self._agent_id = state.agent_id
        self._platform_type = state.type
        self._reference_position_m = state.position
        self._reference_lead_m = 0.0
        self._reference_lead_clamped = False
        self._water_guidance_active = False
        self._water_heading_error_rad = 0.0
        self._water_target_heading_error_rad = 0.0
        self._water_desired_heading_rad = _qn_attitude(quaternion)[2]
        self._water_steering_signal = 0.0
        self._water_surge_alignment = 1.0
        self._water_horizontal_speed_mps = 0.0
        self._state = QnClosedLoopState(
            plant=plant,
            controllers=controllers,
            controller_fxp_memory=(0.0,) * len(QN_CONTROLLER_NAMES),
        )
        if self.initialization_mode == "STATIC_TRIM":
            self._state = _static_trim_state(self._state, self.constants)
        # Before a platform Action the node's INITIAL_HOLD takes the actual
        # position on each outer tick. This also applies to a dedicated qn
        # instance initialized submerged; predict_idle still integrates and
        # checks its complete WATER state instead of assuming a fixed pose.
        if (self.reference_mode == "ROUTE_POSITION" and
                medium_flag(state.position[2], self.constants.hg_m) in (0.0, 1.0)):
            self._idle_reference = ("INITIAL_HOLD", None, 0.0, self._state)

    def hold_reference(self) -> None:
        """任务取消/首次保持时锁定实际位置，保留执行器和控制器的连续状态。"""
        if self._state is None:
            raise RuntimeError("qn backend must be reset before hold_reference")
        self._reference_position_m = self._state.plant.position_xyz_m
        self._idle_reference = None

    @property
    def collision_radius_m(self):
        return 0.25  # existing qn/AAV geometric proxy used by the Action server

    def retain_idle_reference(self):
        """Seal the last input only after its native terminal was verified.

        The caller owns terminal/reference semantics. A normal model step alone
        cannot establish that no later command from an active program is due.
        This cold-path evidence is internal to a read-only terminal forecast.
        """
        target = self._reference_input_position_m
        yaw = self._reference_yaw_rad
        if (self._state is None or self.reference_mode != "ROUTE_POSITION" or
                target is None or len(target) != 3 or
                not all(math.isfinite(v) for v in target) or yaw is None or not math.isfinite(yaw)):
            raise ValueError('qn terminal continuation needs the last complete position/yaw input')
        self._idle_reference = ("FIXED_REFERENCE", tuple(target), float(yaw), self._state)

    def predict_idle(self, duration, scene, deadline, dt=.01):
        """Read-only continuation of a witnessed initial or terminal reference.

        All physical, controller, actuator and guidance state is copied. An
        arbitrary mid-program snapshot has no continuation evidence and cannot
        be silently replaced by a position hold.
        """
        import copy
        import time
        from .contracts import ControlCmd, CommandMode, PlatformAdapterCmd
        from .platform_execution import actual_mode
        if not math.isfinite(duration) or duration < 0 or not math.isfinite(deadline):
            raise ValueError('finite idle duration and deadline required')
        if dt != .01 or self.model_step_s != .001:
            return dict(status='UNKNOWN', reason='QN_IDLE_STEP_NOT_QUALIFIED', duration_s=0.)
        if time.monotonic() >= deadline:
            return dict(status='UNKNOWN', reason='QUERY_BUDGET_EXHAUSTED', duration_s=0.)
        evidence = self._idle_reference
        if (evidence is None or evidence[3] is not self._state or
                self.reference_mode != 'ROUTE_POSITION'):
            return dict(status='UNKNOWN', reason='QN_IDLE_REFERENCE_NOT_DETERMINED', duration_s=0.)
        if scene is None:
            return dict(status='UNKNOWN', reason='SCENE_GEOMETRY_NOT_PROVIDED', duration_s=0.)
        behavior, target, yaw, _ = evidence
        model = copy.deepcopy(self)
        state = model.snapshot()
        mode = actual_mode(state.medium_flag)
        samples = [(0., state.position)]
        t = 0.
        status, reason = 'FEASIBLE', 'BOUNDED_QN_REFERENCE_CONTINUATION'
        violation = scene.violation(state.position, self.collision_radius_m)
        if violation or mode not in ('AIR', 'WATER'):
            status, reason = 'INFEASIBLE', violation or 'QN_IDLE_TERMINAL_MODE_NOT_STABLE'
        # A compiled copy of the same source can approach a fixed point over
        # several small controller steps. Never approximate that transient:
        # integrate it normally, then recognize only an *exact* repeated full
        # backend state under the constant INITIAL_HOLD input.
        import pickle
        fixed_signature=None;fixed_position=None
        for _ in range(0 if status != 'FEASIBLE' else math.ceil(duration / dt)):
            if time.monotonic() >= deadline:
                status, reason = 'UNKNOWN', 'QUERY_BUDGET_EXHAUSTED'
                break
            point = state.position if behavior == 'INITIAL_HOLD' else target
            position_before=state.position
            command = ControlCmd('read-only-idle', state.agent_id, t, CommandMode.DESIRED_POSITION,
                                 (0., 0., 0.), desired_position=point, desired_yaw_rad=yaw)
            result = model.step(PlantStepInput(state, command, PlatformAdapterCmd(state.agent_id), dt, 2., 8.))
            t += dt
            state = model.snapshot(t)
            samples.append((t, state.position))
            for position in result.diagnostics['model_positions']:
                violation = scene.violation(position, self.collision_radius_m)
                if violation or actual_mode(medium_flag(position[2], self.constants.hg_m)) != mode:
                    status, reason = 'INFEASIBLE', violation or 'QN_IDLE_MEDIUM_CHANGED'
                    break
            if status != 'FEASIBLE':
                break
            if (behavior=='INITIAL_HOLD' and state.position==position_before and
                    all(position==state.position for position in result.diagnostics['model_positions'])):
                signature=pickle.dumps(vars(model),protocol=4)
                if signature==fixed_signature and state.position==fixed_position:
                    # For deterministic F(x,u), exact F(x,u)=x and the same
                    # constant input imply every later state is x by induction.
                    # Only this read-only forecast skips repeated calculations;
                    # qn_aav_node still integrates every physical model step.
                    for _ in range(math.ceil(duration/dt)-(len(samples)-1)):
                        if time.monotonic()>=deadline:
                            status,reason='UNKNOWN','QUERY_BUDGET_EXHAUSTED'
                            break
                        t+=dt;samples.append((t,state.position))
                    if status=='FEASIBLE':reason='QN_EXACT_INITIAL_HOLD_FIXED_POINT'
                    break
                fixed_signature,fixed_position=signature,state.position
            else:
                fixed_signature=fixed_position=None
        if time.monotonic() >= deadline and status == 'FEASIBLE':
            status, reason = 'UNKNOWN', 'QUERY_BUDGET_EXHAUSTED'
        model._idle_reference = (behavior, target, yaw, model._state)
        return dict(status=status, reason=reason, duration_s=t, requested_wait_s=duration,
                    terminal_position=state.position, terminal_mode=actual_mode(state.medium_flag),
                    initial_mode=mode, collision_radius_m=self.collision_radius_m,
                    trajectory=tuple(samples), terminal_backend=model)

    def execution_state_bytes(self):
        """Complete qn plant/controller/actuator state for local qualification."""
        from .contracts import canonical_model_state_bytes
        if self._state is None:
            raise ValueError('qn model state is not initialized')
        # _idle_reference is a read-only query witness, never consumed by the
        # running qn controller.  The live node has no such witness, so it must
        # not make equal physical/controller states hash differently.
        return canonical_model_state_bytes({key:value for key,value in vars(self).items()
                                            if key!='_idle_reference'})

    def snapshot(self, timestamp_s=0.0):
        """Physical state of this exact plant; controller/actuator state stays here."""
        if self._state is None:
            raise RuntimeError('qn backend has not been initialized')
        plant=self._state.plant
        return AgentState(self._agent_id,self._platform_type,timestamp_s,plant.position_xyz_m,
            self._map_velocity(plant),orientation_quat_wxyz=plant.quaternion_wxyz,
            body_linear_velocity_mps=plant.body_twist[:3],body_angular_velocity_radps=plant.body_twist[3:],
            medium_flag=medium_flag(plant.position_xyz_m[2],self.constants.hg_m))

    def predict_native_fragment(self, segments, scene, deadline, *, dt=.01,
                                position_tolerance=.2, terminal_speed=.03, hold_duration=4.,
                                radius=.25, max_model_time=180., include_state=False,terminal_wait_s=0.):
        """Bounded read-only rollout of the existing local finite qn program.

        All RBF, actuator, memory and guidance states are copied. No reset or
        ROS publication occurs. The snapshot caller must retain the source
        version; this sampled result is not a robust tracking-error bound.
        """
        import copy
        import time
        from .contracts import ControlCmd,CommandMode,PlatformAdapterCmd
        from .platform_execution import actual_mode,validate_fragment,segment_terminal_ready
        if not math.isfinite(deadline) or any(not math.isfinite(v) or v<=0 for v in
                (dt,position_tolerance,terminal_speed,hold_duration,radius,max_model_time)):
            raise ValueError('query needs finite deadline and native positive limits')
        source=self.snapshot()
        if not math.isfinite(terminal_wait_s) or not 0<=terminal_wait_s<max_model_time:
            raise ValueError('terminal wait must fit the finite model horizon')
        def reply(status,reason,t=0.,state=source):
            return dict(status=status,reason=reason,duration_s=t,terminal_position=state.position,
                        terminal_mode=actual_mode(state.medium_flag),geometry_checked=scene is not None,
                        collision_radius_m=radius)
        if time.monotonic()>=deadline:return reply('UNKNOWN','PLANNING_BUDGET_EXHAUSTED')
        if (self.reference_mode!='ROUTE_POSITION' or self.water_horizontal_controller_mode!='LOS_SURGE_YAW' or
                self.water_guidance_mode!='LOS_VELOCITY_REFERENCE'):
            return reply('UNKNOWN','QN_METHOD_NOT_QUALIFIED')
        if scene is None:return reply('UNKNOWN','SCENE_GEOMETRY_REQUIRED')
        try:
            validate_fragment(segments,actual_mode(source.medium_flag),'FIXED_REFERENCE',
                              frozenset(('ENTER_WATER','WATER_PATH','EXIT_WATER')))
        except ValueError as error:return reply('INFEASIBLE',str(error))
        if (math.dist(source.position,segments[0].points[0])>position_tolerance or
                _norm3(source.velocity)>terminal_speed):
            return reply('INFEASIBLE','FRAGMENT_ENTRY_NOT_SETTLED_AT_START')
        for segment in segments:
            if actual_mode(medium_flag(segment.points[-1][2],self.constants.hg_m))!=segment.target_mode:
                return reply('INFEASIBLE','ENDPOINT_MEDIUM_MISMATCH')
            if segment.operation!='WATER_PATH' and any(p[:2]!=segment.points[0][:2] for p in segment.points):
                return reply('UNKNOWN','NON_VERTICAL_TRANSITION_NOT_QUALIFIED')
            reason=scene.path_violation(segment.points,radius)
            if reason:return reply('INFEASIBLE',reason)
        candidate=copy.deepcopy(self)
        state=source;t=0.;start=0.;settled=None;index=0;trajectory=[]
        while t<=max_model_time:
            if time.monotonic()>=deadline:return reply('UNKNOWN','PLANNING_BUDGET_EXHAUSTED',t,state)
            segment=segments[index]
            reason=scene.violation(state.position,radius)
            if reason:return reply('INFEASIBLE',reason,t,state)
            if segment.operation=='WATER_PATH' and actual_mode(state.medium_flag)!='WATER':
                return reply('INFEASIBLE','ACTUAL_MEDIUM_OUTSIDE_WATER_PHASE',t,state)
            target=segment.reference(t-start)
            if segment_terminal_ready(segment,t-start,state.position,state.velocity,actual_mode(state.medium_flag),
                                      position_tolerance,terminal_speed):
                if settled is None:settled=t
            else:settled=None
            required_hold=hold_duration+(terminal_wait_s if index==len(segments)-1 else 0.)
            if settled is not None and t-settled>=required_hold:
                if index==len(segments)-1:
                    candidate.retain_idle_reference()
                    result=reply('FEASIBLE','COMPLETE_NATIVE_FRAGMENT_AND_TERMINAL',t,state)
                    if include_state:result.update(terminal_backend=candidate,trajectory=tuple(trajectory))
                    return result
                index+=1;start=t;settled=None
                # Like the real worker, this step still uses the preceding
                # endpoint. The next completed tick uses the next segment.
            cmd=ControlCmd('read-only-query',source.agent_id,t,CommandMode.DESIRED_POSITION,
                           (0.,0.,0.),desired_position=target,desired_yaw_rad=0.)
            # These existing node defaults only label backend diagnostics;
            # qn never clamps state using them. Geometry/terminal feasibility
            # here does not assert a certified dynamic tracking envelope.
            result=candidate.step(PlantStepInput(state,cmd,PlatformAdapterCmd(source.agent_id),dt,2.,8.))
            for position in result.diagnostics['model_positions']:
                reason=scene.violation(position,radius)
                if reason:return reply('INFEASIBLE',reason,t,state)
            t+=dt;state=candidate.snapshot(t)
            if include_state:trajectory.append((t,state.position))
        return reply('UNKNOWN','MODEL_HORIZON_EXHAUSTED',t,state)

    def step(self, step_input: PlantStepInput) -> PlantStepResult:
        if self._state is None or self._agent_id is None:
            raise RuntimeError("qn Python closed-loop backend must be reset before step")
        if step_input.state.agent_id != self._agent_id:
            raise ValueError("qn backend instance cannot be shared across agents")
        ratio = step_input.dt_s / self.model_step_s
        substeps = round(ratio)
        if substeps < 1 or not math.isclose(ratio, substeps, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                "qn outer dt_s must be an integer multiple of model_step_s "
                f"({step_input.dt_s} / {self.model_step_s})"
            )
        # Retain the existing input for cold terminal-continuation queries;
        # an ordinary step does not prove that an active program has finished.
        self._reference_yaw_rad = step_input.control_cmd.desired_yaw_rad
        self._reference_input_position_m = step_input.control_cmd.desired_position
        self._idle_reference = None
        previous_reference = self._reference_position_m
        reference_position = self._reference_position(step_input)
        # 原模型先对实际输入的xd/yd求导。不能用安全层未同步修改的desired_velocity
        # 替代最终位置参考的导数，否则位置模式又会隐含第二个控制入口。
        reference_velocity = tuple(
            (reference_position[index] - previous_reference[index]) / step_input.dt_s
            for index in range(3)
        )
        self._reference_position_m = reference_position
        reference = QnReference(
            position_m=reference_position,
            velocity_mps=reference_velocity,
            yaw_rad=step_input.control_cmd.desired_yaw_rad,
        )
        water_guidance = (
            QnWaterActuatorGuidance(
                desired_speed_mps=self._water_horizontal_speed_mps,
                desired_heading_rad=self._water_desired_heading_rad,
                minimum_turn_surge_fraction=self.water_min_turn_surge_fraction,
            )
            if self.water_horizontal_controller_mode == "LOS_SURGE_YAW"
            and self._water_guidance_active
            else None
        )
        previous_velocity = self._map_velocity(self._state.plant)
        collect_extrema = self.reference_mode == "ROUTE_POSITION"
        model_velocity = previous_velocity
        model_acceleration = self._last_model_acceleration
        extrema = {"speed_mps": _norm3(previous_velocity), "acceleration_mps2": 0.,
                   "jerk_mps3": 0., "yaw_rate_rps": 0., "model_step_s": self.model_step_s,
                   "position_reference_error_m": _norm3(tuple(self._state.plant.position_xyz_m[i]-reference_position[i] for i in range(3)))}
        model_positions = []
        state = self._state
        last_commands: Vector10 = (0.0,) * 10
        for _ in range(substeps):
            state, last_commands = qn_closed_loop_ode4_step(
                state,
                reference,
                self.model_step_s,
                self.constants,
                water_guidance=water_guidance,
            )
            if collect_extrema:
                velocity_now = self._map_velocity(state.plant)
                acceleration_now = tuple((velocity_now[i]-model_velocity[i])/self.model_step_s for i in range(3))
                extrema["speed_mps"] = max(extrema["speed_mps"], _norm3(velocity_now))
                extrema["acceleration_mps2"] = max(extrema["acceleration_mps2"], _norm3(acceleration_now))
                if model_acceleration is not None:
                    extrema["jerk_mps3"] = max(extrema["jerk_mps3"], _norm3(tuple(
                        (acceleration_now[i]-model_acceleration[i])/self.model_step_s for i in range(3))))
                extrema["yaw_rate_rps"] = max(extrema["yaw_rate_rps"], abs(state.plant.body_twist[5]))
                extrema["position_reference_error_m"] = max(extrema["position_reference_error_m"], _norm3(tuple(
                    state.plant.position_xyz_m[i]-reference_position[i] for i in range(3))))
                model_positions.append(state.plant.position_xyz_m)
                model_velocity, model_acceleration = velocity_now, acceleration_now
        self._last_model_acceleration = model_acceleration
        self._state = state
        quaternion = normalize_quaternion(state.plant.quaternion_wxyz)
        velocity = body_to_map_velocity(state.plant.body_twist[:3], quaternion)
        acceleration = tuple(
            (velocity[index] - previous_velocity[index]) / step_input.dt_s
            for index in range(3)
        )
        tracking_error = _norm3(
            tuple(
                step_input.control_cmd.desired_velocity[index] - velocity[index]
                for index in range(3)
            )
        )
        flag = medium_flag(state.plant.position_xyz_m[2], self.constants.hg_m)
        return PlantStepResult(
            position=state.plant.position_xyz_m,
            velocity=velocity,
            acceleration=acceleration,
            velocity_tracking_error_mps=tracking_error,
            acceleration_limited=False,
            speed_limited=False,
            backend_id=self.backend_id,
            orientation_quat_wxyz=quaternion,
            body_linear_velocity_mps=state.plant.body_twist[:3],
            body_angular_velocity_radps=state.plant.body_twist[3:],
            medium_flag=flag,
            diagnostics={
                "model_source": "qn.slx Python transcription",
                "initialization_mode": self.initialization_mode,
                "solver": "ode4",
                "model_step_s": self.model_step_s,
                "substeps": substeps,
                "model_extrema": extrema if collect_extrema else None,
                "model_positions": tuple(model_positions),
                "reference_mode": self.reference_mode,
                "control_semantics": (
                    "MAP_POSITION_TO_QN_INNER_LOOP"
                    if self.reference_mode == "ROUTE_POSITION"
                    else "MAP_VELOCITY_INTEGRAL_TO_QN_POSITION_INNER_LOOP"
                ),
                "acceleration_directly_consumed": False,
                "reference_position_m": reference_position,
                "reference_velocity_mps": reference_velocity,
                # The reference the controller actually used this step.  The
                # publisher records this instead of recomputing a second,
                # possibly diverging derivation of the same quantity.
                "reference_yaw_rad": reference.yaw_rad,
                "reference_used": {
                    "position_m": reference.position_m,
                    "velocity_mps": reference.velocity_mps,
                    "yaw_rad": reference.yaw_rad,
                },
                "reference_lead_m": self._reference_lead_m,
                "reference_lead_limit_m": self.max_reference_lead_m,
                "reference_lead_clamped": self._reference_lead_clamped,
                "water_guidance_mode": self.water_guidance_mode,
                "water_horizontal_controller_mode": self.water_horizontal_controller_mode,
                "water_guidance_active": self._water_guidance_active,
                "water_heading_error_rad": self._water_heading_error_rad,
                "water_target_heading_error_rad": self._water_target_heading_error_rad,
                "water_desired_heading_rad": self._water_desired_heading_rad,
                "water_steering_signal": self._water_steering_signal,
                "water_surge_alignment": self._water_surge_alignment,
                "actuator_commands": dict(zip(
                    ("T1", "T2", "Tf", "Tp1", "Tp2", "Ts1", "Ts2", "det1", "det2", "det3"),
                    last_commands,
                )),
                "actuator_outputs": state.plant.actuator_outputs,
                "physical_speed_limit_exceeded": (
                    _norm3(velocity) > step_input.max_speed_mps + 1e-9
                ),
                "physical_acceleration_limit_exceeded": (
                    _norm3(acceleration) > step_input.max_acc_mps2 + 1e-9
                ),
            },
        )

    def _reference_position(self, step_input: PlantStepInput) -> Vector3:
        if self.reference_mode == "ROUTE_POSITION":
            if step_input.control_cmd.desired_position is None:
                raise ValueError("ROUTE_POSITION requires ControlCmd.desired_position")
            candidate = step_input.control_cmd.desired_position
            # One reference contract: derive guidance from the adopted position
            # stream, just as step() derives the original qn reference velocity.
            velocity = tuple((candidate[i] - self._reference_position_m[i]) /
                             step_input.dt_s for i in range(3))
        else:
            if self._reference_position_m is None:
                raise RuntimeError("qn reference integrator must be reset before step")
            velocity = step_input.control_cmd.desired_velocity
            candidate = tuple(
                self._reference_position_m[index]
                + velocity[index] * step_input.dt_s
                for index in range(3)
            )
        # qn 原控制器接收位置参考，HUC 局部安全层最终输出安全速度。适配器对该速度
        # 积分成连续虚拟位置参考；安全速度为零时参考点保持不动，qn 因而能真正 HOLD。
        actual = self._state.plant.position_xyz_m
        delta = tuple(candidate[index] - actual[index] for index in range(3))
        lead = _norm3(delta)
        self._reference_lead_clamped = (self.reference_mode != "ROUTE_POSITION"
                                       and lead > self.max_reference_lead_m)
        if self._reference_lead_clamped:
            scale = self.max_reference_lead_m / lead
            candidate = tuple(
                actual[index] + delta[index] * scale for index in range(3)
            )
            lead = self.max_reference_lead_m
        flag = medium_flag(actual[2], self.constants.hg_m)
        self._water_guidance_active = (
            self.water_guidance_mode == "LOS_VELOCITY_REFERENCE" and flag > 0.0
        )
        if self._water_guidance_active:
            # A timed reference may stop ahead of the actual plant after a
            # turn. Pure feed-forward reference velocity then becomes zero
            # and strands the vehicle outside its terminal region. Keep the
            # existing qn controller/actuators, but let the guidance velocity
            # close the remaining position error over its declared lead time.
            capture_time = max(1.0, self.water_surge_reference_gain_s)
            guidance_x = velocity[0] + (candidate[0] - actual[0]) / capture_time
            guidance_y = velocity[1] + (candidate[1] - actual[1]) / capture_time
            horizontal_speed = min(step_input.max_speed_mps, math.hypot(guidance_x, guidance_y))
            self._water_horizontal_speed_mps = horizontal_speed
            _, _, yaw = _qn_attitude(
                normalize_quaternion(self._state.plant.quaternion_wxyz)
            )
            target_heading = (
                math.atan2(guidance_y, guidance_x)
                if horizontal_speed > 1e-6
                else yaw
            )
            target_heading_error = _wrap_pi(target_heading - yaw)
            heading_step = _wrap_pi(target_heading - self._water_desired_heading_rad)
            max_heading_step = self.water_heading_rate_limit_radps * step_input.dt_s
            heading_step = max(-max_heading_step, min(max_heading_step, heading_step))
            self._water_desired_heading_rad = _wrap_pi(
                self._water_desired_heading_rad + heading_step
            )
            heading_error = _wrap_pi(self._water_desired_heading_rad - yaw)
            steering_signal = heading_error - (
                self.water_yaw_rate_damping_s * self._state.plant.body_twist[5]
            )
            steering_signal = max(-math.pi, min(math.pi, steering_signal))
            surge_alignment = (
                max(
                    self.water_min_turn_surge_fraction,
                    max(0.0, math.cos(target_heading_error)),
                )
                if horizontal_speed > 1e-6
                else 0.0
            )
            surge_lead = min(
                self.water_surge_reference_max_m,
                self.water_surge_reference_gain_s * horizontal_speed,
            ) * surge_alignment
            water_reference = (
                actual[0] + surge_lead,
                actual[1] + self.water_steering_reference_scale_m * steering_signal,
                candidate[2],
            )
            # 只有原 ex/ey 水下通道需要这种历史位置编码。直接 Tf/det3 模式
            # 已消费 speed/heading，必须保留世界系位置参考；否则过渡期仍在
            # 工作的空中 x/y 控制器会把 surge/steering 编码误作位置目标。
            if self.water_horizontal_controller_mode == "QN_ORIGINAL_RBF_PD":
                candidate = tuple(
                    (1.0 - flag) * candidate[index]
                    + flag * water_reference[index]
                    for index in range(3)
                )
            self._water_heading_error_rad = heading_error
            self._water_target_heading_error_rad = target_heading_error
            self._water_steering_signal = steering_signal
            self._water_surge_alignment = surge_alignment
        else:
            self._water_heading_error_rad = 0.0
            self._water_target_heading_error_rad = 0.0
            self._water_steering_signal = 0.0
            self._water_surge_alignment = 1.0
            self._water_horizontal_speed_mps = 0.0
        self._reference_position_m = candidate  # type: ignore[assignment]
        self._reference_lead_m = lead
        return self._reference_position_m

    @staticmethod
    def _map_velocity(plant: QnEquationState) -> Vector3:
        return body_to_map_velocity(
            plant.body_twist[:3], normalize_quaternion(plant.quaternion_wxyz)
        )


@dataclass(frozen=True)
class QnReference:
    """位置参考及其导数；velocity_mps不是平台速度控制指令。"""

    position_m: Vector3
    velocity_mps: Vector3
    yaw_rad: float


def _static_trim_state(
    state: QnClosedLoopState, constants: QnModelConstants
) -> QnClosedLoopState:
    """由原模型静力平衡求驻留初态；不修改原控制律、物理系数或执行器动态。"""
    from dataclasses import replace

    plant = state.plant
    flag = medium_flag(plant.position_xyz_m[2], constants.hg_m)
    if flag not in {0.0, 1.0}:
        raise ValueError("STATIC_TRIM requires an AIR or WATER initial position")
    if _norm3(plant.body_twist[:3]) > 1e-9 or _norm3(plant.body_twist[3:]) > 1e-9:
        raise ValueError("STATIC_TRIM requires zero initial body velocity")
    if abs(plant.quaternion_wxyz[1]) + abs(plant.quaternion_wxyz[2]) > 1e-9:
        raise ValueError("STATIC_TRIM requires a level initial attitude")
    buoyancy, _ = buoyancy_wrench(
        plant.quaternion_wxyz, plant.position_xyz_m[2], constants
    )
    mass, _ = mass_and_inertia(flag, constants)
    thrust = (mass * constants.gravity_mps2 + buoyancy[2]) / 2.0
    commands = [0.0] * 10
    channel = 0 if flag == 0.0 else 6
    if flag == 0.0:
        if not 0.0 <= thrust <= 50.0:
            raise ValueError("AIR static trim exceeds qn actuator limits")
        commands[0] = commands[1] = thrust
    else:
        if not -25.0 <= thrust <= 25.0:
            raise ValueError("WATER static trim exceeds qn actuator limits")
        commands[3] = commands[4] = thrust
    # observer二阶平衡要求 fxp + b*u = 0；同一RBF基函数展开求最小范数权重。
    basis = tuple(math.exp(-(c[0] ** 2 + c[1] ** 2) / 50.0) for c in _RBF_CENTERS)
    disturbance = -QN_CONTROLLER_PARAMETERS[channel].input_gain * thrust
    denominator = sum(value * value for value in basis)
    controllers = list(state.controllers)
    controllers[channel] = replace(
        controllers[channel],
        weights=tuple(disturbance * value / denominator for value in basis),
    )
    memories = list(state.controller_fxp_memory)
    memories[channel] = disturbance
    return replace(
        state,
        plant=replace(plant, actuator_outputs=tuple(commands), buoyancy_memory=buoyancy),
        controllers=tuple(controllers),
        controller_fxp_memory=tuple(memories),
    )


@dataclass(frozen=True)
class QnWaterActuatorGuidance:
    """HUC 实时导引到 qn 水下前推/转向执行器的显式适配参数。"""

    desired_speed_mps: float
    desired_heading_rad: float
    minimum_turn_surge_fraction: float
    speed_kp: float = 3.0
    heading_kp: float = 1.5
    yaw_rate_kd: float = 1.0


def qn_closed_loop_ode4_step(
    state: QnClosedLoopState,
    reference: QnReference,
    dt_s: float,
    constants: QnModelConstants,
    *,
    water_guidance: QnWaterActuatorGuidance | None = None,
) -> tuple[QnClosedLoopState, Vector10]:
    """按 qn 的 ode4 同时积分控制器、执行器和 13 状态刚体。"""
    from itertools import chain

    flat = _flatten_closed_loop_state(state)
    buoyancy_memory = state.plant.buoyancy_memory
    controller_memories = state.controller_fxp_memory

    def derivative(values: tuple[float, ...]) -> tuple[float, ...]:
        # RK stages already contain native float tuples. Build only the
        # normalized algebraic plant, without an unused enclosing state or
        # converting every controller scalar back to float at every stage.
        controllers = tuple([
            QnControllerState(values[index:index + 2], values[index + 2:index + 9])
            for index in range(23, len(values), 9)
        ])
        actuator_outputs = values[13:23]
        normalized_plant = QnEquationState(
            body_twist=values[:6],
            quaternion_wxyz=normalize_quaternion(values[6:10]),
            position_xyz_m=values[10:13],
            actuator_outputs=actuator_outputs,
            buoyancy_memory=buoyancy_memory,
        )
        commands, controller_dots, _ = controller_output_and_derivatives(
            controllers,
            controller_memories,
            normalized_plant,
            reference,
            constants,
            water_guidance,
        )
        actuator_dot = tuple([
            constants.actuator_pole_radps
            * (command - output)
            for command, output in zip(commands, actuator_outputs)
        ])
        plant_dot = rigid_body_derivative(normalized_plant, constants) + actuator_dot
        return plant_dot + tuple(chain.from_iterable(controller_dots))

    k1 = derivative(flat)
    k2 = derivative(_add_scaled(flat, k1, dt_s * 0.5))
    k3 = derivative(_add_scaled(flat, k2, dt_s * 0.5))
    k4 = derivative(_add_scaled(flat, k3, dt_s))
    next_flat = tuple([
        value
        + dt_s
        * (first + 2.0 * second + 2.0 * third + fourth)
        / 6.0
        for value, first, second, third, fourth in zip(flat, k1, k2, k3, k4)
    ])
    next_state = _unflatten_closed_loop_state(
        next_flat,
        state.plant.buoyancy_memory,
        state.controller_fxp_memory,
    )
    quaternion = normalize_quaternion(next_state.plant.quaternion_wxyz)
    normalized_plant = QnEquationState(
        body_twist=next_state.plant.body_twist,
        quaternion_wxyz=quaternion,
        position_xyz_m=next_state.plant.position_xyz_m,
        actuator_outputs=next_state.plant.actuator_outputs,
        buoyancy_memory=next_state.plant.buoyancy_memory,
    )
    commands, _, fxp_inputs = controller_output_and_derivatives(
        next_state.controllers,
        next_state.controller_fxp_memory,
        normalized_plant,
        reference,
        constants,
        water_guidance,
    )
    buoyancy, _ = buoyancy_wrench(
        quaternion, next_state.plant.position_xyz_m[2], constants
    )
    return (
        QnClosedLoopState(
            plant=QnEquationState(
                body_twist=next_state.plant.body_twist,
                # 保留四元数积分器原状态；归一化块是其后的代数输出块。
                quaternion_wxyz=next_state.plant.quaternion_wxyz,
                position_xyz_m=next_state.plant.position_xyz_m,
                actuator_outputs=next_state.plant.actuator_outputs,
                buoyancy_memory=buoyancy,
            ),
            controllers=next_state.controllers,
            controller_fxp_memory=fxp_inputs,
        ),
        commands,
    )


def controller_output_and_derivatives(
    controllers: tuple[QnControllerState, ...],
    memories: tuple[float, ...],
    plant: QnEquationState,
    reference: QnReference,
    constants: QnModelConstants,
    water_guidance: QnWaterActuatorGuidance | None = None,
) -> tuple[Vector10, tuple[tuple[float, ...], ...], tuple[float, ...]]:
    """复现 11/control 的级联、模式门控、饱和和执行器分配。"""
    flag = medium_flag(plant.position_xyz_m[2], constants.hg_m)
    roll, pitch, yaw = _qn_attitude(plant.quaternion_wxyz)
    x, y, height = plant.position_xyz_m
    desired_x, desired_y, desired_height = reference.position_m
    error_x = x - desired_x
    error_y = y - desired_y
    xdn = error_x * math.cos(yaw) + error_y * math.sin(yaw)
    ydn = error_y * math.cos(yaw) - error_x * math.sin(yaw)
    # chart_1039：水下总线虽命名ex/ey，实际接入的是按参考切向变换的xdn/ydn。
    # 原cart2pol(0,0)返回0，不增加保持上一航向等原模型没有的控制逻辑。
    dx, dy = reference.velocity_mps[:2]
    path_heading = math.atan2(dy, dx) if dx != 0.0 or dy != 0.0 else 0.0
    water_xdn = error_x * math.cos(path_heading) + error_y * math.sin(path_heading)
    water_ydn = error_y * math.cos(path_heading) - error_x * math.sin(path_heading)
    yaw_error = _wrap_pi(yaw - reference.yaw_rad)
    measurements = [
        height - desired_height,
        pitch,
        roll,
        xdn,
        ydn,
        yaw_error,
        height - desired_height,
        water_xdn,
        pitch,
        roll,
        water_ydn,
        0.0,
        0.0,
    ]
    active = (flag != 1.0,) * 6 + (flag != 0.0,) * 7

    # 外环 x/y 先产生俯仰/滚转参考；其余通道参考均为未连接端口的默认 0。
    references = [0.0] * len(QN_CONTROLLER_NAMES)
    air_x_output = _pd_output(3, controllers[3], memories[3], 0.0, active[3])
    air_y_output = _pd_output(4, controllers[4], memories[4], 0.0, active[4])
    references[1] = _saturate(-air_x_output, -1.0, 1.0) if active[1] else 0.0
    references[2] = _saturate(air_y_output, -1.0, 1.0) if active[2] else 0.0

    outputs = [
        _pd_output(index, controllers[index], memories[index], references[index], active[index])
        if index not in (3, 4) else (air_x_output if index == 3 else air_y_output)
        for index in range(len(QN_CONTROLLER_NAMES))
    ]
    controller_dots = []
    fxp_inputs = []
    for index, controller in enumerate(controllers):
        # A gated channel with zero PD output has exactly nine zero
        # derivatives, regardless of its retained (ungated) integrator state.
        # Keep that state in RK4; only avoid evaluating the same zero inputs.
        if not active[index] and outputs[index] == 0.0:
            controller_dots.append((0.0,) * 9)
            fxp_inputs.append(0.0)
            continue
        derivative, fxp = _rbf_derivative(
            index,
            controller,
            outputs[index],
            measurements[index] if active[index] else 0.0,
            active[index],
        )
        controller_dots.append(derivative)
        # 每个 PD 子系统的 fxp 在进入 Memory 前还经过同介质门控。
        fxp_inputs.append(fxp if active[index] else 0.0)

    air_height, air_pitch, air_roll, _, _, air_yaw = outputs[:6]
    water_z, water_x, water_pitch, water_roll, water_y = outputs[6:11]
    if flag == 1.0:
        air_height = air_pitch = air_roll = air_yaw = 0.0
    if flag == 0.0:
        water_z = water_x = water_pitch = water_roll = water_y = 0.0

    wu1 = _saturate(air_height + air_pitch, 0.0, 50.0)
    wu2 = _saturate(air_height - air_pitch, 0.0, 50.0)
    det1 = _saturate(air_roll + air_yaw, -1.0, 1.0)
    det2 = _saturate(air_roll - air_yaw, -1.0, 1.0)
    wp1 = _saturate(water_z + water_pitch, -25.0, 25.0)
    wp2 = _saturate(water_z - water_pitch, -25.0, 25.0)
    ws1 = _saturate(water_roll, -25.0, 25.0)
    ws2 = _saturate(-water_roll, -25.0, 25.0)
    wf = _saturate(water_x, 0.0, 30.0)
    det3 = _saturate(water_y, -1.0, 1.0)
    if water_guidance is not None and flag > 0.0:
        heading_error = _wrap_pi(water_guidance.desired_heading_rad - yaw)
        alignment = max(
            water_guidance.minimum_turn_surge_fraction,
            max(0.0, math.cos(heading_error)),
        )
        desired_surge = max(0.0, water_guidance.desired_speed_mps) * alignment
        actual_surge = plant.body_twist[0]
        # 线性水阻的稳态前馈 + surge 误差反馈。只替换原模型无法支持任意航向的
        # 水平 ex/ey 两通道，深度、姿态、介质切换、执行器滞后和 6DOF 均保持 qn。
        wf = _saturate(
            constants.water_damping[0] * desired_surge
            + water_guidance.speed_kp * (desired_surge - actual_surge),
            0.0,
            30.0,
        )
        det3 = _saturate(
            water_guidance.heading_kp * heading_error
            - water_guidance.yaw_rate_kd * plant.body_twist[5],
            -1.0,
            1.0,
        )
    commands: Vector10 = (wu1, wu2, wf, wp1, wp2, ws1, ws2, det1, det2, det3)
    return commands, tuple(controller_dots), tuple(fxp_inputs)


def _pd_output(
    index: int,
    state: QnControllerState,
    fxp_memory: float,
    reference: float,
    active: bool,
) -> float:
    params = QN_CONTROLLER_PARAMETERS[index]
    observer = state.observer if active else (0.0, 0.0)
    z1 = observer[0] - reference
    z2 = observer[1]
    return (-params.k1 * z1 - params.k2 * z2 - fxp_memory) / params.input_gain


def _rbf_derivative(
    index: int,
    state: QnControllerState,
    control: float,
    measurement: float,
    active: bool,
) -> tuple[tuple[float, ...], float]:
    params = QN_CONTROLLER_PARAMETERS[index]
    observer = state.observer if active else (0.0, 0.0)
    weights = state.weights if active else (0.0,) * 7
    error = measurement - observer[0]
    # 权重全零且自适应项恒为零时无需重复算7个Gaussian；仍积分原方程的全部状态。
    if not any(weights) and (params.adaptation_gain == 0.0 or error == 0.0):
        return (observer[1] + 2.0*params.w0*error,
                0.0 + params.input_gain*control + params.w0*params.w0*error,
                *(0.0,)*7), 0.0
    basis = [
        math.exp(
            -(
                (observer[0] - center[0]) ** 2
                + (observer[1] - center[1]) ** 2
            )
            / (2.0 * 5.0**2)
        )
        for center in _RBF_CENTERS
    ]
    fxp = sum([weight * activation for weight, activation in zip(weights, basis)])
    decay_gain = 0.001 * params.adaptation_gain * abs(error)
    weight_dot = tuple([
        params.adaptation_gain * basis[index_] * error
        - decay_gain * weights[index_]
        for index_ in range(7)
    ])
    observer_dot = (
        observer[1] + 2.0 * params.w0 * error,
        fxp
        + params.input_gain * control
        + params.w0 * params.w0 * error,
    )
    return observer_dot + weight_dot, fxp


def _channel_active(index: int, flag: float) -> bool:
    if index in _AIR_CHANNELS:
        return flag != 1.0
    if index in _WATER_CHANNELS:
        return flag != 0.0
    raise IndexError(index)


def _qn_attitude(quaternion: Quaternion) -> tuple[float, float, float]:
    e0, e1, e2, e3 = quaternion
    roll = math.atan2(
        2.0 * (e0 * e1 + e2 * e3),
        e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2,
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (e0 * e2 - e1 * e3))))
    yaw = math.atan2(
        2.0 * (e0 * e3 + e2 * e1),
        e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3,
    )
    return roll, pitch, yaw


def _wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _saturate(value: float, lower: float, upper: float) -> float:
    # Preserve max/min argument order (including equal and NaN values),
    # while avoiding two Python-to-builtin calls for every actuator limit.
    bounded = value if value > lower else lower
    return bounded if bounded < upper else upper


def _flatten_closed_loop_state(state: QnClosedLoopState) -> tuple[float, ...]:
    from itertools import chain

    plant = state.plant
    return (
        plant.body_twist
        + plant.quaternion_wxyz
        + plant.position_xyz_m
        + plant.actuator_outputs
        + tuple(chain.from_iterable(
            [controller.observer + controller.weights for controller in state.controllers]
        ))
    )


def _unflatten_closed_loop_state(
    values: Sequence[float],
    buoyancy_memory: Vector6,
    controller_memories: tuple[float, ...],
) -> QnClosedLoopState:
    controller_values = tuple(map(float, values[23:]))
    controllers = tuple([
        QnControllerState(
            controller_values[index:index + 2],
            controller_values[index + 2:index + 9],
        )
        for index in range(0, len(QN_CONTROLLER_NAMES) * 9, 9)
    ])
    return QnClosedLoopState(
        plant=QnEquationState(
            body_twist=tuple(values[0:6]),  # type: ignore[arg-type]
            quaternion_wxyz=tuple(values[6:10]),  # type: ignore[arg-type]
            position_xyz_m=tuple(values[10:13]),  # type: ignore[arg-type]
            actuator_outputs=tuple(values[13:23]),  # type: ignore[arg-type]
            buoyancy_memory=buoyancy_memory,
        ),
        controllers=controllers,
        controller_fxp_memory=controller_memories,
    )


def _add_scaled(
    values: Sequence[float], derivative: Sequence[float], scale: float
) -> tuple[float, ...]:
    return tuple([value + scale * slope for value, slope in zip(values, derivative)])


def _norm3(vector: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))
