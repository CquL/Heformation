"""从 ``qn.slx`` 逐式转写的执行器与六自由度动力学。

本模块只实现模型中 ``11/plane modle`` 与 ``11/ODE`` 的有证据部分，不自行补写
``11/control``。因此 backend 必须收到十路执行器命令，不能把期望速度直接当作力。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Tuple

from .contracts import ActuatorCmd, AgentState, Quaternion, Vector3
from .contracts import PlantStepInput, PlantStepResult

Vector6 = Tuple[float, float, float, float, float, float]
Vector10 = Tuple[float, float, float, float, float, float, float, float, float, float]

QN_ACTUATOR_SCHEMA = "QN_AIR_WATER_ACTUATOR_V1"
QN_ACTUATOR_FIELDS = (
    "T1",
    "T2",
    "Tf",
    "Tp1",
    "Tp2",
    "Ts1",
    "Ts2",
    "det1",
    "det2",
    "det3",
)


@dataclass(frozen=True)
class QnModelConstants:
    """直接来自 qn.slx 常量块的物理参数。"""

    gravity_mps2: float = 9.804  # 11/earth environment/g
    buoyancy_gravity_mps2: float = 9.8  # 11/ODE/MATLAB Function1 内部覆盖值
    base_mass_kg: float = 4.0
    added_mass_kg: float = 4.0
    base_inertia_kgm2: Vector3 = (7.6e-3, 317.7e-3, 317.7e-3)
    added_inertia_kgm2: Vector3 = (7.6e-3, 317.7e-3, 317.7e-3)
    air_damping: Vector6 = (2.02, 4.0, 6.08, 2.0, 2.0, 4.4)
    water_damping: Vector6 = (5.0, 30.0, 10.0, 0.2, 0.2, 0.586)
    hg_m: float = 0.17
    lp_m: float = 0.32
    l1_m: float = 0.32
    l2_m: float = 0.064
    lt_m: float = 0.233
    ltz_m: float = 0.0
    buoyancy_volume_m3: float = 0.008
    actuator_pole_radps: float = 30.0


@dataclass(frozen=True)
class QnEquationState:
    """ode4 同时积分的刚体、四元数、位置和十路执行器状态。"""

    body_twist: Vector6
    quaternion_wxyz: Quaternion
    position_xyz_m: Vector3
    actuator_outputs: Vector10
    buoyancy_memory: Vector6


class QnSlxDynamicsBackend:
    """执行 qn.slx 物理方程的 plant backend，不包含未定参数的内部控制器。"""

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.model_step_s = float(config.get("model_step_s", 0.001))
        if self.model_step_s <= 0.0:
            raise ValueError("qn model_step_s must be positive")
        self.constants = QnModelConstants()
        self._agent_id: str | None = None
        self._state: QnEquationState | None = None

    @property
    def backend_id(self) -> str:
        return "QN_SLX_6DOF_DYNAMICS"

    def reset(self, state: AgentState) -> None:
        quaternion = normalize_quaternion(state.orientation_quat_wxyz)
        body_linear = state.body_linear_velocity_mps
        if _norm3(body_linear) <= 1e-12 and _norm3(state.velocity) > 1e-12:
            body_linear = map_velocity_to_body(state.velocity, quaternion)
        body_twist: Vector6 = (
            body_linear[0],
            body_linear[1],
            body_linear[2],
            state.body_angular_velocity_radps[0],
            state.body_angular_velocity_radps[1],
            state.body_angular_velocity_radps[2],
        )
        self._agent_id = state.agent_id
        self._state = QnEquationState(
            body_twist=body_twist,
            quaternion_wxyz=quaternion,
            position_xyz_m=state.position,
            actuator_outputs=(0.0,) * 10,
            buoyancy_memory=(0.0,) * 6,
        )

    def step(self, step_input: PlantStepInput) -> PlantStepResult:
        if self._state is None or self._agent_id is None:
            raise RuntimeError("qn plant backend must be reset before step")
        if step_input.state.agent_id != self._agent_id:
            raise ValueError("qn plant backend instance cannot be shared across agents")
        command = _read_actuator_command(step_input.actuator_command)
        ratio = step_input.dt_s / self.model_step_s
        substeps = round(ratio)
        if substeps < 1 or not math.isclose(ratio, substeps, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                "qn outer dt_s must be an integer multiple of model_step_s "
                f"({step_input.dt_s} / {self.model_step_s})"
            )

        previous_velocity = body_to_map_velocity(
            self._state.body_twist[:3], self._state.quaternion_wxyz
        )
        state = self._state
        for _ in range(substeps):
            state = ode4_step(state, command, self.model_step_s, self.constants)
        self._state = state

        velocity = body_to_map_velocity(
            state.body_twist[:3], state.quaternion_wxyz
        )
        acceleration = tuple(
            (velocity[index] - previous_velocity[index]) / step_input.dt_s
            for index in range(3)
        )
        desired_velocity = step_input.control_cmd.desired_velocity
        tracking_error = _norm3(
            tuple(desired_velocity[index] - velocity[index] for index in range(3))
        )
        mode = medium_flag(state.position_xyz_m[2], self.constants.hg_m)
        return PlantStepResult(
            position=state.position_xyz_m,
            velocity=velocity,
            acceleration=acceleration,
            velocity_tracking_error_mps=tracking_error,
            acceleration_limited=False,
            speed_limited=False,
            backend_id=self.backend_id,
            orientation_quat_wxyz=state.quaternion_wxyz,
            body_linear_velocity_mps=state.body_twist[:3],
            body_angular_velocity_radps=state.body_twist[3:],
            medium_flag=mode,
            diagnostics={
                "model_source": "qn.slx",
                "solver": "ode4",
                "model_step_s": self.model_step_s,
                "substeps": substeps,
                "actuator_schema": QN_ACTUATOR_SCHEMA,
                "actuator_outputs": dict(zip(QN_ACTUATOR_FIELDS, state.actuator_outputs)),
                "physical_speed_limit_exceeded": (
                    _norm3(velocity) > step_input.max_speed_mps + 1e-9
                ),
                "physical_acceleration_limit_exceeded": (
                    _norm3(acceleration) > step_input.max_acc_mps2 + 1e-9
                ),
            },
        )


def medium_flag(height_m: float, hg_m: float) -> float:
    """逐式对应 ``11/MATLAB Function3``。"""
    if height_m >= 0.5 * hg_m:
        return 0.0
    if height_m <= -0.5 * hg_m:
        return 1.0
    return 0.5 - height_m / hg_m


def mass_and_inertia(
    flag: float, constants: QnModelConstants
) -> tuple[float, Vector3]:
    """逐式对应 ``plane modle/MATLAB Function1/2``。"""
    if flag == 0.0:
        return constants.base_mass_kg, constants.base_inertia_kgm2
    if flag == 1.0:
        return (
            constants.base_mass_kg + constants.added_mass_kg,
            tuple(
                constants.base_inertia_kgm2[i]
                + constants.added_inertia_kgm2[i]
                for i in range(3)
            ),
        )
    factor = flag + 1.0
    return (
        constants.base_mass_kg * factor,
        tuple(value * factor for value in constants.base_inertia_kgm2),
    )


def actuator_wrench(
    actuator: Sequence[float], flag: float, constants: QnModelConstants
) -> Vector6:
    """逐式对应 ``11/ODE/F``，输入顺序与 qn 十路执行器一致。"""
    t1, t2, tf, tp1, tp2, ts1, ts2, det1, det2, det3 = actuator
    air: Vector6 = (
        0.0,
        t1 * math.sin(det1) + t2 * math.sin(det2),
        -t1 * math.cos(det1) - t2 * math.cos(det2),
        (t1 * math.sin(det1) + t2 * math.sin(det2)) * constants.hg_m * 0.5,
        constants.lp_m * (t1 * math.cos(det1) - t2 * math.cos(det2)),
        constants.lp_m * (t1 * math.sin(det1) - t2 * math.sin(det2)),
    )
    water: Vector6 = (
        tf * math.cos(det3),
        tf * math.sin(det3),
        -tp1 - tp2,
        constants.l2_m * (ts1 - ts2),
        constants.lt_m * (tp1 - tp2)
        + constants.ltz_m * tf * math.cos(det3),
        constants.l1_m * tf * math.sin(det3),
    )
    if flag == 0.0:
        return air
    if flag == 1.0:
        return water
    return tuple((1.0 - flag) * air[i] + flag * water[i] for i in range(6))


def damping_wrench(
    body_twist: Vector6, flag: float, constants: QnModelConstants
) -> Vector6:
    """逐式对应 ``11/ODE/F1`` 的两组线性阻尼插值。"""
    return tuple(
        -(
            (1.0 - flag) * constants.air_damping[i]
            + flag * constants.water_damping[i]
        )
        * body_twist[i]
        for i in range(6)
    )


def buoyancy_wrench(
    quaternion: Quaternion, height_m: float, constants: QnModelConstants
) -> tuple[Vector6, int]:
    """逐分支对应 ``11/ODE/MATLAB Function1``，包括源模型的叉乘顺序。"""
    dcm = qn_dcm(quaternion)
    y1 = _mat_vec(dcm, (0.0, 0.4, 0.0))
    y2 = _mat_vec(dcm, (0.0, -0.4, 0.0))
    h1 = y1[2] + height_m
    h2 = y2[2] + height_m
    volume = 0.0
    moment_arm = (0.0, 0.0, 0.0)
    branch = 5
    if h1 > 0.0 and h2 > 0.0:
        branch = 1
    elif h1 > 0.0 and h2 < 0.0:
        volume = constants.buoyancy_volume_m3 * math.sqrt(
            y2[0] ** 2 + y2[1] ** 2 + (height_m + y2[2]) ** 2
        )
        moment_arm = (y2[0] / 2.0, y2[1] / 2.0, (height_m + y2[2]) / 2.0)
        branch = 2
    elif h1 < 0.0 and h2 > 0.0:
        volume = constants.buoyancy_volume_m3 * math.sqrt(
            y1[0] ** 2 + y1[1] ** 2 + (height_m + y1[2]) ** 2
        )
        moment_arm = (y1[0] / 2.0, y1[1] / 2.0, (height_m + y1[2]) / 2.0)
        branch = 3
    elif h1 < 0.0 and h2 < 0.0:
        volume = constants.buoyancy_volume_m3
        moment_arm = tuple((y1[i] + y2[i]) / 2.0 for i in range(3))
        branch = 4
    force: Vector3 = (
        0.0,
        0.0,
        -1000.0 * constants.buoyancy_gravity_mps2 * volume,
    )
    # qn 源码是 cross(f, arm)，不是更常见的 cross(arm, f)。
    moment = _cross(force, moment_arm)
    return force + moment, branch


def ode4_step(
    state: QnEquationState,
    actuator_command: Vector10,
    dt_s: float,
    constants: QnModelConstants,
) -> QnEquationState:
    """按 Simulink ``ode4`` 对连续状态推进一个固定主步。"""
    flat = _flatten_state(state)

    def derivative(values: tuple[float, ...]) -> tuple[float, ...]:
        stage = _unflatten_state(values, state.buoyancy_memory)
        actuator_dot = tuple(
            constants.actuator_pole_radps
            * (actuator_command[i] - stage.actuator_outputs[i])
            for i in range(10)
        )
        rigid_dot = rigid_body_derivative(stage, constants)
        return rigid_dot + actuator_dot

    k1 = derivative(flat)
    k2 = derivative(_add_scaled(flat, k1, dt_s * 0.5))
    k3 = derivative(_add_scaled(flat, k2, dt_s * 0.5))
    k4 = derivative(_add_scaled(flat, k3, dt_s))
    next_flat = tuple(
        flat[i] + dt_s * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0
        for i in range(len(flat))
    )
    next_state = _unflatten_state(next_flat, state.buoyancy_memory)
    quaternion = normalize_quaternion(next_state.quaternion_wxyz)
    buoyancy, _ = buoyancy_wrench(
        quaternion, next_state.position_xyz_m[2], constants
    )
    return QnEquationState(
        body_twist=next_state.body_twist,
        quaternion_wxyz=quaternion,
        position_xyz_m=next_state.position_xyz_m,
        actuator_outputs=next_state.actuator_outputs,
        buoyancy_memory=buoyancy,
    )


def rigid_body_derivative(
    state: QnEquationState, constants: QnModelConstants
) -> tuple[float, ...]:
    """逐式对应 ``11/ODE/12ODE``，返回 13 个刚体状态导数。"""
    u, v, w, p, q, r = state.body_twist
    e0, e1, e2, e3 = state.quaternion_wxyz
    flag = medium_flag(state.position_xyz_m[2], constants.hg_m)
    mass, inertia = mass_and_inertia(flag, constants)
    actuation = actuator_wrench(state.actuator_outputs, flag, constants)
    damping = damping_wrench(state.body_twist, flag, constants)
    total = tuple(
        actuation[i] + damping[i] + state.buoyancy_memory[i] for i in range(6)
    )
    gravity: Vector6 = (
        mass * constants.gravity_mps2 * 2.0 * (e1 * e3 - e2 * e0),
        2.0 * mass * constants.gravity_mps2 * (e2 * e3 + e1 * e0),
        mass
        * constants.gravity_mps2
        * (e3 * e3 + e0 * e0 - e1 * e1 - e2 * e2),
        0.0,
        0.0,
        0.0,
    )
    omega = (p, q, r)
    linear_coriolis = tuple(mass * value for value in _cross(omega, (u, v, w)))
    angular_momentum = tuple(inertia[i] * omega[i] for i in range(3))
    angular_coriolis = _cross(omega, angular_momentum)
    linear_dot = tuple(
        (total[i] + gravity[i] - linear_coriolis[i]) / mass for i in range(3)
    )
    angular_dot = tuple(
        (total[i + 3] - angular_coriolis[i]) / inertia[i] for i in range(3)
    )
    dx = (
        u * (e1 * e1 + e0 * e0 - e2 * e2 - e3 * e3)
        + 2.0 * v * (e1 * e2 - e3 * e0)
        + 2.0 * w * (e1 * e3 + e2 * e0)
    )
    dy = (
        2.0 * u * (e1 * e2 + e3 * e0)
        + v * (e2 * e2 + e0 * e0 - e1 * e1 - e3 * e3)
        + 2.0 * w * (e2 * e3 - e1 * e0)
    )
    dh = (
        -2.0 * u * (e1 * e3 - e2 * e0)
        - 2.0 * v * (e2 * e3 + e1 * e0)
        - w * (e3 * e3 + e0 * e0 - e2 * e2 - e1 * e1)
    )
    quaternion_dot = (
        -0.5 * (p * e1 + q * e2 + r * e3),
        0.5 * (p * e0 + r * e2 - q * e3),
        0.5 * (q * e0 - r * e1 + p * e3),
        0.5 * (r * e0 + q * e1 - p * e2),
    )
    return linear_dot + angular_dot + quaternion_dot + (dx, dy, dh)


def qn_dcm(quaternion: Quaternion) -> tuple[Vector3, Vector3, Vector3]:
    """原 ``12ODE`` 输出的 DCM；保留其第三行符号，不替换成库函数。"""
    e0, e1, e2, e3 = quaternion
    return (
        (
            e1 * e1 + e0 * e0 - e2 * e2 - e3 * e3,
            2.0 * (e1 * e2 - e3 * e0),
            2.0 * (e1 * e3 + e2 * e0),
        ),
        (
            2.0 * (e1 * e2 + e3 * e0),
            e2 * e2 + e0 * e0 - e1 * e1 - e3 * e3,
            2.0 * (e2 * e3 - e1 * e0),
        ),
        (
            -2.0 * (e1 * e3 - e2 * e0),
            2.0 * (e2 * e3 + e1 * e0),
            e3 * e3 + e0 * e0 - e2 * e2 - e1 * e1,
        ),
    )


def body_to_map_velocity(body_velocity: Sequence[float], quaternion: Quaternion) -> Vector3:
    """使用 ``12ODE`` 的 dx/dy/dh 三式，不使用其符号不一致的 DCM 第三行。"""
    dummy = QnEquationState(
        body_twist=(body_velocity[0], body_velocity[1], body_velocity[2], 0.0, 0.0, 0.0),
        quaternion_wxyz=quaternion,
        position_xyz_m=(0.0, 0.0, 0.0),
        actuator_outputs=(0.0,) * 10,
        buoyancy_memory=(0.0,) * 6,
    )
    derivative = rigid_body_derivative_kinematics_only(dummy)
    return derivative


def rigid_body_derivative_kinematics_only(state: QnEquationState) -> Vector3:
    u, v, w = state.body_twist[:3]
    e0, e1, e2, e3 = state.quaternion_wxyz
    return (
        u * (e1 * e1 + e0 * e0 - e2 * e2 - e3 * e3)
        + 2.0 * v * (e1 * e2 - e3 * e0)
        + 2.0 * w * (e1 * e3 + e2 * e0),
        2.0 * u * (e1 * e2 + e3 * e0)
        + v * (e2 * e2 + e0 * e0 - e1 * e1 - e3 * e3)
        + 2.0 * w * (e2 * e3 - e1 * e0),
        -2.0 * u * (e1 * e3 - e2 * e0)
        - 2.0 * v * (e2 * e3 + e1 * e0)
        - w * (e3 * e3 + e0 * e0 - e2 * e2 - e1 * e1),
    )


def map_velocity_to_body(map_velocity: Vector3, quaternion: Quaternion) -> Vector3:
    """求解原位置运动学的 3x3 线性系统，避免假设第三轴符号。"""
    columns = tuple(
        body_to_map_velocity(axis, quaternion)
        for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    )
    matrix = tuple(tuple(columns[col][row] for col in range(3)) for row in range(3))
    return _solve_3x3(matrix, map_velocity)


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    magnitude = math.sqrt(sum(value * value for value in quaternion))
    if magnitude <= 1e-12:
        raise ValueError("qn quaternion norm must be non-zero")
    return tuple(float(value / magnitude) for value in quaternion)  # type: ignore[return-value]


def _read_actuator_command(command: ActuatorCmd | None) -> Vector10:
    if command is None:
        raise ValueError(
            "QN_SLX_6DOF_DYNAMICS requires an ActuatorCmd; desired_velocity cannot "
            "be connected directly to qn 12ODE"
        )
    if command.command_schema != QN_ACTUATOR_SCHEMA:
        raise ValueError(
            f"qn actuator schema must be {QN_ACTUATOR_SCHEMA!r}, got {command.command_schema!r}"
        )
    missing = [name for name in QN_ACTUATOR_FIELDS if name not in command.values]
    extra = [name for name in command.values if name not in QN_ACTUATOR_FIELDS]
    if missing or extra:
        raise ValueError(f"qn actuator fields mismatch: missing={missing}, extra={extra}")
    return tuple(float(command.values[name]) for name in QN_ACTUATOR_FIELDS)  # type: ignore[return-value]


def _flatten_state(state: QnEquationState) -> tuple[float, ...]:
    return (
        state.body_twist
        + state.quaternion_wxyz
        + state.position_xyz_m
        + state.actuator_outputs
    )


def _unflatten_state(values: Sequence[float], buoyancy_memory: Vector6) -> QnEquationState:
    return QnEquationState(
        body_twist=tuple(values[0:6]),  # type: ignore[arg-type]
        quaternion_wxyz=tuple(values[6:10]),  # type: ignore[arg-type]
        position_xyz_m=tuple(values[10:13]),  # type: ignore[arg-type]
        actuator_outputs=tuple(values[13:23]),  # type: ignore[arg-type]
        buoyancy_memory=buoyancy_memory,
    )


def _add_scaled(
    values: Sequence[float], derivative: Sequence[float], scale: float
) -> tuple[float, ...]:
    return tuple(values[i] + scale * derivative[i] for i in range(len(values)))


def _cross(left: Sequence[float], right: Sequence[float]) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _mat_vec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Vector3:
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _solve_3x3(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> Vector3:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant) <= 1e-12:
        raise ValueError("qn position kinematics matrix is singular")
    inverse = (
        ((e * i - f * h), (c * h - b * i), (b * f - c * e)),
        ((f * g - d * i), (a * i - c * g), (c * d - a * f)),
        ((d * h - e * g), (b * g - a * h), (a * e - b * d)),
    )
    return tuple(
        sum(inverse[row][col] * rhs[col] for col in range(3)) / determinant
        for row in range(3)
    )  # type: ignore[return-value]


def _norm3(vector: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))
