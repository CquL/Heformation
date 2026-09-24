"""Only the data contracts required by the mechanically migrated qn core."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Tuple


def canonical_model_state_bytes(value):
    """Stable value encoding for trusted native-state equality, not a wire API.

    Object pickle memoization depends on aliasing, so two equal model states
    can have different pickle bytes. Preserve every array bit and container
    type while removing that aliasing difference before hashing a local state.
    """
    import json
    import numpy as np
    def stable(item):
        if isinstance(item,np.ndarray):
            return ['ndarray',item.dtype.str,list(item.shape),item.tobytes(order='C').hex()]
        if isinstance(item,np.generic):
            return ['numpy_scalar',item.dtype.str,item.tobytes().hex()]
        if isinstance(item,dict):
            return ['dict',[[stable(key),stable(item[key])] for key in sorted(item)]]
        if isinstance(item,tuple):return ['tuple',[stable(entry) for entry in item]]
        if isinstance(item,list):return ['list',[stable(entry) for entry in item]]
        if isinstance(item,bytes):return ['bytes',item.hex()]
        if isinstance(item,(str,int,float,bool,type(None))):return item
        if hasattr(item,'__dict__'):
            return ['object',item.__class__.__module__,item.__class__.__qualname__,stable(vars(item))]
        raise TypeError('native state contains an unsupported value: '+type(item).__name__)
    return json.dumps(stable(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


class CommandMode(str, Enum):
    DESIRED_VELOCITY = "DESIRED_VELOCITY"
    DESIRED_POSITION = "DESIRED_POSITION"


@dataclass(frozen=True)
class AgentState:
    agent_id: str
    type: str
    timestamp_s: float
    position: Vector3
    velocity: Vector3
    energy: float = 1.0
    status: str = "ACTIVE"
    current_task_id: str | None = None
    acceleration: Vector3 = (0.0, 0.0, 0.0)
    orientation_quat_wxyz: Quaternion = (1.0, 0.0, 0.0, 0.0)
    body_linear_velocity_mps: Vector3 = (0.0, 0.0, 0.0)
    body_angular_velocity_radps: Vector3 = (0.0, 0.0, 0.0)
    medium_flag: float | None = None


@dataclass(frozen=True)
class ControlCmd:
    cmd_id: str
    agent_id: str
    timestamp_s: float
    command_mode: CommandMode
    desired_velocity: Vector3
    desired_position: Vector3 | None = None
    desired_acceleration: Vector3 = (0.0, 0.0, 0.0)
    desired_jerk: Vector3 = (0.0, 0.0, 0.0)
    desired_yaw_rad: float = 0.0
    desired_attitude_quat_wxyz: Quaternion = (1.0, 0.0, 0.0, 0.0)
    frame_id: str = "map"


@dataclass(frozen=True)
class PlatformAdapterCmd:
    agent_id: str


@dataclass(frozen=True)
class ActuatorCmd:
    agent_id: str
    command_schema: str
    values: Mapping[str, float]


@dataclass(frozen=True)
class PlantStepInput:
    state: AgentState
    control_cmd: ControlCmd
    platform_adapter: PlatformAdapterCmd
    dt_s: float
    max_speed_mps: float
    max_acc_mps2: float
    actuator_command: ActuatorCmd | None = None
    environment: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PlantStepResult:
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    velocity_tracking_error_mps: float
    acceleration_limited: bool
    speed_limited: bool
    backend_id: str = "UNSPECIFIED"
    orientation_quat_wxyz: Quaternion = (1.0, 0.0, 0.0, 0.0)
    body_linear_velocity_mps: Vector3 = (0.0, 0.0, 0.0)
    body_angular_velocity_radps: Vector3 = (0.0, 0.0, 0.0)
    medium_flag: float | None = None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
