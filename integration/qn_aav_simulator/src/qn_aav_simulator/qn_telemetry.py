"""Reference-usage and model-time bookkeeping for the qn outer control loop.

ROS-free helpers that record the immutable per-outer-step command snapshot and
the outer/integration step bookkeeping required by plan.md P0.2.  They do not
touch the qn controller, the actuator loop or the 6DOF dynamics.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

Vector3 = Tuple[float, float, float]

# PositionCommand.trajectory_flag values used by the Swarm traj_server.
TRAJECTORY_STATUS_READY = 1


def vector3(value) -> Vector3:
    result = tuple(float(component) for component in value)
    if len(result) != 3:
        raise ValueError("expected three coordinates")
    if not all(math.isfinite(component) for component in result):
        raise ValueError("coordinates must be finite")
    return result


@dataclass(frozen=True)
class CommandSnapshot:
    """One immutable PositionCommand frozen before ``backend.step()``.

    The control loop must build this object first and then never re-read the
    latest-callback buffer, so that the reference reported as used is exactly
    the reference that entered the integrator.
    """

    agent_id: str
    outer_step_index: int
    received_ros_time_s: float
    stamp_s: float
    trajectory_id: int
    trajectory_flag: int
    position: Vector3
    velocity: Vector3
    acceleration: Vector3
    yaw_rad: float

    def __post_init__(self):
        if not self.agent_id:
            raise ValueError("agent_id must not be empty")
        if type(self.trajectory_id) is not int or self.trajectory_id < 0:
            raise ValueError("trajectory_id must be a nonnegative int")
        for name in ("received_ros_time_s", "stamp_s", "yaw_rad"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        vector3(self.position)
        vector3(self.velocity)
        vector3(self.acceleration)


@dataclass(frozen=True)
class ReferenceUsage:
    """Reference actually handed to the integrator for one outer step."""

    agent_id: str
    used_outer_step: int
    source_trajectory_id: int
    source_command_stamp: float
    outer_dt_s: float
    model_interval_start_s: float
    model_interval_end_s: float
    position: Vector3
    velocity: Vector3
    velocity_was_derived: bool

    @property
    def model_interval_s(self) -> float:
        return self.model_interval_end_s - self.model_interval_start_s


class ReferenceUsageTracker:
    """Reconstruct ``v_used,k = (p_used,k - p_used,k-1) / outer_dt_s``.

    ``p_used,k`` is the position of the frozen snapshot, i.e. the reference the
    qn backend receives in ``ROUTE_POSITION`` mode.  The outer step is never
    mixed with the qn internal integration sub-step.
    """

    def __init__(self, agent_id: str) -> None:
        if not agent_id:
            raise ValueError("agent_id must not be empty")
        self.agent_id = agent_id
        self._previous_position: Optional[Vector3] = None
        self._next_step = 0

    @property
    def used_outer_step_count(self) -> int:
        return self._next_step

    def reset(self, position) -> None:
        self._previous_position = vector3(position)
        self._next_step = 0

    def record(self, snapshot: CommandSnapshot, outer_dt_s: float,
               model_interval_start_s: float) -> ReferenceUsage:
        if snapshot.agent_id != self.agent_id:
            raise ValueError("snapshot belongs to another agent")
        if not math.isfinite(outer_dt_s) or outer_dt_s <= 0.0:
            raise ValueError("outer_dt_s must be finite and positive")
        if snapshot.outer_step_index != self._next_step:
            raise ValueError("command snapshot is stale or out of order")
        position = vector3(snapshot.position)
        derived = self._previous_position is not None
        if derived:
            velocity = tuple(
                (position[index] - self._previous_position[index]) / outer_dt_s
                for index in range(3)
            )
        else:
            # No reset reference exists, so no derivative is defined.  The value
            # is left at zero rather than copied from a different (sub-step)
            # interval; a control loop is expected to reset before stepping.
            velocity = (0.0, 0.0, 0.0)
        start = float(model_interval_start_s)
        usage = ReferenceUsage(
            agent_id=self.agent_id,
            used_outer_step=self._next_step,
            source_trajectory_id=snapshot.trajectory_id,
            source_command_stamp=snapshot.stamp_s,
            outer_dt_s=float(outer_dt_s),
            model_interval_start_s=start,
            model_interval_end_s=start + float(outer_dt_s),
            position=position,
            velocity=velocity,
            velocity_was_derived=derived,
        )
        self._previous_position = position
        self._next_step += 1
        return usage


class ModelClock:
    """Accumulated model time and outer/integration step counters.

    ``model_time_s`` only advances by completed outer steps of ``backend.step()``;
    it is never derived from ROS wall time.
    """

    def __init__(self) -> None:
        self.model_time_s = 0.0
        self.outer_step_count = 0
        self.integration_step_count = 0
        self.integration_step_s: Optional[float] = None
        self.outer_dt_s: Optional[float] = None

    def advance(self, outer_dt_s: float, integration_step_s: float, substeps: int) -> float:
        if not math.isfinite(outer_dt_s) or outer_dt_s <= 0.0:
            raise ValueError("outer_dt_s must be finite and positive")
        if not math.isfinite(integration_step_s) or integration_step_s <= 0.0:
            raise ValueError("integration_step_s must be finite and positive")
        if int(substeps) < 1 or int(substeps) != float(substeps):
            raise ValueError("substeps must be a positive integer")
        if not math.isclose(outer_dt_s, integration_step_s * float(substeps),
                            rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("outer step must be an integer number of integration sub-steps")
        self.outer_dt_s = float(outer_dt_s)
        self.integration_step_s = float(integration_step_s)
        self.outer_step_count += 1
        self.integration_step_count += int(substeps)
        self.model_time_s += float(outer_dt_s)
        return self.model_time_s


def command_flag_name(flag: int) -> str:
    names = {0: "EMPTY", 1: "READY", 3: "COMPLETED", 4: "ABORT"}
    return names.get(int(flag), "UNKNOWN_{}".format(int(flag)))


class CommandAdoptionBuffer:
    """Adopt one command per fixed outer step, never replaying history.

    The outer step is a fixed model interval, so a command is adopted at a step
    boundary and held for that whole interval.  A command that arrives later is
    never applied to an interval that has already been integrated, and nothing
    is recomputed after a stall: the model simply falls behind ROS time and the
    alignment gate reports it.

    The cache is bounded.  A burst that does not fit, or a command that arrives
    out of order, is counted as a violation so the experiment can be marked
    invalid instead of silently losing the input.
    """

    def __init__(self, maxlen: int = 32) -> None:
        if int(maxlen) < 1:
            raise ValueError("maxlen must be a positive integer")
        self.maxlen = int(maxlen)
        self._buffer = deque(maxlen=self.maxlen)
        self._queued: Optional[Dict[str, object]] = None
        self.dropped_count = 0
        self.order_violations = 0
        self.adopted_count = 0
        self.last_adopted_ros_time_s: Optional[float] = None
        self.last_adopted_command: Optional[Dict[str, object]] = None

    def note(self, fields: Dict[str, object]) -> None:
        arrived = float(fields["received_ros_time_s"])
        if not math.isfinite(arrived):
            raise ValueError("command arrival time must be finite")
        if len(self._buffer) == self.maxlen:
            self.dropped_count += 1
        if (self.last_adopted_ros_time_s is not None
                and arrived < self.last_adopted_ros_time_s):
            self.order_violations += 1
        self._buffer.append(fields)
        self._queued = fields

    def adopt(self) -> Optional[Dict[str, object]]:
        """Take the queued command for the next step, or None to hold."""
        adopted = self._queued
        self._queued = None
        if adopted is None:
            return None
        self.adopted_count += 1
        self.last_adopted_ros_time_s = float(adopted["received_ros_time_s"])
        self.last_adopted_command = adopted
        return adopted

    @property
    def buffered(self) -> int:
        return len(self._buffer)

    @property
    def violated(self) -> bool:
        return bool(self.dropped_count or self.order_violations)
