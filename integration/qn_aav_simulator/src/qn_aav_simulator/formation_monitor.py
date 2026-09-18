"""ROS-independent completion checks for the fixed seven-member AIR formation."""

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Mapping, Optional, Tuple


Vector3 = Tuple[float, float, float]
AGENT_IDS = tuple(range(7))
DEFAULT_RELATIVE_SLOTS = {
    0: (0.0, 0.0, 0.0), 1: (1.7321, -1.0, 0.0),
    2: (0.0, -2.0, 0.0), 3: (-1.7321, -1.0, 0.0),
    4: (-1.7321, 1.0, 0.0), 5: (0.0, 2.0, 0.0),
    6: (1.7321, 1.0, 0.0),
}


def _vector(value):
    if len(value) != 3:
        raise ValueError("positions and velocities must have three coordinates")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError("coordinates must be finite")
    return result


def validate_target(frame_id, center, hold_duration):
    center = _vector(center)
    if frame_id != "world" or abs(center[2] - 0.5) > 1e-6:
        raise ValueError("AIR targets require frame_id=world and z=0.5 m")
    if not math.isfinite(hold_duration) or hold_duration < 0.0:
        raise ValueError("hold_duration must be finite and nonnegative")
    return center


def validate_configuration(agent_ids, relative_slots, scale, epsilon_p,
                           epsilon_v, odom_timeout, execution_timeout):
    ids = tuple(agent_ids)
    if (len(ids) != 7 or set(ids) != set(AGENT_IDS)
            or any(type(agent_id) is not int for agent_id in ids)):
        raise ValueError("this action requires exactly agent_ids 0 through 6")
    slots = {int(key): _vector(value) for key, value in relative_slots.items()}
    if len(relative_slots) != 7 or set(slots) != set(ids):
        raise ValueError("relative_slots must contain exactly the seven agent IDs")
    if any(abs(slot[2]) > 1e-9 for slot in slots.values()):
        raise ValueError("all AIR slots must have zero relative height")
    for name, value in (("swarm_scale", scale), ("epsilon_p", epsilon_p),
                        ("epsilon_v", epsilon_v), ("odom_timeout", odom_timeout),
                        ("execution_timeout", execution_timeout)):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))
    return slots


@dataclass(frozen=True)
class OdometrySample:
    """One member state.

    ``velocity`` is always the world-frame linear velocity, whatever convention
    the source message used.  ``body_velocity`` keeps the body-frame value from
    the standard qn Odometry so consumers can tell the two conventions apart.
    """

    stamp: float
    position: Vector3
    velocity: Vector3
    frame_id: str = "world"
    body_velocity: Vector3 = (0.0, 0.0, 0.0)
    orientation_quat_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    child_frame_id: str = ""

    def is_fresh(self, now, timeout):
        return (self.frame_id == "world" and math.isfinite(self.stamp)
                and 0.0 <= now - self.stamp <= timeout
                and len(self.position) == 3 and len(self.velocity) == 3
                and all(math.isfinite(value)
                        for value in self.position + self.velocity))


@dataclass(frozen=True)
class MonitorSnapshot:
    phase: str
    max_position_error: Optional[float]
    max_velocity: Optional[float]
    min_inter_agent_distance: Optional[float]
    fresh_agent_count: int
    stale_agent_ids: Tuple[int, ...]
    hold_started: Optional[float]
    hold_elapsed: float
    terminal_state: Optional[str] = None
    reason: int = 0
    model_hold_elapsed_s: Optional[float] = None
    model_hold_pending: bool = False


class GroupCompletionMonitor:
    """Require a continuous observed window of fresh, settled member states.

    All times use one clock. A gap between evaluations longer than odom_timeout
    invalidates the hold window. Planner nominal finish is not an input.
    """

    INVALID_TARGET = 1
    ODOMETRY_TIMEOUT = 2
    EXECUTION_TIMEOUT = 3
    MODEL_HOLD_PENDING = 4

    def __init__(self, center, hold_duration, start_time, *, agent_ids=AGENT_IDS,
                 relative_slots=None, swarm_scale=2.0, epsilon_p=0.5,
                 epsilon_v=0.25, odom_timeout=0.25, execution_timeout=180.0):
        self.center = validate_target("world", center, hold_duration)
        self.slots = validate_configuration(
            agent_ids, DEFAULT_RELATIVE_SLOTS if relative_slots is None else relative_slots,
            swarm_scale, epsilon_p, epsilon_v, odom_timeout, execution_timeout)
        if not math.isfinite(start_time):
            raise ValueError("start_time must be finite")
        self.agent_ids = tuple(agent_ids)
        self.targets = {
            agent_id: tuple(self.center[axis] + swarm_scale * self.slots[agent_id][axis]
                            for axis in range(3))
            for agent_id in self.agent_ids
        }
        self.hold_duration = hold_duration
        self.start_time = start_time
        self.epsilon_p = epsilon_p
        self.epsilon_v = epsilon_v
        self.odom_timeout = odom_timeout
        self.execution_timeout = execution_timeout
        self.hold_started = None
        self.last_evaluation = None
        self.snapshot = None

    def _fresh_window(self, value, now):
        """Fresh samples for one member, newest last."""
        window = [value] if isinstance(value, OdometrySample) else list(value)
        return [sample for sample in window
                if sample.is_fresh(now, self.odom_timeout)]

    def evaluate(self, now: float, samples: Mapping[int, OdometrySample], *,
                 cancelled: bool = False,
                 model_hold_satisfied=None) -> MonitorSnapshot:
        """One completion evaluation.

        ``model_hold_satisfied(hold_start_ros, now_ros)`` optionally gates the
        SUCCEEDED transition on the qn *model* time actually advanced during the
        hold window.  ROS dwell time alone never completes a task.
        """
        if self.snapshot is not None and self.snapshot.terminal_state is not None:
            return self.snapshot
        # ``samples`` is either the latest state per member or every state
        # received since the previous evaluation.  The settled condition is
        # evaluated over the whole fresh window: "the seven dynamics states
        # continuously satisfy the completion conditions" cannot be verified
        # from one sample per 50 ms monitor tick when odometry arrives at
        # 100 Hz.
        fresh = {}
        for agent_id in self.agent_ids:
            value = samples.get(agent_id) if agent_id in samples else None
            if value is None:
                continue
            window = self._fresh_window(value, now)
            if window:
                fresh[agent_id] = window
        stale = tuple(agent_id for agent_id in self.agent_ids if agent_id not in fresh)
        position_errors = [math.dist(sample.position, self.targets[agent_id])
                           for agent_id, window in fresh.items()
                           for sample in window]
        speeds = [math.sqrt(sum(value * value for value in sample.velocity))
                  for window in fresh.values() for sample in window]
        latest = [window[-1] for window in fresh.values()]
        distances = [math.dist(first.position, second.position)
                     for first, second in combinations(latest, 2)]
        max_error = max(position_errors, default=None)
        max_speed = max(speeds, default=None)
        min_distance = min(distances, default=None)
        terminal = None
        reason = 0
        if cancelled:
            terminal = "PREEMPTED"
        elif (not math.isfinite(now) or now < self.start_time
              or (self.last_evaluation is not None and now < self.last_evaluation)
              or now - self.start_time >= self.execution_timeout):
            terminal, reason = "ABORTED", self.EXECUTION_TIMEOUT
        elif stale:
            terminal, reason = "ABORTED", self.ODOMETRY_TIMEOUT
        if (terminal is not None or self.last_evaluation is not None
                and now - self.last_evaluation > self.odom_timeout):
            self.hold_started = None
        settled = (terminal is None and not stale
                   and max_error <= self.epsilon_p and max_speed <= self.epsilon_v)
        model_hold_elapsed = None
        model_hold_pending = False
        if settled:
            if self.hold_started is None:
                self.hold_started = now
            if now - self.hold_started >= self.hold_duration:
                if model_hold_satisfied is None:
                    terminal = "SUCCEEDED"
                else:
                    model_hold_elapsed = model_hold_satisfied(self.hold_started, now)
                    if model_hold_elapsed is None:
                        model_hold_pending = True
                        reason = self.MODEL_HOLD_PENDING
                    elif model_hold_elapsed + 1e-9 < self.hold_duration:
                        model_hold_pending = True
                        reason = self.MODEL_HOLD_PENDING
                    else:
                        terminal = "SUCCEEDED"
        else:
            self.hold_started = None
        self.last_evaluation = now
        self.snapshot = MonitorSnapshot(
            phase="HOLDING" if settled else "MOVING",
            max_position_error=max_error, max_velocity=max_speed,
            min_inter_agent_distance=min_distance, fresh_agent_count=len(fresh),
            stale_agent_ids=stale, hold_started=self.hold_started,
            hold_elapsed=0.0 if self.hold_started is None else now - self.hold_started,
            terminal_state=terminal, reason=reason,
            model_hold_elapsed_s=model_hold_elapsed,
            model_hold_pending=model_hold_pending)
        return self.snapshot
