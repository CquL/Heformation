"""Fixed resource state machine for the FormationAction server.

    BOOTING -> READY_IDLE -> ACTIVE -> HOLDING -> SUCCEEDED -> READY_IDLE
                                     \\-> UNKNOWN_LOCKED

Original P1 defaults, with an opt-in local safety disposition:

* the goal callback only validates, reserves and hands over; it never runs the
  task loop,
* the idle check and the resource reservation are one atomic operation,
* a running task refuses new goals,
* cancellation never releases resources or dispatches the next task. The
  original path only records cancellation. With ``stop_and_lock=True`` the
  Action worker must observe the local reference takeover before terminating,
* odometry loss, execution timeout and result timeout all enter
  ``UNKNOWN_LOCKED``, which can only be left by restarting the whole execution
  chain (old trajectory servers, qn nodes and this Action server included).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

BOOTING = "BOOTING"
READY_IDLE = "READY_IDLE"
ACTIVE = "ACTIVE"
HOLDING = "HOLDING"
SUCCEEDED = "SUCCEEDED"
UNKNOWN_LOCKED = "UNKNOWN_LOCKED"

MOVING = "MOVING"

ACCEPTED = "ACCEPTED"
REJECTED_BUSY = "REJECTED_BUSY"
REJECTED_NOT_READY = "REJECTED_NOT_READY"
REJECTED_INVALID = "REJECTED_INVALID"
REJECTED_LOCKED = "REJECTED_LOCKED"

_ACCEPTING_STATES = (READY_IDLE,)
_RESERVING_STATES = (ACTIVE, HOLDING)
_LOCKED_STATES = (UNKNOWN_LOCKED,)


@dataclass
class LifecycleEvent:
    ros_time_s: float
    monotonic_s: float
    state: str
    event: str
    detail: str = ""

    def as_dict(self) -> Dict[str, object]:
        return {
            "ros_time_s": self.ros_time_s,
            "monotonic_s": self.monotonic_s,
            "state": self.state,
            "event": self.event,
            "detail": self.detail,
        }


class ActionResourceStateMachine:
    """Resource lifecycle for one configured Action executor."""

    def __init__(self, clock: Optional[Callable[[], float]] = None,
                 lock: Optional[threading.RLock] = None) -> None:
        self._lock = lock if lock is not None else threading.RLock()
        self._clock = clock if clock is not None else time.monotonic
        self._state = BOOTING
        self._goal_id: Optional[str] = None
        self._execution_id: Optional[str] = None
        self._reserved_at_s: Optional[float] = None
        self._cancel_requested = False
        self._cancel_ignored = False
        self.events: List[LifecycleEvent] = []
        self.ros_time_s = 0.0
        self.unknown_locked_reason: Optional[str] = None

    # -- introspection -----------------------------------------------------
    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def goal_id(self) -> Optional[str]:
        with self._lock:
            return self._goal_id

    @property
    def execution_id(self) -> Optional[str]:
        with self._lock:
            return self._execution_id

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def resources_released(self) -> bool:
        with self._lock:
            return self._state in (READY_IDLE,) and self._goal_id is None

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {
                "state": self._state,
                "goal_id": self._goal_id,
                "execution_id": self._execution_id,
                "reserved_at_s": self._reserved_at_s,
                "cancel_requested": self._cancel_requested,
                "cancel_ignored_while_running": self._cancel_ignored,
                "unknown_locked_reason": self.unknown_locked_reason,
                "events": [event.as_dict() for event in self.events],
            }

    # -- internal ----------------------------------------------------------
    def _record(self, event: str, detail: str = "") -> None:
        self.events.append(LifecycleEvent(
            ros_time_s=self.ros_time_s, monotonic_s=self._clock(),
            state=self._state, event=event, detail=detail))

    def _set_state(self, state: str, event: str, detail: str = "") -> None:
        self._state = state
        self._record(event, detail)

    # -- lifecycle ---------------------------------------------------------
    def note_readiness(self, ready: bool) -> str:
        with self._lock:
            if self._state == BOOTING and ready:
                self._set_state(READY_IDLE, "readiness_satisfied")
            elif self._state == READY_IDLE and not ready:
                self._set_state(BOOTING, "readiness_lost")
            return self._state

    def request_goal(self, goal_id: str, *, validation_error: Optional[str] = None,
                     execution_id: Optional[str] = None
                     ) -> Tuple[str, str, Optional[str]]:
        """Validate + check idle + reserve the group resource atomically."""
        with self._lock:
            if validation_error:
                self._record("goal_rejected_invalid", validation_error)
                return REJECTED_INVALID, validation_error, self._goal_id
            if self._state in _LOCKED_STATES:
                reason = "resources are UNKNOWN_LOCKED: {}".format(
                    self.unknown_locked_reason)
                self._record("goal_rejected_locked", reason)
                return REJECTED_LOCKED, reason, self._goal_id
            if self._state in _RESERVING_STATES:
                reason = "a formation task is already {}".format(self._state)
                self._record("goal_rejected_busy", reason)
                return REJECTED_BUSY, reason, self._goal_id
            if self._state != READY_IDLE:
                reason = "server is {}, not READY_IDLE".format(self._state)
                self._record("goal_rejected_not_ready", reason)
                return REJECTED_NOT_READY, reason, self._goal_id
            if not goal_id:
                reason = "goal_id must not be empty"
                self._record("goal_rejected_invalid", reason)
                return REJECTED_INVALID, reason, self._goal_id
            self._goal_id = str(goal_id)
            self._execution_id = execution_id
            self._reserved_at_s = self._clock()
            self._cancel_requested = False
            self._cancel_ignored = False
            self._set_state(ACTIVE, "resource_reserved", str(goal_id))
            return ACCEPTED, "reserved", self._goal_id

    def note_phase(self, phase: str) -> str:
        with self._lock:
            if self._state in (ACTIVE, HOLDING):
                if phase == "HOLDING" and self._state == ACTIVE:
                    self._set_state(HOLDING, "hold_started")
                elif phase != "HOLDING" and self._state == HOLDING:
                    self._set_state(ACTIVE, "hold_interrupted")
            return self._state

    def note_cancel_request(self, goal_id=None, *, stop_and_lock=False) -> bool:
        """Record legacy cancellation, or lock the matching Goal for disposition."""
        with self._lock:
            if stop_and_lock:
                if goal_id != self._goal_id or self._state not in (ACTIVE, HOLDING, UNKNOWN_LOCKED):
                    return False
                if not self._cancel_requested:
                    self._cancel_requested = True
                    self._cancel_ignored = False
                    self.unknown_locked_reason = "cancel requested; awaiting local safety disposition"
                    self._set_state(UNKNOWN_LOCKED, "cancel_requires_hold", str(goal_id))
                return True
            if self._state in _RESERVING_STATES:
                self._cancel_requested = True
                self._cancel_ignored = True
                self._record("cancel_ignored_while_running")
                return False
            self._cancel_requested = True
            self._record("cancel_recorded")
            return True

    def finish_success(self) -> str:
        with self._lock:
            if self._state != HOLDING:
                raise ValueError("success requires a completed HOLDING phase, state={}".format(
                    self._state))
            self._set_state(SUCCEEDED, "hold_completed")
            return self._state

    def release(self) -> str:
        """Release the group resource; only valid after SUCCEEDED."""
        with self._lock:
            if self._state != SUCCEEDED:
                raise ValueError("resources may only be released after SUCCEEDED")
            self._goal_id = None
            self._execution_id = None
            self._reserved_at_s = None
            self._cancel_requested = False
            self._set_state(READY_IDLE, "resource_released")
            return self._state

    def fault(self, reason: str) -> str:
        with self._lock:
            self.unknown_locked_reason = str(reason)
            self._set_state(UNKNOWN_LOCKED, "fault", str(reason))
            return self._state

    def restart_chain(self) -> str:
        """Recovery requires restarting the whole chain, not clearing a lock."""
        with self._lock:
            if self._state != UNKNOWN_LOCKED:
                raise ValueError("restart_chain is only valid from UNKNOWN_LOCKED")
            self.unknown_locked_reason = None
            self._goal_id = None
            self._execution_id = None
            self._reserved_at_s = None
            self._cancel_requested = False
            self._cancel_ignored = False
            self._set_state(BOOTING, "full_chain_restart")
            return self._state
