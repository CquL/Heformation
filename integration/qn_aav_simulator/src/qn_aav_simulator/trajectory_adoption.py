"""Trajectory-ownership evidence for "the new task reference was adopted".

``PositionCommand.stamp > goal_dispatch_time`` is *not* evidence: the Swarm
traj_server keeps publishing endpoint-hold commands at 100 Hz with a refreshed
stamp while the trajectory id stays unchanged.  Adoption is therefore decided
by trajectory ownership:

    execution_id -> Action GoalID -> dispatch time
                 -> per-member trajectory_id before/after dispatch
                 -> qn ``source_trajectory_id`` and ``used_outer_step``

A new trajectory id is *not* by itself a new task: local replanning during a
mission keeps the same execution context and may introduce further ids.  When
ownership cannot be established the verdict is
``REFERENCE_ADOPTION_UNCONFIRMED``; there is no timestamp-update fallback.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

ADOPTED = "ADOPTED"
REFERENCE_ADOPTION_UNCONFIRMED = "REFERENCE_ADOPTION_UNCONFIRMED"

# In this workspace the FormationAction server is the only authorized
# publisher of the group-level goal (P0.3 rule 2).  The set is supplied by the
# caller so the tracker stays free of ROS imports.
AUTHORIZED_GROUP_GOAL_PUBLISHERS = ()


@dataclass
class AgentAdoptionEvidence:
    agent_id: int
    pre_dispatch_trajectory_id: Optional[int] = None
    pre_dispatch_qn_source_trajectory_id: Optional[int] = None
    pre_dispatch_used_outer_step: Optional[int] = None
    post_dispatch_trajectory_ids: List[int] = field(default_factory=list)
    qn_source_trajectory_ids: List[int] = field(default_factory=list)
    adopted_trajectory_id: Optional[int] = None
    adopted_used_outer_step: Optional[int] = None
    adopted_ros_time_s: Optional[float] = None
    adopted_command_stamp_s: Optional[float] = None
    stamp_only_evidence_seen: bool = False
    failure_reason: str = ""

    @property
    def prior_trajectory_id(self) -> Optional[int]:
        """Reference in force before this dispatch, if any.

        On the first dispatch of a run no PositionCommand has ever been
        published, but the qn is already tracking its synthetic hold reference
        and reporting it as ``source_trajectory_id``.  That observable id is the
        prior reference, not "unknown".
        """
        if self.pre_dispatch_trajectory_id is not None:
            return self.pre_dispatch_trajectory_id
        return self.pre_dispatch_qn_source_trajectory_id

    @property
    def new_trajectory_observed(self) -> bool:
        return any(trajectory_id != self.prior_trajectory_id
                   for trajectory_id in self.post_dispatch_trajectory_ids)

    @property
    def qn_source_matches(self) -> bool:
        """The qn must consume a trajectory actually dispatched for this task.

        Matching against everything the qn has ever reported would be vacuous;
        the adopted id has to be one of the post-dispatch command trajectories.
        """
        if self.adopted_trajectory_id is None:
            return False
        return self.adopted_trajectory_id in self.post_dispatch_trajectory_ids

    def as_dict(self) -> Dict[str, object]:
        return {
            "agent_id": self.agent_id,
            "pre_dispatch_trajectory_id": self.pre_dispatch_trajectory_id,
            "pre_dispatch_qn_source_trajectory_id":
                self.pre_dispatch_qn_source_trajectory_id,
            "pre_dispatch_used_outer_step": self.pre_dispatch_used_outer_step,
            "post_dispatch_trajectory_ids": list(self.post_dispatch_trajectory_ids),
            "qn_source_trajectory_ids": list(self.qn_source_trajectory_ids),
            "adopted_trajectory_id": self.adopted_trajectory_id,
            "adopted_used_outer_step": self.adopted_used_outer_step,
            "adopted_ros_time_s": self.adopted_ros_time_s,
            "adopted_command_stamp_s": self.adopted_command_stamp_s,
            "new_trajectory_observed": self.new_trajectory_observed,
            "qn_source_matches": self.qn_source_matches,
            "stamp_only_evidence_seen": self.stamp_only_evidence_seen,
            "failure_reason": self.failure_reason,
        }


@dataclass
class AdoptionVerdict:
    state: str
    execution_id: Optional[str]
    goal_id: Optional[str]
    dispatch_ros_time_s: Optional[float]
    group_goal_publish_count: int
    group_goal_publisher: Optional[str]
    group_goal_authorized: bool
    per_agent: Tuple[AgentAdoptionEvidence, ...]
    failures: Tuple[str, ...]
    fallback_used: bool = False

    @property
    def adopted(self) -> bool:
        return self.state == ADOPTED

    def as_dict(self) -> Dict[str, object]:
        return {
            "state": self.state,
            "execution_id": self.execution_id,
            "goal_id": self.goal_id,
            "dispatch_ros_time_s": self.dispatch_ros_time_s,
            "group_goal_publish_count": self.group_goal_publish_count,
            "group_goal_publisher": self.group_goal_publisher,
            "group_goal_authorized": self.group_goal_authorized,
            "per_agent": [evidence.as_dict() for evidence in self.per_agent],
            "failures": list(self.failures),
            "fallback_used": self.fallback_used,
        }


class TrajectoryAdoptionTracker:
    """Collect ownership evidence for one dispatch."""

    def __init__(self, agent_ids: Iterable[int], *,
                 authorized_goal_publishers: Iterable[str] = ()) -> None:
        self.agent_ids = tuple(int(agent_id) for agent_id in agent_ids)
        self.authorized_goal_publishers = tuple(authorized_goal_publishers)
        if not self.agent_ids:
            raise ValueError("agent_ids must not be empty")
        self.evidence: Dict[int, AgentAdoptionEvidence] = {
            agent_id: AgentAdoptionEvidence(agent_id) for agent_id in self.agent_ids}
        self.execution_id: Optional[str] = None
        self.goal_id: Optional[str] = None
        self.dispatch_ros_time_s: Optional[float] = None
        self.group_goal_publish_count = 0
        self.group_goal_publisher: Optional[str] = None
        self._last_seen: Dict[int, Tuple[Optional[int], Optional[int]]] = {}

    # -- observation -------------------------------------------------------
    def note_position_command(self, agent_id: int, trajectory_id: int,
                              stamp_s: float, ros_time_s: float) -> None:
        if agent_id not in self.evidence:
            raise ValueError("unknown agent {}".format(agent_id))
        if type(trajectory_id) is not int or trajectory_id < 0:
            raise ValueError("trajectory_id must be a nonnegative int")
        evidence = self.evidence[agent_id]
        if self.dispatch_ros_time_s is None:
            evidence.pre_dispatch_trajectory_id = trajectory_id
            self._last_seen[agent_id] = (trajectory_id, None)
        else:
            if trajectory_id not in evidence.post_dispatch_trajectory_ids:
                evidence.post_dispatch_trajectory_ids.append(trajectory_id)
            if stamp_s > self.dispatch_ros_time_s and trajectory_id == \
                    evidence.pre_dispatch_trajectory_id:
                # Endpoint-hold traffic: a fresh stamp on the old trajectory.
                evidence.stamp_only_evidence_seen = True

    def note_qn_source(self, agent_id: int, source_trajectory_id: int,
                       used_outer_step: int, ros_time_s: float,
                       source_command_stamp_s: float) -> None:
        if agent_id not in self.evidence:
            raise ValueError("unknown agent {}".format(agent_id))
        if type(source_trajectory_id) is not int or source_trajectory_id < 0:
            raise ValueError("source_trajectory_id must be a nonnegative int")
        if type(used_outer_step) is not int or used_outer_step < 0:
            raise ValueError("used_outer_step must be a nonnegative int")
        evidence = self.evidence[agent_id]
        if source_trajectory_id not in evidence.qn_source_trajectory_ids:
            evidence.qn_source_trajectory_ids.append(source_trajectory_id)
        if self.dispatch_ros_time_s is None:
            evidence.pre_dispatch_used_outer_step = used_outer_step
            if evidence.pre_dispatch_qn_source_trajectory_id is None:
                evidence.pre_dispatch_qn_source_trajectory_id = source_trajectory_id
            self._last_seen[agent_id] = (
                self._last_seen.get(agent_id, (None, None))[0], used_outer_step)
            return
        pre_step = evidence.pre_dispatch_used_outer_step
        prior_id = evidence.prior_trajectory_id
        already_adopted = evidence.adopted_trajectory_id is not None
        if already_adopted:
            return
        is_new = source_trajectory_id != prior_id
        owned = source_trajectory_id in evidence.post_dispatch_trajectory_ids
        advanced = pre_step is None or used_outer_step > pre_step
        if is_new and owned and advanced:
            evidence.adopted_trajectory_id = source_trajectory_id
            evidence.adopted_used_outer_step = used_outer_step
            evidence.adopted_ros_time_s = float(ros_time_s)
            evidence.adopted_command_stamp_s = float(source_command_stamp_s)

    def adopted_at_s(self) -> Optional[float]:
        """Latest adoption time over all members, or None if not yet adopted.

        The hold window of the dispatched task may not start before this moment:
        a member that already sits on the previous target must not bank dwell
        time for the new reference before it actually uses it.
        """
        times = []
        for agent_id in self.agent_ids:
            evidence = self.evidence.get(agent_id)
            if evidence is None or evidence.adopted_ros_time_s is None:
                return None
            times.append(float(evidence.adopted_ros_time_s))
        return max(times) if times else None

    def note_group_goal(self, publisher: Optional[str] = None) -> None:
        self.group_goal_publish_count += 1
        if publisher is not None:
            self.group_goal_publisher = str(publisher)

    # -- dispatch boundary -------------------------------------------------
    def begin_dispatch(self, execution_id: str, goal_id: str,
                       dispatch_ros_time_s: float) -> None:
        if not execution_id or not goal_id:
            raise ValueError("execution_id and goal_id must not be empty")
        if not math.isfinite(dispatch_ros_time_s):
            raise ValueError("dispatch time must be finite")
        self.execution_id = str(execution_id)
        self.goal_id = str(goal_id)
        self.dispatch_ros_time_s = float(dispatch_ros_time_s)

    # -- verdict -----------------------------------------------------------
    def verdict(self) -> AdoptionVerdict:
        failures: List[str] = []
        if self.dispatch_ros_time_s is None:
            failures.append("dispatch boundary was never recorded")
        if self.group_goal_publish_count != 1:
            failures.append("group goal was published {} times".format(
                self.group_goal_publish_count))
        authorized = (not self.authorized_goal_publishers
                      or self.group_goal_publisher in self.authorized_goal_publishers)
        if not authorized:
            failures.append(
                "group goal publisher {} is not authorized ({})".format(
                    self.group_goal_publisher,
                    ", ".join(self.authorized_goal_publishers)))
        for agent_id in self.agent_ids:
            evidence = self.evidence[agent_id]
            evidence.failure_reason = ""
            if evidence.prior_trajectory_id is None:
                evidence.failure_reason = (
                    "no pre-dispatch trajectory id or qn source observed")
            elif not evidence.new_trajectory_observed:
                evidence.failure_reason = "no new trajectory id after dispatch"
            elif evidence.adopted_trajectory_id is None:
                evidence.failure_reason = (
                    "qn never reported a newly owned source_trajectory_id")
            elif not evidence.qn_source_matches:
                evidence.failure_reason = (
                    "qn source is not a trajectory dispatched for this execution")
            if evidence.pre_dispatch_used_outer_step is None:
                evidence.failure_reason = (
                    evidence.failure_reason
                    or "no pre-dispatch used_outer_step observed")
            if evidence.failure_reason:
                failures.append("drone_{}: {}".format(agent_id, evidence.failure_reason))
        state = ADOPTED if not failures else REFERENCE_ADOPTION_UNCONFIRMED
        return AdoptionVerdict(
            state=state,
            execution_id=self.execution_id,
            goal_id=self.goal_id,
            dispatch_ros_time_s=self.dispatch_ros_time_s,
            group_goal_publish_count=self.group_goal_publish_count,
            group_goal_publisher=self.group_goal_publisher,
            group_goal_authorized=authorized,
            per_agent=tuple(self.evidence[agent_id] for agent_id in self.agent_ids),
            failures=tuple(failures),
            fallback_used=False,
        )
