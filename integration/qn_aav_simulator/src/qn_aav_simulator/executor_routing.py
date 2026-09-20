"""Static executor routing and the active-unit mutual-exclusion invariant.

The plan allows several execution units to own the same physical AAVs - a
single-platform unit and a group unit are two ways of using one fleet, not two
fleets - while forbidding two of them from being occupied at the same time.  The
planning layer enforces that on the plan (``mrta_python.executors``); this module
enforces it at dispatch, where the question is "may this unit be dispatched while
that one is active".

Kept free of ROS so it can be tested directly; the runner imports it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ExecutionUnit:
    """One dispatchable unit as the runner sees it."""

    executor_id: str
    physical_agent_ids: Tuple[str, ...]
    capabilities: Tuple[str, ...]
    action_endpoint: Optional[str]
    initial_target_ref: str
    action_type: str = "FormationAction"
    operations: Tuple[str, ...] = ("AIR_MOVE",)
    odometry_topics: Tuple[Tuple[str, str], ...] = ()

    @property
    def dispatchable(self) -> bool:
        """A unit is dispatchable only if it has a real Action endpoint.

        A target-fleet platform may be planned for while having no backend yet;
        it must never produce a dispatch.
        """
        return self.action_endpoint is not None

    def members(self) -> frozenset:
        return frozenset(self.physical_agent_ids)


def load_routing(configured: Optional[Sequence[Mapping]], *,
                 default_members: Iterable[str],
                 default_initial_target_ref: str,
                 default_capabilities: Sequence[str] = ("AIR", "AAV"),
                 default_endpoint: str = "/formation_action") -> Dict[str, ExecutionUnit]:
    """Build the routing table, falling back to the single seven-member unit."""
    if configured is None:
        members = tuple(default_members)
        unit = ExecutionUnit("aav_formation", members, tuple(default_capabilities),
                             default_endpoint, default_initial_target_ref,
                             odometry_topics=tuple((member,"/"+member+"_qn/odometry") for member in members))
        return {unit.executor_id: unit}
    routing: Dict[str, ExecutionUnit] = {}
    for entry in configured:
        executor_id = str(entry["executor_id"])
        if not executor_id or executor_id in routing:
            raise ValueError("executor ids must be nonempty and unique")
        members = tuple(entry["physical_agent_ids"])
        if not members or len(set(members)) != len(members):
            raise ValueError("unit members must be nonempty and unique")
        endpoint = entry.get("action_endpoint")
        action_type = entry.get("action_type", "FormationAction")
        if action_type not in ("FormationAction", "PlatformTaskAction"):
            raise ValueError("unsupported Action type: " + str(action_type))
        operations = tuple(entry.get("operations", ("AIR_MOVE",) if action_type == "FormationAction" else ()))
        allowed = ({"AIR_MOVE"} if action_type == "FormationAction" else
                   {"ENTER_WATER", "WATER_PATH", "EXIT_WATER", "SURFACE_PATH"})
        if not operations or len(set(operations)) != len(operations) or not set(operations) <= allowed:
            raise ValueError("operations must match the configured Action type")
        topics = entry.get("odometry_topics")
        if topics is None and action_type == "FormationAction":
            topics = {member: "/" + member + "_qn/odometry" for member in members}
        if (not isinstance(topics, Mapping) or set(topics) != set(members) or
                any(not isinstance(topic, str) or not topic.startswith('/') for topic in topics.values())):
            raise ValueError("explicit odometry topics must cover exactly the physical members")
        routing[executor_id] = ExecutionUnit(
            executor_id=executor_id,
            physical_agent_ids=tuple(entry["physical_agent_ids"]),
            capabilities=tuple(entry.get("capabilities", ())),
            action_endpoint=None if endpoint in (None, "", "null") else str(endpoint),
            initial_target_ref=entry.get("initial_target_ref", default_initial_target_ref),
            action_type=action_type, operations=operations,
            odometry_topics=tuple(sorted(topics.items())),
        )
    return routing


def dispatchable_units(routing: Mapping[str, ExecutionUnit]) -> Tuple[ExecutionUnit, ...]:
    """Units that may actually be dispatched, in a stable order."""
    return tuple(routing[key] for key in sorted(routing) if routing[key].dispatchable)


def unit_for_coalition(routing: Mapping[str, ExecutionUnit],
                       coalition: Iterable[str], *,
                       operations: Optional[Iterable[str]] = None,
                       executor_id: Optional[str] = None) -> Optional[ExecutionUnit]:
    """Match physical members AND selected execution operations/identity.

    The same member can have AIR and native endpoints. Ambiguous legacy calls
    fail closed; registration order must never choose its reference source.
    """
    wanted = frozenset(coalition)
    required = None if operations is None else frozenset(operations)
    if required is not None and not required:
        raise ValueError("at least one execution operation is required")
    candidates = [unit for unit in dispatchable_units(routing)
                  if unit.members() == wanted
                  and (executor_id is None or unit.executor_id == executor_id)
                  and (required is None or required <= set(unit.operations))]
    if len(candidates) > 1:
        raise ValueError("ambiguous executor: specify selected identity and operations")
    return candidates[0] if candidates else None


def conflicting_active_unit(routing: Mapping[str, ExecutionUnit],
                            active_executor_ids: Iterable[str],
                            candidate_id: str) -> Optional[ExecutionUnit]:
    """An already-active unit that shares a physical agent with the candidate.

    Returns the offender, or None when the candidate may be dispatched.  This is
    the dispatch-side form of CoCoPlan's mutual-exclusion relation: two units may
    be registered together, but a shared member cannot be in two places.
    """
    try:
        candidate = routing[candidate_id]
    except KeyError as error:
        raise ValueError("unknown executor_id: {}".format(candidate_id)) from error
    for other_id in active_executor_ids:
        try:
            other = routing[other_id]
        except KeyError as error:
            raise ValueError("unknown active executor_id: {}".format(other_id)) from error
        if other.members() & candidate.members():
            return other
    return None
