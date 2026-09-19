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
                             default_endpoint, default_initial_target_ref)
        return {unit.executor_id: unit}
    routing: Dict[str, ExecutionUnit] = {}
    for entry in configured:
        executor_id = str(entry["executor_id"])
        endpoint = entry.get("action_endpoint")
        routing[executor_id] = ExecutionUnit(
            executor_id=executor_id,
            physical_agent_ids=tuple(entry["physical_agent_ids"]),
            capabilities=tuple(entry.get("capabilities", ())),
            action_endpoint=None if endpoint in (None, "", "null") else str(endpoint),
            initial_target_ref=entry.get("initial_target_ref", default_initial_target_ref),
        )
    return routing


def dispatchable_units(routing: Mapping[str, ExecutionUnit]) -> Tuple[ExecutionUnit, ...]:
    """Units that may actually be dispatched, in a stable order."""
    return tuple(routing[key] for key in sorted(routing) if routing[key].dispatchable)


def unit_for_coalition(routing: Mapping[str, ExecutionUnit],
                       coalition: Iterable[str]) -> Optional[ExecutionUnit]:
    """The dispatchable unit whose members are exactly this coalition.

    Matching by membership rather than by name is what makes the plan and the
    routing agree: the plan says which agents run the task, and the routing says
    which endpoint owns those agents.
    """
    wanted = frozenset(coalition)
    for unit in dispatchable_units(routing):
        if unit.members() == wanted:
            return unit
    return None


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
        if other_id == candidate_id:
            continue
        try:
            other = routing[other_id]
        except KeyError as error:
            raise ValueError("unknown active executor_id: {}".format(other_id)) from error
        if other.members() & candidate.members():
            return other
    return None
