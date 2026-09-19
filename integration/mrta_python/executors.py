"""Offline multi-resource executor layer (plan.md P3).

Planning layer only.  An executor is a bookable execution unit that owns one or
more physical agents.  This module selects which executor runs a task and how
tasks compete for the same unit; it does not split the seven-member physical
formation used by the ROS mission and it claims no multi-coalition physical
closed loop.

Scope of this stage: resource selection and plan-level competition.  The
completion/repair semantics ported for the fixed coalition (``process_completion``
/ ``plan_repair``) are deliberately not duplicated here.
"""

from __future__ import annotations

import math
import random
import os
import pickle
import signal
import subprocess
import sys
import time
import copy
from dataclasses import dataclass, field
from typing import Callable, Dict, FrozenSet, List, Mapping, Sequence, Tuple
from typing import Optional

from .validation import identifier, nonnegative


ExecutorTravelTimeFunction = Callable[[str, str, str], float]


class PlanningBudgetExceeded(RuntimeError):
    """No complete candidate was submitted within this invocation's budget."""


def bounded_travel_query(provider, arguments, deadline):
    """A hung query cannot outlive the caller's observation budget.

    A clean interpreter avoids both forking ROS threads and multiprocessing's
    re-execution of catkin's __main__ wrapper. New budgeted callers provide
    serializable, snapshot-based providers (internal trusted objects only).
    Legacy unbudgeted callers retain their in-process callable interface.
    """
    if deadline is None:
        return provider(*arguments)
    if time.monotonic()>=deadline:
        raise PlanningBudgetExceeded('planning budget exhausted')
    payload=pickle.dumps((provider,arguments))
    environment=dict(os.environ,PYTHONPATH=os.pathsep.join(sys.path))
    process=subprocess.Popen([sys.executable,'-m','mrta_python.query_worker'],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
        env=environment,start_new_session=True)
    try:
        try:
            output,error=process.communicate(payload,timeout=max(0.,deadline-time.monotonic()))
        except subprocess.TimeoutExpired as exc:
            raise PlanningBudgetExceeded('motion query exceeded remaining planning budget') from exc
        if process.returncode:
            raise ValueError('motion query process failed: '+error.decode(errors='replace')[-2000:])
        ok,value=pickle.loads(output)
        if time.monotonic()>deadline:
            raise PlanningBudgetExceeded('planning budget exhausted during motion query')
        if not ok:
            raise ValueError('motion query failed: '+value)
        return value
    finally:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:process.communicate(timeout=.2)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.communicate()


def checked_predecessors(tasks, extra_edges=()):
    """Check the UNION of task, resource and motion precedence constraints."""
    predecessors={t.task_id:set(getattr(t,'predecessors',())) for t in tasks}
    for before,after in extra_edges:
        if after not in predecessors:
            raise ValueError('precedence refers to unknown task: '+after)
        predecessors[after].add(before)
    if any(p not in predecessors for values in predecessors.values() for p in values):
        raise ValueError('precedence refers to unknown task')
    remaining={k:set(v) for k,v in predecessors.items()}
    while remaining:
        ready={k for k,v in remaining.items() if not v}
        if not ready:
            raise ValueError('combined task/resource/motion precedence cycle')
        remaining={k:v-ready for k,v in remaining.items() if k not in ready}
    return predecessors

def overlapping_units(first: str, second: str,
                      membership: Mapping[str, Tuple[str, ...]]) -> bool:
    """True when two executor ids share at least one physical agent."""
    return bool(set(membership[first]) & set(membership[second]))


def _distance(first, second) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _capabilities(values, name: str) -> None:
    if not isinstance(values, frozenset):
        raise ValueError("{} must be a frozenset".format(name))
    for value in values:
        identifier(value, name)


def validate_executor_inputs(executors: Sequence[Executor], tasks: Sequence) -> None:
    """Check the offline executor domain before allocating anything.

    Unlike the fixed-coalition domain this does not require seven agents per
    task: a unit must be capable and large enough for the task it runs.
    """
    if not executors:
        raise ValueError("at least one executor is required")
    executor_ids = set()
    membership: Dict[str, Tuple[str, ...]] = {}
    for executor in executors:
        identifier(executor.executor_id, "executor_id")
        if executor.executor_id in executor_ids:
            raise ValueError("duplicate executor_id: {}".format(executor.executor_id))
        executor_ids.add(executor.executor_id)
        if (not isinstance(executor.physical_agent_ids, tuple)
                or not executor.physical_agent_ids):
            raise ValueError("physical_agent_ids must be a non-empty tuple")
        members = set()
        for agent_id in executor.physical_agent_ids:
            identifier(agent_id, "physical agent id")
            if agent_id in members:
                raise ValueError(
                    "duplicate physical agent in {}: {}".format(
                        executor.executor_id, agent_id))
            members.add(agent_id)
        nonnegative(executor.available_from, "available_from")
        nonnegative(executor.nominal_speed_mps, "nominal_speed_mps")
        if executor.nominal_speed_mps == 0:
            raise ValueError("nominal_speed_mps must be positive")
        _capabilities(executor.capabilities, "executor capabilities")
        membership[executor.executor_id] = executor.physical_agent_ids
    # Static membership may overlap on purpose: a single-platform unit and a
    # group unit can own the same physical agents, because they are two ways of
    # using the fleet rather than two fleets.  What must never happen is both
    # being occupied at once; that is enforced on the plan, not on registration.
    task_ids = set()
    for task in tasks:
        identifier(task.task_id, "task_id")
        if task.task_id in task_ids:
            raise ValueError("duplicate task_id: {}".format(task.task_id))
        task_ids.add(task.task_id)
        if type(task.required_agent_count) is not int or task.required_agent_count < 1:
            raise ValueError("required_agent_count must be a positive integer")
        nonnegative(task.service_time, "service_time")
        nonnegative(task.deadline, "deadline")
        _capabilities(task.required_capabilities, "required capabilities")
        if not eligible_executors(executors, task):
            raise ValueError("no eligible executor for {}".format(task.task_id))


@dataclass(frozen=True)
class Executor:
    """One bookable execution unit.

    ``physical_agent_ids`` is the unit's fixed membership: this stage never
    splits or reshuffles it.  ``capabilities`` are the unit's own capabilities;
    ``nominal_speed_mps`` is the unit's travel speed, which is why two eligible
    units can have different costs for the same task.
    """

    executor_id: str
    physical_agent_ids: Tuple[str, ...]
    capabilities: FrozenSet[str]
    available_from: float = 0.0
    nominal_speed_mps: float = 1.5
    initial_target_ref: Optional[str] = None


@dataclass
class ExecutorPlanItem:
    execution_id: str
    task_id: str
    executor_id: str
    coalition: Tuple[str, ...]
    planned_start: float
    planned_finish: float
    travel_time: float
    wait_time: float
    service_time: float
    status: str = "PLANNED"
    actual_finish: Optional[float] = None


@dataclass
class ExecutorPlan:
    items: List[ExecutorPlanItem] = field(default_factory=list)
    serial: bool = True
    precedence_edges: Tuple[Tuple[str, str], ...] = ()

    def item(self, execution_id: str) -> ExecutorPlanItem:
        for item in self.items:
            if item.execution_id == execution_id:
                return item
        raise KeyError(execution_id)

    @property
    def makespan(self) -> float:
        return max((item.planned_finish for item in self.items), default=0.0)

    @property
    def executor_queues(self) -> Dict[str, Tuple[str, ...]]:
        queues: Dict[str, List[str]] = {}
        for item in self.items:
            queues.setdefault(item.executor_id, []).append(item.execution_id)
        return {executor_id: tuple(queue) for executor_id, queue in queues.items()}

    @property
    def executor_finish_times(self) -> Dict[str, float]:
        finish: Dict[str, float] = {}
        for item in self.items:
            finish[item.executor_id] = max(
                finish.get(item.executor_id, 0.0), item.planned_finish)
        return finish

    @property
    def task_start_times(self) -> Dict[str, float]:
        return {item.task_id: item.planned_start for item in self.items}

    @property
    def task_finish_times(self) -> Dict[str, float]:
        return {item.task_id: item.planned_finish for item in self.items}

    @property
    def assignments(self) -> Dict[str, str]:
        return {item.task_id: item.executor_id for item in self.items}


@dataclass(frozen=True)
class ExecutorTravelTimeProvider:
    """Travel time for the *selected unit*.

    With no member detail this is the centre-to-centre Euclidean distance divided
    by the unit's nominal speed, which is what the offline cases need.  Once
    ``member_slots`` and ``member_positions`` are supplied the estimate is taken
    per member instead: member *i* travels from where it actually is to its own
    slot at the destination, and the group finishes when the **last** member
    arrives.  That matters as soon as the fleet is not already in formation -
    after independent single-platform tasks the members are scattered, and a
    single averaged centre would understate the reassembly cost.
    """

    centers: Mapping[str, Tuple[float, float, float]]
    nominal_speed_mps: Mapping[str, float]
    # executor_id -> {member_id: (x, y, z)} slot offset relative to the centre
    member_slots: Mapping[str, Mapping[str, Tuple[float, float, float]]] = field(
        default_factory=dict)
    # member_id -> (x, y, z) where that member is now.  Keyed by the physical
    # member, not by the unit: a member belongs to the fleet, and a group unit has
    # to see where a member was left by an earlier single-platform task.  Written
    # from execution feedback, or advanced by the planner as it schedules.
    member_positions: Mapping[str, Tuple[float, float, float]] = field(
        default_factory=dict)

    def __post_init__(self) -> None:
        for target_ref, center in self.centers.items():
            identifier(target_ref, "target_ref")
            if len(center) != 3 or any(not math.isfinite(value) for value in center):
                raise ValueError("target centers must contain three finite coordinates")
        for executor_id, speed in self.nominal_speed_mps.items():
            identifier(executor_id, "executor_id")
            nonnegative(speed, "nominal_speed_mps")
            if speed == 0:
                raise ValueError("nominal_speed_mps must be positive")

    def __call__(self, executor_id: str, from_target_ref: str,
                 to_target_ref: str) -> float:
        try:
            source = self.centers[from_target_ref]
            destination = self.centers[to_target_ref]
            speed = self.nominal_speed_mps[executor_id]
        except KeyError as error:
            raise ValueError("unknown reference: {}".format(error.args[0])) from error
        slots = self.member_slots.get(executor_id)
        if not slots:
            return _distance(source, destination) / speed
        longest = 0.0
        for member_id, slot in slots.items():
            goal = tuple(destination[axis] + slot[axis] for axis in range(3))
            # A member with no reported position is assumed to still be in
            # formation at the source, which is the optimistic case and is
            # stated here rather than hidden.
            start = self.member_positions.get(member_id) or tuple(
                source[axis] + slot[axis] for axis in range(3))
            longest = max(longest, _distance(start, goal))
        return longest / speed


def eligible_executors(executors: Sequence[Executor], task) -> Tuple[Executor, ...]:
    """Units that could execute ``task``: capable, and the right size.

    Capable and large enough is not sufficient.  A unit larger than the task
    asks for is only eligible when the task opts in with
    ``allow_larger_unit``; otherwise a one-platform survey would be handed to
    the group unit whenever the group happened to be cheaper.
    """
    wanted = task.required_agent_count
    allow_larger = bool(getattr(task, "allow_larger_unit", False))
    return tuple(
        executor for executor in executors
        if task.required_capabilities.issubset(executor.capabilities)
        and (not getattr(task,'required_members',()) or
             set(task.required_members)==set(executor.physical_agent_ids))
        and (len(executor.physical_agent_ids) >= wanted if allow_larger
             else len(executor.physical_agent_ids) == wanted))


def build_executor_plan(executors: Sequence[Executor], tasks: Sequence,
                        travel_time_provider: ExecutorTravelTimeFunction, *,
                        initial_target_ref: str, seed: int = 0,
                        serial: bool = False,
                        serial_units: Optional[Iterable[str]] = None,
                        precedence_edges=(), budget_s=None,
                        hard_deadlines: bool = False) -> ExecutorPlan:
    """Allocate every task to one eligible unit with the v9 reward order.

    Generalization of the fixed-coalition port: each unit keeps its own queue
    finishing time, so a candidate's earliest start is that unit's queue end
    (identical to the single global queue when only one unit exists).  The
    lexicographic reward is unchanged, including the reciprocal terms written
    as equivalent ascending costs, zero planned wait, and a soft deadline.
    Ties are broken by ``executor_id`` so selection does not depend on input
    order.

    ``serial=True`` makes the plan respect a globally serial execution, which is
    what the task line runs.  The start of a candidate is

        t_start = max(F_serial, max over the unit's members of A_pred)

    where ``F_serial`` is the finish of the previous *scheduled* item and
    ``A_pred`` is each member's predicted availability.  It deliberately does not
    take the maximum over every unit: a resource that this task does not use must
    not delay it.  ``serial_units`` names the units that take part in the online
    serial clock - platforms without an execution endpoint are planned for but
    never dispatched, so they must not hold that clock back either.
    """
    started=time.monotonic()
    if budget_s is not None and (not math.isfinite(budget_s) or budget_s<=0):
        raise ValueError('planning budget must be finite and positive')
    deadline=None if budget_s is None else started+budget_s
    validate_executor_inputs(executors, tasks)
    predecessors=checked_predecessors(tasks,precedence_edges)
    finishes={}
    if budget_s is not None or hard_deadlines or any(predecessors.values()):
        # Speculative planning cannot move the caller's accepted member state.
        travel_time_provider=copy.deepcopy(travel_time_provider)
    identifier(initial_target_ref, "initial_target_ref")
    if type(seed) is not int:
        raise ValueError("seed must be an integer")
    if not callable(travel_time_provider):
        raise ValueError("travel_time_provider must be callable")
    rng = random.Random(seed)
    # Each unit keeps its own availability *and* its own current position.  A
    # single shared position would move every unit to whichever target the last
    # allocation happened to visit.
    queue_finish = {executor.executor_id: executor.available_from
                    for executor in executors}
    current_target_ref = {
        executor.executor_id: (executor.initial_target_ref or initial_target_ref)
        for executor in executors}
    participants = (set(serial_units) if serial_units is not None
                    else {executor.executor_id for executor in executors})
    # Each physical member has its own predicted availability; a unit's queue is
    # derived from its members rather than kept as a second, competing truth.
    member_available: Dict[str, float] = {}
    for executor in executors:
        for agent_id in executor.physical_agent_ids:
            member_available[agent_id] = max(
                member_available.get(agent_id, 0.0), executor.available_from)
    serial_release = 0.0

    def earliest_start(executor_id: str) -> float:
        """When this unit may begin.

        Serial mode: the previous scheduled item's finish, and this unit's own
        members' predicted availability.  Otherwise: this unit's queue plus any
        unit it shares members with, which is the mutual-exclusion rule.
        """
        if serial and executor_id in participants:
            start = serial_release
            for agent_id in routing_members[executor_id]:
                start = max(start, member_available.get(agent_id, 0.0))
            return start
        start = queue_finish[executor_id]
        for other_id, other_finish in queue_finish.items():
            if other_id != executor_id and overlapping_units(executor_id, other_id, routing_members):
                start = max(start, other_finish)
        return start

    routing_members = {executor.executor_id: executor.physical_agent_ids
                       for executor in executors}

    remaining = list(tasks)
    plan = ExecutorPlan(serial=serial,precedence_edges=tuple(
        (before,after) for after,values in predecessors.items() for before in sorted(values)))
    while remaining:
        if deadline is not None and time.monotonic()>=deadline:
            raise PlanningBudgetExceeded('no complete plan within invocation budget')
        candidates = sorted((t for t in remaining if predecessors[t.task_id]<=finishes.keys()),
                            key=lambda task: task.task_id)
        rng.shuffle(candidates)
        makespan = max(queue_finish.values())
        scored = []
        for task in candidates:
            for executor in eligible_executors(executors, task):
                transit = bounded_travel_query(travel_time_provider, (
                    executor.executor_id,
                    current_target_ref[executor.executor_id], task.target_ref), deadline)
                nonnegative(transit, "travel_time_provider result")
                start = max(earliest_start(executor.executor_id),
                            max((finishes[p] for p in predecessors[task.task_id]),default=0.))
                finish = start + transit + task.service_time
                nonnegative(finish, "candidate planned_finish")
                if hard_deadlines and finish>task.deadline:
                    continue
                introduced_makespan = max(0.0, finish - makespan)
                urgent = task.deadline < start + 1.55 * task.service_time
                key = (-int(urgent), introduced_makespan, 0.0, -task.service_time,
                       -1.0, transit, executor.executor_id)
                scored.append((key, task, executor, transit, start, finish))
        if not scored:
            raise ValueError('no admissible next candidate found; not a proof of infeasibility')
        _, selected, executor, transit, start, finish = min(
            scored, key=lambda candidate: candidate[0])
        plan.items.append(ExecutorPlanItem(
            execution_id="exec-{:04d}-{}".format(len(plan.items), selected.task_id),
            task_id=selected.task_id, executor_id=executor.executor_id,
            coalition=executor.physical_agent_ids,
            planned_start=start, planned_finish=finish,
            travel_time=transit, wait_time=0.0, service_time=selected.service_time,
        ))
        queue_finish[executor.executor_id] = finish
        finishes[selected.task_id]=finish
        current_target_ref[executor.executor_id] = selected.target_ref
        if serial and executor.executor_id in participants:
            serial_release = finish
        for agent_id in executor.physical_agent_ids:
            member_available[agent_id] = finish
        # Physical members are authoritative: a later group task must start from
        # where each member is predicted to be, not from the unit's old centre.
        positions = getattr(travel_time_provider, "member_positions", None)
        if positions is not None:
            destination = travel_time_provider.centers[selected.target_ref]
            slots = getattr(travel_time_provider, "member_slots", {}).get(
                executor.executor_id, {})
            for member_id in executor.physical_agent_ids:
                slot = slots.get(member_id, (0.0, 0.0, 0.0))
                positions[member_id] = tuple(
                    destination[axis] + slot[axis] for axis in range(3))
        remaining.remove(selected)
    validate_executor_plan(plan, executors, tasks)
    if deadline is not None and time.monotonic()>deadline:
        raise PlanningBudgetExceeded('complete candidate exceeded invocation budget')
    return plan


def validate_executor_plan(plan: ExecutorPlan, executors: Sequence[Executor],
                           tasks: Sequence) -> None:
    """Raise ValueError on domain violations; deadlines stay soft."""
    validate_executor_inputs(executors, tasks)
    by_id = {executor.executor_id: executor for executor in executors}
    task_by_id = {task.task_id: task for task in tasks}
    execution_ids, task_ids = set(), set()
    queue_finish: Dict[str, float] = {}
    for item in plan.items:
        identifier(item.execution_id, "execution_id")
        identifier(item.task_id, "task_id")
        identifier(item.executor_id, "executor_id")
        if item.execution_id in execution_ids or item.task_id in task_ids:
            raise ValueError("plan execution_id and task_id must be unique")
        execution_ids.add(item.execution_id)
        task_ids.add(item.task_id)
        try:
            executor = by_id[item.executor_id]
        except KeyError as error:
            raise ValueError("unknown executor_id: {}".format(item.executor_id)) from error
        try:
            task = task_by_id[item.task_id]
        except KeyError as error:
            raise ValueError("unknown task_id: {}".format(item.task_id)) from error
        if executor not in eligible_executors(executors, task):
            raise ValueError("executor {} is not eligible for task {}".format(
                item.executor_id, item.task_id))
        if item.coalition != executor.physical_agent_ids:
            raise ValueError("plan coalition must equal the unit's fixed membership")
        for name in ("planned_start", "planned_finish", "travel_time", "wait_time",
                     "service_time"):
            nonnegative(getattr(item, name), name)
        if item.wait_time != 0:
            raise ValueError("positive planned wait is outside the current domain")
        expected = item.planned_start + item.travel_time + item.wait_time + item.service_time
        if not math.isclose(item.planned_finish, expected, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("planned_finish must equal start + travel + wait + service")
        if item.service_time != task.service_time:
            raise ValueError("plan service_time differs from task service_time")
        if item.planned_start < queue_finish.get(item.executor_id,
                                                 executor.available_from) - 1e-6:
            raise ValueError("tasks on one executor overlap or are out of queue order")
        queue_finish[item.executor_id] = item.planned_finish
    # Mutual exclusion (CoCoPlan): two units that share a physical agent may be
    # registered together but must never be occupied at the same time.
    occupied: Dict[str, List[ExecutorPlanItem]] = {}
    for item in plan.items:
        occupied.setdefault(item.executor_id, []).append(item)
    membership = {e.executor_id: e.physical_agent_ids for e in executors}
    for first, items_a in occupied.items():
        for second, items_b in occupied.items():
            if first >= second or not overlapping_units(first, second, membership):
                continue
            for a in items_a:
                for b in items_b:
                    if (a.planned_start < b.planned_finish
                            and b.planned_start < a.planned_finish):
                        raise ValueError(
                            "units {} and {} share a physical agent but {} and {} "
                            "overlap in time".format(
                                first, second, a.execution_id, b.execution_id))
    if task_ids != set(task_by_id):
        raise ValueError("plan must allocate each supplied task exactly once")
    before_by_task=checked_predecessors(tasks,plan.precedence_edges)
    item_by_task={i.task_id:i for i in plan.items}
    for task_id,values in before_by_task.items():
        if any(item_by_task[task_id].planned_start<item_by_task[p].planned_finish-1e-6 for p in values):
            raise ValueError('plan violates a task/resource/motion predecessor')
