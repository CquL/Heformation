"""Checks for the fixed seven-member, zero-wait planning domain."""

import math
from numbers import Real
from typing import Optional, Sequence

from .models import Agent, Plan, Task


NUMERICAL_TOLERANCE = 1e-6
STATUSES = frozenset({"PLANNED", "RUNNING", "COMPLETED", "FAILED", "CANCELLED"})


def identifier(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} must be a non-empty string".format(name))


def nonnegative(value: float, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value < 0:
        raise ValueError("{} must be finite and non-negative".format(name))


def _capabilities(values, name: str) -> None:
    if not isinstance(values, frozenset):
        raise ValueError("{} must be a frozenset".format(name))
    for value in values:
        identifier(value, name)


def validate_inputs(agents: Sequence[Agent], tasks: Sequence[Task]) -> None:
    if len(agents) != 7:
        raise ValueError("the restricted domain requires exactly seven agents")
    agent_ids = set()
    for agent in agents:
        identifier(agent.id, "agent id")
        if agent.id in agent_ids:
            raise ValueError("duplicate agent id: {}".format(agent.id))
        agent_ids.add(agent.id)
        nonnegative(agent.available_from, "available_from")
        _capabilities(agent.capabilities, "agent capabilities")
    task_ids = set()
    for task in tasks:
        identifier(task.task_id, "task_id")
        identifier(task.target_ref, "target_ref")
        if task.task_id in task_ids:
            raise ValueError("duplicate task_id: {}".format(task.task_id))
        task_ids.add(task.task_id)
        if type(task.required_agent_count) is not int or task.required_agent_count != 7:
            raise ValueError("every task must require the complete seven-agent coalition")
        nonnegative(task.service_time, "service_time")
        if task.deadline is not None:
            nonnegative(task.deadline, "deadline")
        _capabilities(task.required_capabilities, "required capabilities")
        if any(not task.required_capabilities.issubset(agent.capabilities) for agent in agents):
            raise ValueError("no eligible fixed coalition for {}".format(task.task_id))


def validate_plan(plan: Plan, agents: Optional[Sequence[Agent]] = None,
                  tasks: Optional[Sequence[Task]] = None) -> None:
    """Raise ValueError on domain violations; deadlines are deliberately soft."""
    if (agents is None) != (tasks is None):
        raise ValueError("supply agents and tasks together")
    if agents is not None:
        validate_inputs(agents, tasks)
    execution_ids, task_ids = set(), set()
    coalition = None
    previous_finish = 0.0
    for item in plan.items:
        identifier(item.execution_id, "execution_id")
        identifier(item.task_id, "task_id")
        if item.execution_id in execution_ids or item.task_id in task_ids:
            raise ValueError("plan execution_id and task_id must be unique")
        execution_ids.add(item.execution_id)
        task_ids.add(item.task_id)
        if not isinstance(item.coalition, tuple) or len(item.coalition) != 7 or len(set(item.coalition)) != 7:
            raise ValueError("plan coalition must contain seven unique agents")
        for agent_id in item.coalition:
            identifier(agent_id, "coalition member")
        if coalition is None:
            coalition = item.coalition
        elif item.coalition != coalition:
            raise ValueError("coalition membership and order must remain fixed")
        if item.status not in STATUSES:
            raise ValueError("invalid plan status: {}".format(item.status))
        for name in ("planned_start", "planned_finish", "travel_time", "wait_time", "service_time"):
            nonnegative(getattr(item, name), name)
        if item.wait_time != 0:
            raise ValueError("positive planned wait is outside the current domain")
        expected = item.planned_start + item.travel_time + item.wait_time + item.service_time
        if not math.isclose(item.planned_finish, expected, rel_tol=0.0, abs_tol=NUMERICAL_TOLERANCE):
            raise ValueError("planned_finish must equal start + travel + wait + service")
        if item.planned_start < previous_finish - NUMERICAL_TOLERANCE:
            raise ValueError("fixed-coalition tasks overlap or are out of queue order")
        previous_finish = item.planned_finish
    if agents is not None:
        task_by_id = {task.task_id: task for task in tasks}
        if task_ids != set(task_by_id):
            raise ValueError("plan must allocate each supplied task exactly once")
        expected_coalition = tuple(agent.id for agent in agents)
        for item in plan.items:
            if item.coalition != expected_coalition:
                raise ValueError("plan coalition differs from configured agents")
            if item.planned_start < max(agent.available_from for agent in agents) - NUMERICAL_TOLERANCE:
                raise ValueError("task starts before the complete coalition is available")
            if item.service_time != task_by_id[item.task_id].service_time:
                raise ValueError("plan service_time differs from task service_time")
