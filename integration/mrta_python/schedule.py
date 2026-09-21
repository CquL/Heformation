"""Calvo v9's lexicographic reward in the fixed-coalition domain."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence, Tuple

from .models import Agent, Plan, PlanItem, Task
from .validation import identifier, nonnegative, validate_inputs, validate_plan


TravelTimeFunction = Callable[[Sequence[str], str, str], float]


@dataclass(frozen=True)
class TravelTimeProvider:
    """Euclidean formation-center distance divided by nominal speed (seconds)."""

    centers: Mapping[str, Tuple[float, float, float]]
    nominal_speed_mps: float = 1.5

    def __post_init__(self) -> None:
        nonnegative(self.nominal_speed_mps, "nominal_speed_mps")
        if self.nominal_speed_mps == 0:
            raise ValueError("nominal_speed_mps must be positive")
        for target_ref, center in self.centers.items():
            identifier(target_ref, "target_ref")
            if len(center) != 3 or any(not math.isfinite(value) for value in center):
                raise ValueError("target centers must contain three finite coordinates")

    def __call__(self, executor: Sequence[str], from_target_ref: str,
                 to_target_ref: str) -> float:
        try:
            source, destination = self.centers[from_target_ref], self.centers[to_target_ref]
        except KeyError as error:
            raise ValueError("unknown target_ref: {}".format(error.args[0])) from error
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(source, destination))) / self.nominal_speed_mps


def build_plan(agents: Sequence[Agent], tasks: Sequence[Task],
               travel_time_provider: TravelTimeFunction, *, initial_target_ref: str,
               seed: int = 0) -> Plan:
    """Allocate all tasks with v9 reward order, soft deadlines and zero wait.

    The reciprocal terms in MATLAB's descending reward are represented by
    their equivalent ascending costs, including the zero-cost/Inf cases.
    Initial availability initializes the virtual group queue's finishing time.
    """
    validate_inputs(agents, tasks)
    identifier(initial_target_ref, "initial_target_ref")
    if type(seed) is not int:
        raise ValueError("seed must be an integer")
    if not callable(travel_time_provider):
        raise ValueError("travel_time_provider must be callable")
    rng = random.Random(seed)
    coalition = tuple(agent.id for agent in agents)
    current_target_ref = initial_target_ref
    current_makespan = max(agent.available_from for agent in agents)
    remaining = list(tasks)
    plan = Plan()
    while remaining:
        # MATLAB shuffles before stable lexicographic sorting. Canonical input
        # order additionally makes this port insensitive to caller list order.
        candidates = sorted(remaining, key=lambda task: task.task_id)
        rng.shuffle(candidates)
        scored = []
        for task in candidates:
            transit = travel_time_provider(coalition, current_target_ref, task.target_ref)
            nonnegative(transit, "travel_time_provider result")
            finish = current_makespan + transit + task.service_time
            nonnegative(finish, "candidate planned_finish")
            introduced_makespan = max(0.0, finish - current_makespan)
            urgent = task.deadline is not None and task.deadline < current_makespan + 1.55 * task.service_time
            # nf=1, W=0, N/nc=7/7. No actual-lateness or EDD tie breaker.
            key = (-int(urgent), introduced_makespan, 0.0, -task.service_time, -1.0, transit)
            scored.append((key, task, transit, finish))
        _, selected, transit, finish = min(scored, key=lambda candidate: candidate[0])
        plan.items.append(PlanItem(
            execution_id="exec-{:04d}-{}".format(len(plan.items), selected.task_id),
            task_id=selected.task_id, coalition=coalition,
            planned_start=current_makespan, planned_finish=finish,
            travel_time=transit, wait_time=0.0, service_time=selected.service_time,
        ))
        current_makespan = finish
        current_target_ref = selected.target_ref
        remaining.remove(selected)
    validate_plan(plan, agents, tasks)
    return plan
