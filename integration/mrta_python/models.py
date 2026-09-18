from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Tuple


@dataclass(frozen=True)
class Agent:
    id: str
    capabilities: FrozenSet[str]
    available_from: float = 0.0


@dataclass(frozen=True)
class Task:
    task_id: str
    required_capabilities: FrozenSet[str]
    required_agent_count: int
    service_time: float
    deadline: float
    target_ref: str


@dataclass
class PlanItem:
    execution_id: str
    task_id: str
    coalition: Tuple[str, ...]
    planned_start: float
    planned_finish: float
    travel_time: float
    wait_time: float
    service_time: float
    status: str = "PLANNED"


@dataclass
class Plan:
    items: List[PlanItem] = field(default_factory=list)

    def item(self, execution_id: str) -> PlanItem:
        for item in self.items:
            if item.execution_id == execution_id:
                return item
        raise KeyError(execution_id)

    @property
    def makespan(self) -> float:
        return max((item.planned_finish for item in self.items), default=0.0)

    @property
    def robot_queues(self) -> Dict[str, Tuple[str, ...]]:
        queues = {}
        for item in self.items:
            for agent_id in item.coalition:
                queues.setdefault(agent_id, []).append(item.execution_id)
        return {agent_id: tuple(queue) for agent_id, queue in queues.items()}

    @property
    def coalitions(self) -> Dict[str, Tuple[str, ...]]:
        return {item.execution_id: item.coalition for item in self.items}

    @property
    def task_start_times(self) -> Dict[str, float]:
        return {item.task_id: item.planned_start for item in self.items}

    @property
    def task_finish_times(self) -> Dict[str, float]:
        return {item.task_id: item.planned_finish for item in self.items}

    @property
    def waiting_times(self) -> Dict[str, float]:
        return {item.execution_id: item.wait_time for item in self.items}


@dataclass(frozen=True)
class DelayEvent:
    event_id: str
    execution_id: str
    task_id: str
    planned_finish: float
    actual_finish: float

    @property
    def delay_seconds(self) -> float:
        return self.actual_finish - self.planned_finish
