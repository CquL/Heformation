from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Tuple, TYPE_CHECKING
import math

if TYPE_CHECKING:
    from .executors import ExecutorPlanItem


@dataclass(frozen=True)
class NativeSegmentSpec:
    operation: str
    points: Tuple[Tuple[float,float,float], ...]
    duration_s: float = 0.0
    propulsion_effort: float = 0.0

    def __post_init__(self):
        object.__setattr__(self,'points',tuple(tuple(p) for p in self.points))
        if self.operation not in ('ENTER_WATER','EXIT_WATER','WATER_PATH','SURFACE_PATH'):
            raise ValueError('unsupported native operation')
        if len(self.points)<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError('native segment needs finite path points')
        if not math.isfinite(self.duration_s) or self.duration_s<0:
            raise ValueError('invalid native reference duration')
        if not math.isfinite(self.propulsion_effort) or self.propulsion_effort<0:
            raise ValueError('invalid native segment propulsion effort')
        if self.operation in ('ENTER_WATER','EXIT_WATER') and self.propulsion_effort:
            raise ValueError('qn transition does not use marine propulsion effort')
        if self.operation in ('ENTER_WATER','EXIT_WATER') and self.duration_s<=0:
            raise ValueError('qn transitions need positive reference duration')


@dataclass(frozen=True)
class NativeActionSpec:
    segments: Tuple[NativeSegmentSpec, ...]
    terminal_behavior: str
    execution_timeout_s: float = 180.0
    observation_ids: Tuple[str, ...] = ()
    terminal_wait_s: float = 0.0

    def __post_init__(self):
        object.__setattr__(self,'segments',tuple(self.segments))
        object.__setattr__(self,'observation_ids',tuple(self.observation_ids))
        if any(not isinstance(v,str) or not v for v in self.observation_ids) or len(set(self.observation_ids))!=len(self.observation_ids):
            raise ValueError('observation IDs must be unique nonempty strings')
        if not all(isinstance(s,NativeSegmentSpec) for s in self.segments):
            raise ValueError('native fragments require typed segment specifications')
        if not 1<=len(self.segments)<=16 or self.terminal_behavior not in ('FIXED_REFERENCE','COAST_STOP','TRIM_PROPULSION'):
            raise ValueError('invalid native fragment/terminal contract')
        if not math.isfinite(self.execution_timeout_s) or self.execution_timeout_s<=0:
            raise ValueError('native observation timeout must be finite and positive')
        if not math.isfinite(self.terminal_wait_s) or not 0<=self.terminal_wait_s<self.execution_timeout_s:
            raise ValueError('terminal wait must fit the finite observation timeout')
        if any(a.points[-1]!=b.points[0] for a,b in zip(self.segments,self.segments[1:])):
            raise ValueError('native fragment paths must connect')

    @property
    def final_mode(self):
        return {'ENTER_WATER':'WATER','WATER_PATH':'WATER','EXIT_WATER':'AIR','SURFACE_PATH':'SURFACE'}[self.segments[-1].operation]


@dataclass(frozen=True)
class ExecutionStep:
    """One already evaluated step of a candidate's complete execution chain."""
    executor_id: str
    duration_s: float
    target_ref: str
    native_action: 'NativeActionSpec | None' = None
    service_time_s: float = 0.0
    native_prediction: dict = field(default_factory=dict)
    # AIR FormationGoal has no NativeActionSpec; only this step may produce
    # the declared local observation. A following same-position hold does not.
    observation_ids: Tuple[str, ...] = ()

    def __post_init__(self):
        if not self.executor_id or not self.target_ref or not math.isfinite(self.duration_s) or self.duration_s<0:
            raise ValueError('invalid candidate execution step')
        if not math.isfinite(self.service_time_s) or not 0<=self.service_time_s<=self.duration_s:
            raise ValueError('invalid step service duration')
        if self.native_action is not None and self.service_time_s!=0:
            raise ValueError('native step already includes its terminal duration')
        if self.native_action is not None and self.observation_ids:
            raise ValueError('native step keeps observation IDs in its native action')
        if len(set(self.observation_ids))!=len(self.observation_ids) or any(not isinstance(v,str) or not v for v in self.observation_ids):
            raise ValueError('AIR step observation IDs must be unique nonempty strings')
        if any(k in self.native_prediction for k in ('trajectory','terminal_backend')):
            raise ValueError('keep dense trajectory/model state outside published step metadata')


@dataclass(frozen=True)
class ExecutionCandidate:
    candidate_id: str
    steps: Tuple[ExecutionStep, ...]
    terminal_states: Dict[str, dict]
    status: str = 'FEASIBLE'
    reason: str = ''
    # Cooperative methods reuse the existing plan activities, with times
    # relative to the queried start. Sequential legacy methods keep steps.
    # Exactly one representation is stored, never both.
    activities: Tuple['ExecutorPlanItem', ...] = ()
    # Internal query evidence in absolute plan time, never copied into a Goal
    # or public Plan metadata. Prefix idle is included; post-action idle is
    # evaluated only after the complete plan's actual successors are known.
    motion_traces: Dict[str, tuple] = field(default_factory=dict)
    collision_radii: Dict[str, float] = field(default_factory=dict)
    generated_products: Tuple[dict, ...] = ()

    def __post_init__(self):
        object.__setattr__(self,'steps',tuple(self.steps))
        object.__setattr__(self,'activities',tuple(self.activities))
        if not self.candidate_id or self.status not in ('FEASIBLE','INFEASIBLE','UNKNOWN'):
            raise ValueError('invalid candidate status')
        if self.status=='FEASIBLE' and (bool(self.steps)==bool(self.activities) or
                not all(isinstance(s,ExecutionStep) for s in self.steps)):
            raise ValueError('feasible candidate needs a complete execution chain')

    @property
    def duration_s(self):
        if self.activities:return max(a.planned_finish for a in self.activities)
        return sum(step.duration_s for step in self.steps)


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
    deadline: float | None
    target_ref: str
    # A single-platform task must not be handed to a larger unit just because
    # that unit is also capable and eligible: the bigger unit would drag members
    # that the task never asked for, and CARIC reports that splitting a small
    # fleet into teams need not help.  Tasks that genuinely want a formation set
    # this to True and then must also declare a group-level completion condition.
    allow_larger_unit: bool = False
    # Optional partial order; legacy fixed-coalition inputs remain unchanged.
    predecessors: Tuple[str, ...] = ()
    # A declared formation slot belongs to its physical member; ordinary survey
    # tasks leave this empty so allocation remains free.
    required_members: Tuple[str, ...] = ()


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
