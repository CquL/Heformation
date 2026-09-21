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
import contextlib
import select
import struct
import tempfile
from dataclasses import dataclass, field
from typing import Callable, Dict, FrozenSet, List, Mapping, Sequence, Tuple
from typing import Optional

from .validation import identifier, nonnegative
from .models import NativeActionSpec, ExecutionStep, ExecutionCandidate


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


def bounded_candidate_query(provider,arguments,deadline):
    """Stream completed candidates from the existing isolated query process.

    The parent can commit a complete feasible plan before a later alternative
    stalls. All input/output waits share the original monotonic deadline.
    This is a trusted local pickle channel, never a ROS or user-file protocol.
    """
    owner=getattr(provider,'__self__',None)
    provider=getattr(owner,'iter_execution_candidates',provider)
    payload=pickle.dumps((provider,arguments));buffer=bytearray()
    environment=dict(os.environ,PYTHONPATH=os.pathsep.join(sys.path))
    with tempfile.TemporaryFile() as errors:
        process=subprocess.Popen([sys.executable,'-m','mrta_python.query_worker','--stream'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=errors,env=environment,start_new_session=True)
        try:
            fd=process.stdin.fileno();os.set_blocking(fd,False);offset=0
            while offset<len(payload):
                left=deadline-time.monotonic()
                if left<=0 or not select.select([],[fd],[],left)[1]:raise PlanningBudgetExceeded('candidate query input deadline')
                offset+=os.write(fd,payload[offset:offset+65536])
            process.stdin.close()
            while True:
                if time.monotonic()>=deadline:raise PlanningBudgetExceeded('candidate query deadline')
                if len(buffer)>=8:
                    size=struct.unpack('!Q',buffer[:8])[0]
                    if len(buffer)>=8+size:
                        ok,value=pickle.loads(buffer[8:8+size]);del buffer[:8+size]
                        if ok=='BUDGET':raise PlanningBudgetExceeded(value)
                        if not ok:raise ValueError('candidate query failed: '+value)
                        yield value
                        continue
                left=deadline-time.monotonic()
                if left<=0 or not select.select([process.stdout],[],[],left)[0]:
                    raise PlanningBudgetExceeded('candidate query output deadline')
                data=os.read(process.stdout.fileno(),65536)
                if not data:
                    try:code=process.wait(timeout=max(.001,deadline-time.monotonic()))
                    except subprocess.TimeoutExpired:raise PlanningBudgetExceeded('candidate worker exit deadline')
                    if code or buffer:raise ValueError('candidate query ended without a complete response')
                    return
                buffer.extend(data)
        finally:
            try:os.killpg(process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:process.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait()
            process.stdin.close();process.stdout.close()


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
        if task.deadline is not None:
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
    # Selected method belongs to a candidate PlanItem, not the business Task.
    execution_steps: Tuple[ExecutionStep, ...] = ()
    candidate_id: str = ''
    predicted_member_states: Mapping = field(default_factory=dict)
    # Support/transit activities belong to a business task but cannot satisfy
    # its observation capability by themselves.
    fulfills_task: bool = True

    @property
    def native_action(self):
        """Read-only compatibility view; the selected steps own the method."""
        return self.execution_steps[0].native_action if len(self.execution_steps)==1 else None

    @property
    def native_prediction(self):
        return self.execution_steps[0].native_prediction if len(self.execution_steps)==1 else {}


@dataclass
class ExecutorPlan:
    items: List[ExecutorPlanItem] = field(default_factory=list)
    serial: bool = True
    precedence_edges: Tuple[Tuple[str, str], ...] = ()
    search_complete: Optional[bool] = None
    evaluated_candidates: int = 0
    activity_edges: Tuple[Tuple[str, str], ...] = ()
    # Compact evidence scope only: dense samples/models stay in the search.
    validation_scope: str = 'SCHEDULE_ONLY'

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
        starts={}
        for item in self.items:starts[item.task_id]=min(starts.get(item.task_id,item.planned_start),item.planned_start)
        return starts

    @property
    def task_finish_times(self) -> Dict[str, float]:
        finishes={}
        for item in self.items:finishes[item.task_id]=max(finishes.get(item.task_id,0.),item.planned_finish)
        return finishes

    @property
    def assignments(self) -> Dict[str, object]:
        units={}
        for item in self.items:units.setdefault(item.task_id,set()).add(item.executor_id)
        return {task:next(iter(ids)) if len(ids)==1 else tuple(sorted(ids)) for task,ids in units.items()}


def activity_predecessors(plan):
    """Lift business dependencies and check the joint execution-activity DAG."""
    by_task={}
    predecessors={i.execution_id:set() for i in plan.items}
    if len(predecessors)!=len(plan.items):raise ValueError('duplicate execution_id')
    for item in plan.items:by_task.setdefault(item.task_id,[]).append(item.execution_id)
    for before,after in plan.precedence_edges:
        if before not in by_task or after not in by_task:raise ValueError('unknown business predecessor')
        for successor in by_task[after]:predecessors[successor].update(by_task[before])
    for before,after in plan.activity_edges:
        if before not in predecessors or after not in predecessors:raise ValueError('unknown activity predecessor')
        predecessors[after].add(before)
    remaining={k:set(v) for k,v in predecessors.items()}
    # Support START precedes work, while their motion intervals may overlap.
    # Check its union with finish dependencies, without converting a start
    # commitment into a wait for support completion.
    groups={}
    for item in plan.items:
        if item.candidate_id:groups.setdefault((item.task_id,item.candidate_id),[]).append(item)
    for items in groups.values():
        supports={i.execution_id for i in items if not i.fulfills_task}
        for item in items:
            if item.fulfills_task:
                remaining[item.execution_id].update(supports)
                if any(item.planned_start<s.planned_start-1e-6 for s in items if not s.fulfills_task):
                    raise ValueError('dependent work precedes its support launch')
    while remaining:
        ready={k for k,v in remaining.items() if not v}
        if not ready:raise ValueError('combined task/resource/motion precedence cycle')
        remaining={k:v-ready for k,v in remaining.items() if k not in ready}
    return predecessors


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
    native_routes: Mapping = field(default_factory=dict)
    native_models: Mapping = field(default_factory=dict)
    native_efforts: Mapping = field(default_factory=dict)
    scene_geometry: object = None
    # Finite joint methods: (work executor, business task) -> mappings from
    # existing Executor objects to their native fragments. No new executor
    # wrapper or online solver; each mapping is a complete proposed method.
    cooperative_routes: Mapping = field(default_factory=dict)
    observation_request: object = None
    mother_position: tuple = ()

    def execution_candidates(self,unit,task,start,states,deadline):
        """Evaluate declared finite native route alternatives with plant state.

        Unqualified/missing methods remain unknown; they never inherit a
        nominal AIR distance/speed estimate. Models are internal forecast state.
        """
        if (unit.executor_id,task.task_id) in self.cooperative_routes:
            return list(self._iter_cooperative_candidates(unit,task,start,states,deadline))
        routes=self.native_routes.get((unit.executor_id,task.task_id),())
        if len(unit.physical_agent_ids)!=1 or not routes:
            return [ExecutionCandidate('unqualified',(),{},status='UNKNOWN',reason='EXECUTION_METHOD_NOT_QUALIFIED')]
        member=unit.physical_agent_ids[0];state=states[member]
        backend=state.get('native_backend',self.native_models.get(member))
        if getattr(backend,'backend_id',None)=='PYTHON_QN_CLOSED_LOOP':
            from qn_aav_simulator.platform_execution import Segment,actual_mode
            from qn_aav_simulator.qn_dynamics import medium_flag
            actual=backend.snapshot()
            if tuple(actual.position)!=tuple(state['position']) or actual_mode(actual.medium_flag)!=state['mode']:
                return [ExecutionCandidate('unknown-state',(),{},status='UNKNOWN',reason='NATIVE_SNAPSHOT_STATE_MISMATCH')]
            idle=max(0.,start-state.get('available_from',0.));idle_trace=()
            if idle:
                waited=self.query_native_idle(backend,idle,self.scene_geometry,deadline)
                if waited['status']!='FEASIBLE':
                    return [ExecutionCandidate('idle-unavailable',(),{},status=waited['status'],reason=waited['reason'])]
                if abs(waited['duration_s']-idle)>1e-6:
                    return [ExecutionCandidate('idle-grid-mismatch',(),{},status='UNKNOWN',reason='NATIVE_IDLE_TIME_GRID_MISMATCH')]
                idle_trace=tuple((state['available_from']+t,p,actual_mode(medium_flag(p[2],backend.constants.hg_m)))
                                 for t,p in waited['trajectory'][:-1])
                backend=waited['terminal_backend'];actual=backend.snapshot()
            alternatives=[]
            for index,route in enumerate(routes):
                name=unit.executor_id+'-'+task.task_id+'-'+str(index)
                if (route.terminal_behavior!='FIXED_REFERENCE' or
                        any(s.operation not in ('ENTER_WATER','WATER_PATH','EXIT_WATER') or s.duration_s<=0
                            for s in route.segments)):
                    alternatives.append(ExecutionCandidate(name,(),{},status='UNKNOWN',reason='QN_METHOD_NOT_QUALIFIED'))
                    continue
                segments=tuple(Segment(s.operation,s.points,s.duration_s) for s in route.segments)
                radius=.25  # existing qn/AAV geometric proxy, also passed to the query
                query=backend.predict_native_fragment(segments,self.scene_geometry,deadline,radius=radius,
                    max_model_time=route.execution_timeout_s,include_state=True,terminal_wait_s=route.terminal_wait_s)
                if query['status']!='FEASIBLE':
                    alternatives.append(ExecutionCandidate(name,(),{},status=query['status'],reason=query['reason']))
                    continue
                summary={k:v for k,v in query.items() if k not in ('terminal_backend','trajectory','source_fingerprint','settled_model_time_s','terminal_wait_s')}
                summary['pre_execution_idle_s']=idle
                trace=idle_trace+((start,actual.position,actual_mode(actual.medium_flag)),)+tuple(
                    (start+t,p,actual_mode(medium_flag(p[2],backend.constants.hg_m)))
                    for t,p in query['trajectory'])
                products=[]
                if route.observation_ids:
                    if self.observation_request is None:
                        alternatives.append(ExecutionCandidate(name,(),{},status='UNKNOWN',reason='OBSERVATION_CONTEXT_MISSING'))
                        continue
                    from qn_aav_simulator.observation_coverage import LocalObservationWindow,ObstacleBox
                    obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene_geometry.objects if kind=='SOLID')
                    window=LocalObservationWindow(self.observation_request,route.observation_ids,member,name,obstacles)
                    for stamp,position,mode in trace:
                        if time.monotonic()>=deadline:raise PlanningBudgetExceeded('observation query budget exhausted')
                        products.extend(window.sample(stamp-start,position,mode,stamp))
                    if window.emitted!=set(route.observation_ids):
                        alternatives.append(ExecutionCandidate(name,(),{},status='INFEASIBLE',reason='REQUIRED_OBSERVATION_NOT_COVERED'))
                        continue
                alternatives.append(ExecutionCandidate(name,
                    (ExecutionStep(unit.executor_id,query['duration_s'],task.target_ref,route,native_prediction=summary),),
                    {member:dict(position=query['terminal_position'],mode=query['terminal_mode'],native_backend=query['terminal_backend'])},
                    motion_traces={member:trace},collision_radii={member:radius},generated_products=tuple(products)))
            return alternatives
        if backend is None or unit.executor_id not in self.native_efforts:
            return [ExecutionCandidate('unknown-state',(),{},status='UNKNOWN',reason='NATIVE_STATE_NOT_AVAILABLE')]
        if getattr(backend,'model',None) not in ('otter','remus100'):
            return [ExecutionCandidate('unqualified-model',(),{},status='UNKNOWN',reason='MODEL_METHOD_NOT_QUALIFIED')]
        actual=backend.snapshot()
        if tuple(actual['position'])!=tuple(state['position']) or actual['actual_mode']!=state['mode']:
            return [ExecutionCandidate('unknown-state',(),{},status='UNKNOWN',reason='NATIVE_SNAPSHOT_STATE_MISMATCH')]
        idle=max(0.,start-state.get('available_from',0.))
        idle_trace=()
        if idle:
            waited=self.query_native_idle(backend,idle,self.scene_geometry,deadline)
            if waited['status']!='FEASIBLE':
                return [ExecutionCandidate('idle-unavailable',(),{},status=waited['status'],reason=waited['reason'])]
            if abs(waited['duration_s']-idle)>1e-6:
                return [ExecutionCandidate('idle-grid-mismatch',(),{},status='UNKNOWN',reason='NATIVE_IDLE_TIME_GRID_MISMATCH')]
            idle_trace=tuple((state['available_from']+t,p,state['mode']) for t,p in waited['trajectory'][:-1])
            backend=waited['terminal_backend']
        alternatives=[]
        from dataclasses import replace
        for index,route in enumerate(routes):
            if route.terminal_behavior!=backend.terminal_behavior:
                alternatives.append(ExecutionCandidate('unqualified-terminal-'+str(index),(),{},
                    status='UNKNOWN',reason='NATIVE_TERMINAL_NOT_QUALIFIED'))
                continue
            expected='SURFACE_PATH' if backend.model=='otter' else 'WATER_PATH'
            if any(s.operation!=expected for s in route.segments):
                alternatives.append(ExecutionCandidate('unqualified-operation-'+str(index),(),{},
                    status='UNKNOWN',reason='NATIVE_OPERATION_NOT_QUALIFIED'))
                continue
            position=tuple(backend.snapshot()['position'])
            if route.segments[0].operation=='SURFACE_PATH':position=(position[0],position[1],0.)
            first=replace(route.segments[0],points=(position,)+route.segments[0].points[1:])
            bound=replace(route,segments=(first,)+route.segments[1:])
            query=self.query_native_fragment(backend,[s.points for s in bound.segments],
                self.native_efforts[unit.executor_id],self.scene_geometry,deadline,
                max_model_time=bound.execution_timeout_s,include_state=True,terminal_wait_s=bound.terminal_wait_s)
            name=unit.executor_id+'-'+task.task_id+'-'+str(index)
            if query['status']!='FEASIBLE':
                alternatives.append(ExecutionCandidate(name,(),{},status=query['status'],reason=query['reason']))
                continue
            summary={k:v for k,v in query.items() if k not in ('terminal_backend','trajectory','source_fingerprint','settled_model_time_s','terminal_wait_s')}
            summary['pre_execution_idle_s']=idle
            terminal=dict(position=query['terminal_position'],mode=query['terminal_mode'],native_backend=query['terminal_backend'])
            alternatives.append(ExecutionCandidate(name,
                (ExecutionStep(unit.executor_id,query['duration_s'],task.target_ref,bound,native_prediction=summary),),
                {member:terminal},motion_traces={member:idle_trace+tuple(
                    (start+t,p,query['terminal_mode']) for t,p in query['trajectory'])},
                collision_radii={member:query['collision_radius_m']}))
        return alternatives

    def iter_execution_candidates(self,unit,task,start,states,deadline):
        if (unit.executor_id,task.task_id) in self.cooperative_routes:
            yield from self._iter_cooperative_candidates(unit,task,start,states,deadline)
        else:
            yield from self.execution_candidates(unit,task,start,states,deadline)

    def _iter_cooperative_candidates(self,unit,task,start,states,deadline):
        """Native work/support alternatives including observation and receipt.

        Only methods backed by full PVS rollouts are currently qualified here.
        Other models remain UNKNOWN, not a distance/speed approximation. Cache
        is invocation-local at this exact state/environment/commitment snapshot.
        """
        from dataclasses import replace
        from qn_aav_simulator.observation_coverage import ObstacleBox,predict_received_products
        choices=self.cooperative_routes[(unit.executor_id,task.task_id)]
        if self.observation_request is None or self.scene_geometry is None or len(self.mother_position)!=3:
            yield ExecutionCandidate('joint-context-missing',(),{},status='UNKNOWN',reason='OBSERVATION_OR_SCENE_CONTEXT_MISSING')
            return
        obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene_geometry.objects if kind=='SOLID')
        cache={}
        for number,routes in enumerate(choices):
            name='joint-'+unit.executor_id+'-'+task.task_id+'-'+str(number)
            if time.monotonic()>=deadline:break
            work=next((e for e in routes if e.executor_id==unit.executor_id),None)
            if work is None or work.physical_agent_ids!=unit.physical_agent_ids:
                raise ValueError('joint method must contain its queried work executor')
            if any(len(e.physical_agent_ids)!=1 for e in routes):
                raise ValueError('native joint fragments currently require individual endpoints')
            participants=[e.physical_agent_ids[0] for e in routes]
            if len(set(participants))!=len(participants):raise ValueError('joint fragments repeat a physical member')
            # This finite method commits its participants together. A later
            # support release shifts this method, not the entire fleet clock.
            # Predict the actual native idle evolution before that release.
            release=max([start]+[max(e.available_from,states[e.physical_agent_ids[0]].get('available_from',0.))
                                 for e in routes])
            offset=release-start
            traces={};evaluated=[];failure=None
            for executor,route in routes.items():
                member=executor.physical_agent_ids[0];state=states[member]
                backend=state.get('native_backend',self.native_models.get(member))
                if state.get('locked',False):
                    failure=('INFEASIBLE','PARTICIPANT_LOCKED');break
                if getattr(backend,'model',None) not in ('otter','remus100'):
                    failure=('UNKNOWN','JOINT_NATIVE_MODEL_NOT_QUALIFIED');break
                actual=backend.snapshot()
                if route.terminal_behavior!=backend.terminal_behavior:
                    failure=('UNKNOWN','NATIVE_TERMINAL_NOT_QUALIFIED');break
                if tuple(actual['position'])!=tuple(state['position']) or actual['actual_mode']!=state['mode']:
                    failure=('UNKNOWN','NATIVE_SNAPSHOT_STATE_MISMATCH');break
                expected='SURFACE_PATH' if backend.model=='otter' else 'WATER_PATH'
                if any(s.operation!=expected for s in route.segments):
                    failure=('UNKNOWN','JOINT_OPERATION_NOT_QUALIFIED');break
                cache_key=(executor.executor_id,route,release)
                if cache_key not in cache:
                    idle=max(0.,release-state.get('available_from',0.))
                    idle_trace=()
                    if idle:
                        waited=self.query_native_idle(backend,idle,self.scene_geometry,deadline)
                        if waited['status']!='FEASIBLE':failure=(waited['status'],waited['reason']);break
                        if abs(waited['duration_s']-idle)>1e-6:
                            failure=('UNKNOWN','NATIVE_IDLE_TIME_GRID_MISMATCH');break
                        idle_trace=tuple((state['available_from']+t,p,state['mode']) for t,p in waited['trajectory'][:-1])
                        backend=waited['terminal_backend']
                    position=tuple(backend.snapshot()['position'])
                    if backend.model=='otter':position=(position[0],position[1],0.)
                    first=replace(route.segments[0],points=(position,)+route.segments[0].points[1:])
                    bound=replace(route,segments=(first,)+route.segments[1:])
                    prefix=cache.get((executor.executor_id,replace(route,terminal_wait_s=0.),release))
                    seed=prefix[1] if prefix is not None and prefix[1]['status']=='FEASIBLE' else None
                    query=self.query_native_fragment(backend,[s.points for s in bound.segments],
                        self.native_efforts[executor.executor_id],self.scene_geometry,deadline,
                        max_model_time=bound.execution_timeout_s,include_state=True,terminal_wait_s=bound.terminal_wait_s,resume=seed)
                    cache[cache_key]=(bound,query,idle_trace)
                bound,query,_=cache[cache_key]
                if query['status']!='FEASIBLE':failure=(query['status'],query['reason']);break
                traces[member]=tuple((t,p,query['terminal_mode']) for t,p in query['trajectory'])
                evaluated.append((executor,bound,query))
            if failure:
                yield ExecutionCandidate(name,(),{},status=failure[0],reason=failure[1]);continue
            # Only reject conflicts inside the actual common activity window.
            # A later selected successor may replace an early member's idle;
            # the complete-plan check supplies only the gaps that remain.
            safety_traces=traces
            radii={e.physical_agent_ids[0]:q['collision_radius_m'] for e,_,q in evaluated}
            horizon=min(q['duration_s'] for _,_,q in evaluated)
            if not failure:
                from bisect import bisect_right
                clocks={m:[r[0] for r in rows] for m,rows in safety_traces.items()}
                for tick in range(int(horizon*100)+1):
                    if time.monotonic()>=deadline:failure=('UNKNOWN','PLANNING_BUDGET_EXHAUSTED');break
                    t=tick/100.;positions={m:rows[max(0,bisect_right(clocks[m],t)-1)][1] for m,rows in safety_traces.items()}
                    if any(math.dist(positions[a],positions[b])<radii[a]+radii[b]+self.scene_geometry.clearance
                           for a in positions for b in positions if a<b):
                        failure=('INFEASIBLE','JOINT_PARTICIPANT_PATH_CONFLICT');break
            if failure:
                yield ExecutionCandidate(name,(),{},status=failure[0],reason=failure[1]);continue
            producer=unit.physical_agent_ids[0]
            work_route=next(route for e,route,_ in evaluated if e.executor_id==unit.executor_id)
            receipt=predict_received_products(self.observation_request,work_route.observation_ids,producer,
                name,traces,self.mother_position,obstacles,deadline)
            if receipt['status']!='FEASIBLE':
                yield ExecutionCandidate(name,(),{},status=receipt['status'],reason=receipt['reason']);continue
            activities=[];terminal_states={}
            for executor,route,query in evaluated:
                member=executor.physical_agent_ids[0];duration=query['duration_s']
                summary={k:v for k,v in query.items() if k not in ('terminal_backend','trajectory','source_fingerprint','settled_model_time_s','terminal_wait_s')}
                summary['nominal_receipt_finish_s']=receipt['receipt_finish_s']
                summary['receipt_prediction_basis']='provided model trace and nominal event identity'
                summary['joint_participant_motion_checked']=True
                summary['pre_execution_idle_s']=release-states[member].get('available_from',0.)
                activities.append(ExecutorPlanItem(executor.executor_id,task.task_id,executor.executor_id,(member,),
                    offset,offset+duration,duration,0.,0.,fulfills_task=executor.executor_id==unit.executor_id,
                    execution_steps=(ExecutionStep(executor.executor_id,duration,task.target_ref,route,native_prediction=summary),)))
                terminal_states[member]=dict(position=query['terminal_position'],mode=query['terminal_mode'],native_backend=query['terminal_backend'])
            complete_traces={}
            for executor,original in routes.items():
                member=executor.physical_agent_ids[0]
                _,query,idle_trace=cache[(executor.executor_id,original,release)]
                complete_traces[member]=idle_trace+tuple((release+t,p,mode) for t,p,mode in traces[member])
            products=tuple(dict(event,generated_at=release+event['generated_at']) for event in receipt['generated_events'])
            yield ExecutionCandidate(name,(),terminal_states,activities=tuple(activities),
                motion_traces=complete_traces,collision_radii=radii,generated_products=products)

    def _check_complete_plan(self,plan,selected,initial_states,deadline):
        """Check known members and all products together before accepting a leaf.

        Evidence stays inside the search. Only real native idle can fill the
        final safety horizon; it never extends a communication commitment.
        This is a sampled nominal-model check, not a tracking-error guarantee.
        """
        from bisect import bisect_right
        from qn_aav_simulator.observation_coverage import ObstacleBox,predict_received_events
        def unknown(reason):return dict(status='UNKNOWN',reason=reason)
        if self.scene_geometry is None:return unknown('PLAN_SCENE_NOT_PROVIDED')
        horizon=plan.makespan;pieces={m:[] for m in initial_states};radii={};products=[];receipt_limits={}
        intervals={m:[] for m in initial_states}
        for candidate,items in selected:
            if time.monotonic()>=deadline:return unknown('PLANNING_BUDGET_EXHAUSTED')
            members={m for item in items for m in item.coalition}
            if set(candidate.motion_traces)!=members or set(candidate.collision_radii)!=members:
                return unknown('PLAN_MOTION_EVIDENCE_MISSING')
            expected={}
            for item in items:
                for member in item.coalition:intervals[member].append((item.planned_start,item.planned_finish))
                step_start=item.planned_start
                for step in item.execution_steps:
                    if step.native_action is not None:
                        for point in step.native_action.observation_ids:
                            for member in item.coalition:expected[(member,point)]=(step_start,step_start+step.duration_s)
                    step_start+=step.duration_s
            witnessed=set()
            for event in candidate.generated_products:
                key=(event.get('producer'),event.get('point_id'))
                if key not in expected or not expected[key][0]<=event['generated_at']<=expected[key][1]:
                    return unknown('PLAN_PRODUCT_ACTIVITY_MISMATCH')
                if (event.get('observed') is not True or event.get('required_bytes')!=32768 or
                        event.get('result',{}).get('model')!='GEOMETRIC_PROXY'):
                    return unknown('PLAN_PRODUCT_EVIDENCE_INVALID')
                ident=event['product_id']
                if ident in receipt_limits:return unknown('PLAN_PRODUCT_ID_REUSED')
                receipt_limits[ident]=expected[key][1];witnessed.add(key);products.append(event)
            if expected.keys()!=witnessed:return unknown('PLAN_OBSERVATION_PRODUCTS_MISSING')
            for member,rows in candidate.motion_traces.items():
                radius=candidate.collision_radii[member]
                if not math.isfinite(radius) or radius<=0 or member in radii and radii[member]!=radius:
                    return unknown('PLAN_MEMBER_GEOMETRY_MISMATCH')
                radii[member]=radius
                if (not rows or any(len(row)!=3 or not math.isfinite(row[0]) or row[0]<0 or
                        len(row[1])!=3 or not all(math.isfinite(v) for v in row[1]) or
                        row[2] not in ('AIR','WATER','SURFACE','TRANSITION') for row in rows) or
                        any(not 0<b[0]-a[0]<=.010001 for a,b in zip(rows,rows[1:]))):
                    return unknown('PLAN_MOTION_SAMPLES_INVALID')
                finish=max(item.planned_finish for item in items if member in item.coalition)
                first=min(item.planned_start for item in items if member in item.coalition)
                terminal=candidate.terminal_states[member]
                if (rows[0][0]>first+1e-6 or abs(rows[-1][0]-finish)>1e-6 or
                        math.dist(rows[-1][1],terminal['position'])>1e-6 or rows[-1][2]!=terminal['mode']):
                    return unknown('PLAN_MOTION_ACTIVITY_BOUNDARY_MISMATCH')
                pieces[member].append((rows,terminal.get('native_backend')))
        traces={};boundaries={0.,horizon}|{stamp for windows in intervals.values() for window in windows for stamp in window}
        for member,state in initial_states.items():
            if time.monotonic()>=deadline:return unknown('PLANNING_BUDGET_EXHAUSTED')
            # A future available state is a terminal forecast, not evidence of
            # where an already committed platform travels before that time.
            if state['available_from']>1e-6:return unknown('PLAN_COMMITTED_PREFIX_MISSING')
            rows=[];backend=state.get('native_backend',self.native_models.get(member))
            for part,terminal_backend in sorted(pieces[member],key=lambda p:p[0][0][0]):
                previous=rows[-1] if rows else (0.,state['position'],state['mode'])
                if (abs(part[0][0]-previous[0])>1e-6 or math.dist(part[0][1],previous[1])>1e-6 or
                        part[0][2]!=previous[2]):
                    return unknown('PLAN_MOTION_GAP_OR_STATE_MISMATCH')
                boundaries.update((part[0][0],part[-1][0]))
                rows.extend(part[1:] if rows else part);backend=terminal_backend
            if not rows:rows=[(0.,state['position'],state['mode'])]
            remaining=horizon-rows[-1][0]
            if remaining>1e-6 or member not in radii:
                is_qn=getattr(backend,'backend_id',None)=='PYTHON_QN_CLOSED_LOOP'
                if not is_qn and getattr(backend,'model',None) not in ('otter','remus100'):
                    return unknown('PLAN_IDLE_MOTION_NOT_QUALIFIED')
                snapshot=backend.snapshot()
                if is_qn:
                    from qn_aav_simulator.platform_execution import actual_mode
                    position=snapshot.position;mode=actual_mode(snapshot.medium_flag)
                else:
                    position=snapshot['position'];mode=snapshot['actual_mode']
                if math.dist(position,rows[-1][1])>1e-6 or mode!=rows[-1][2]:
                    return unknown('PLAN_IDLE_STATE_MISMATCH')
                if member in radii and radii[member]!=backend.collision_radius_m:
                    return unknown('PLAN_MEMBER_GEOMETRY_MISMATCH')
                radii[member]=backend.collision_radius_m
                idle=self.query_native_idle(backend,max(0.,remaining),self.scene_geometry,deadline)
                if idle['status']!='FEASIBLE':return dict(status=idle['status'],reason=idle['reason'])
                origin=rows[-1][0]
                if is_qn:
                    from qn_aav_simulator.qn_dynamics import medium_flag
                    rows.extend((origin+t,p,actual_mode(medium_flag(p[2],backend.constants.hg_m)))
                                for t,p in idle['trajectory'][1:])
                else:
                    rows.extend((origin+t,p,mode) for t,p in idle['trajectory'][1:])
            if rows[-1][0]+1e-6<horizon:return unknown('PLAN_MOTION_HORIZON_MISSING')
            traces[member]=tuple(rows)
        clocks={member:[row[0] for row in rows] for member,rows in traces.items()}
        # Include exact activity boundaries and the endpoint, even when they
        # are not an integer number of 10 ms samples.
        times=sorted(boundaries|{tick/100. for tick in range(int(horizon*100)+1)})
        for stamp in times:
            if time.monotonic()>=deadline:return unknown('PLANNING_BUDGET_EXHAUSTED')
            positions={}
            for member,rows in traces.items():
                index=bisect_right(clocks[member],stamp+1e-8)-1
                if index<0 or stamp-rows[index][0]>.010001:return unknown('PLAN_MOTION_SAMPLES_MISSING')
                position=rows[index][1];positions[member]=position
                reason=self.scene_geometry.violation(position,radii[member])
                if reason:return dict(status='INFEASIBLE',reason=reason)
            if any(math.dist(positions[a],positions[b])<radii[a]+radii[b]+self.scene_geometry.clearance
                    for a in positions for b in positions if a<b):
                return dict(status='INFEASIBLE',reason='PLAN_MEMBER_PATH_CONFLICT')
        if products:
            if len(self.mother_position)!=3:return unknown('PLAN_RECEIVER_NOT_PROVIDED')
            obstacles=tuple(ObstacleBox(c,s) for _,kind,c,s in self.scene_geometry.objects if kind=='SOLID')
            receipt=predict_received_events(products,traces,intervals,self.mother_position,obstacles,deadline)
            if receipt['status']!='FEASIBLE':return receipt
            if any(stamp>receipt_limits[key]+1e-6 for key,stamp in receipt['received_at'].items()):
                return dict(status='INFEASIBLE',reason='PLAN_RECEIPT_AFTER_PRODUCER_TERMINAL')
        return dict(status='FEASIBLE',reason='NOMINAL_COMPLETE_PLAN_MOTION_AND_CAPACITY')

    @staticmethod
    def query_swarm_reference(parameter_namespace,request,deadline):
        """Call the native optimizer in a disposable, non-publishing process.

        The supplied static map and peer references are explicit query inputs.
        This predicts a nominal reference, not actual qn tracking or task
        completion. The same caller deadline includes serialization and wait.
        """
        import json,shutil
        if not math.isfinite(deadline):raise ValueError('finite query deadline required')
        if shutil.which('rosrun') is None:return dict(status='UNKNOWN',reason='ROS_SWARM_QUERY_UNAVAILABLE')
        with tempfile.TemporaryDirectory(prefix='swarm-query-') as directory:
            source=os.path.join(directory,'request.json');destination=os.path.join(directory,'result.json')
            with open(source,'w') as stream:json.dump(request,stream,allow_nan=False)
            if time.monotonic()>=deadline:return dict(status='UNKNOWN',reason='QUERY_BUDGET_EXHAUSTED')
            with tempfile.TemporaryFile() as log:
                process=subprocess.Popen(['rosrun','ego_planner','swarm_readonly_query',parameter_namespace,source,destination],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    try:process.wait(timeout=max(.001,deadline-time.monotonic()))
                    except subprocess.TimeoutExpired:return dict(status='UNKNOWN',reason='QUERY_BUDGET_EXHAUSTED')
                    if not os.path.exists(destination):
                        log.seek(max(0,log.tell()-2000))
                        return dict(status='UNKNOWN',reason='SWARM_QUERY_PROCESS_FAILED',detail=log.read().decode(errors='replace'))
                    with open(destination) as stream:result=json.load(stream)
                    if result.get('status')=='FEASIBLE':
                        if process.returncode!=0:return dict(status='UNKNOWN',reason='SWARM_QUERY_ABNORMAL_EXIT')
                        result['duration_s']=float(result['duration_s'])
                        result['max_speed_mps']=float(result['max_speed_mps'])
                        result['durations']=[float(t) for t in result['durations']]
                        result['coefficients']=[[[float(v) for v in row] for row in piece] for piece in result['coefficients']]
                        result['formation']=str(result['formation']).lower()=='true'
                        result['formation_nodes']=int(result['formation_nodes'])
                        if 'reference_samples' in result:
                            result['reference_samples']=[tuple(float(v) for v in row) for row in result['reference_samples']]
                            result['reference_sample_period_s']=float(result['reference_sample_period_s'])
                    if time.monotonic()>=deadline:return dict(status='UNKNOWN',reason='QUERY_BUDGET_EXHAUSTED')
                    return result
                finally:
                    try:os.killpg(process.pid,signal.SIGTERM)
                    except ProcessLookupError:pass
                    try:process.wait(timeout=.2)
                    except subprocess.TimeoutExpired:
                        try:os.killpg(process.pid,signal.SIGKILL)
                        except ProcessLookupError:pass
                        process.wait()

    @staticmethod
    def query_air_reference(backend,reference,scene,deadline,*,position_tolerance=.5,
                            terminal_speed=.25,hold_duration=4.,max_model_time=180.,include_state=False):
        """Read-only qn rollout of one native Swarm command sequence and hold.

        The original trajectory server supplies the yaw samples. This checks
        the provided reference, not future online replans or communication
        delay. Actual Action adoption and safety remain separate requirements.
        """
        from qn_aav_simulator.contracts import ControlCmd,CommandMode,PlatformAdapterCmd,PlantStepInput
        from qn_aav_simulator.platform_execution import actual_mode
        from qn_aav_simulator.qn_dynamics import medium_flag
        if not math.isfinite(deadline) or any(not math.isfinite(v) or v<=0 for v in
                (position_tolerance,terminal_speed,hold_duration,max_model_time)):
            raise ValueError('finite AIR query deadline and positive limits required')
        if getattr(backend,'backend_id',None)!='PYTHON_QN_CLOSED_LOOP' or backend.reference_mode!='ROUTE_POSITION':
            return dict(status='UNKNOWN',reason='AIR_MODEL_NOT_QUALIFIED')
        source=backend.snapshot();state=source;t=0.;radius=.25
        def reply(status,reason):
            return dict(status=status,reason=reason,duration_s=t,terminal_position=state.position,
                terminal_mode=actual_mode(state.medium_flag),geometry_checked=scene is not None,
                collision_radius_m=radius,scope='provided native reference and sampled qn model; online replans not certified')
        if time.monotonic()>=deadline:return reply('UNKNOWN','PLANNING_BUDGET_EXHAUSTED')
        if scene is None:return reply('UNKNOWN','SCENE_GEOMETRY_REQUIRED')
        if actual_mode(source.medium_flag)!='AIR':return reply('INFEASIBLE','AIR_ENTRY_MEDIUM_MISMATCH')
        if reference.get('status')!='FEASIBLE':return reply('UNKNOWN','NATIVE_REFERENCE_NOT_AVAILABLE')
        rows=reference.get('reference_samples',());dt=reference.get('reference_sample_period_s')
        duration=reference.get('duration_s')
        if (dt!=.01 or not isinstance(duration,(int,float)) or not math.isfinite(duration) or duration<=0 or
                not rows or any(len(row)!=5 or not all(math.isfinite(v) for v in row) or
                    abs(row[0]-i*dt)>1e-8 for i,row in enumerate(rows)) or
                rows[-1][0]<duration or rows[-1][0]-duration>dt+1e-8):
            return reply('UNKNOWN','NATIVE_COMMAND_SAMPLES_MISSING_OR_INVALID')
        if math.dist(source.position,rows[0][1:4])>position_tolerance:
            return reply('INFEASIBLE','AIR_REFERENCE_ENTRY_MISMATCH')
        candidate=copy.deepcopy(backend);end=tuple(rows[-1][1:4]);settled=None
        trajectory=[(0.,source.position,'AIR')]
        for tick in range(int(max_model_time/dt)+1):
            if time.monotonic()>=deadline:return reply('UNKNOWN','PLANNING_BUDGET_EXHAUSTED')
            if actual_mode(state.medium_flag)!='AIR':return reply('INFEASIBLE','ACTUAL_MEDIUM_OUTSIDE_AIR_PHASE')
            reason=scene.violation(state.position,radius)
            if reason:return reply('INFEASIBLE',reason)
            if t>=duration and math.dist(state.position,end)<=position_tolerance and math.sqrt(sum(v*v for v in state.velocity))<=terminal_speed:
                if settled is None:settled=t
            else:settled=None
            if settled is not None and t-settled>=hold_duration:
                candidate.retain_idle_reference()
                result=reply('FEASIBLE','NATIVE_AIR_REFERENCE_AND_TERMINAL')
                if include_state:result.update(terminal_backend=candidate,trajectory=tuple(trajectory))
                return result
            if tick==int(max_model_time/dt):break
            row=rows[min(tick,len(rows)-1)]
            cmd=ControlCmd('read-only-air',source.agent_id,t,CommandMode.DESIRED_POSITION,(0.,0.,0.),
                desired_position=tuple(row[1:4]),desired_yaw_rad=row[4])
            step=candidate.step(PlantStepInput(state,cmd,PlatformAdapterCmd(source.agent_id),dt,2.,8.))
            for position in step.diagnostics['model_positions']:
                reason=scene.violation(position,radius)
                if reason:return reply('INFEASIBLE',reason)
                if actual_mode(medium_flag(position[2],candidate.constants.hg_m))!='AIR':
                    return reply('INFEASIBLE','ACTUAL_MEDIUM_OUTSIDE_AIR_PHASE')
            t=(tick+1)*dt;state=candidate.snapshot(t)
            if include_state:trajectory.append((t,state.position,'AIR'))
        return reply('UNKNOWN','MODEL_HORIZON_EXHAUSTED')

    @staticmethod
    def query_native_fragment(backend,paths,effort,scene,deadline,dt=.01,
                              terminal_speed=.03,hold_duration=4.,max_model_time=180.,include_state=False,terminal_wait_s=0.,resume=None):
        """Motion query using a supplied native state snapshot, including coast.

        No endpoint or controller factory is introduced. Unknown/horizon/budget
        outcomes remain distinct from a witnessed geometric infeasibility.
        """
        return backend.predict_native_fragment(paths,effort,scene,deadline,dt,
            terminal_speed,hold_duration,max_model_time,include_state,terminal_wait_s,resume)

    @staticmethod
    def query_native_idle(backend,duration,scene,deadline,dt=.01):
        if (getattr(backend,'backend_id',None)!='PYTHON_QN_CLOSED_LOOP' and
                getattr(backend,'model',None) not in ('otter','remus100')):
            return dict(status='UNKNOWN',reason='NATIVE_IDLE_MODEL_NOT_QUALIFIED',duration_s=0.)
        return backend.predict_idle(duration,scene,deadline,dt)

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
                        hard_deadlines: bool = False,
                        execution_candidates=None, member_states=None) -> ExecutorPlan:
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
    if execution_candidates is not None:
        if deadline is None:raise ValueError('complete candidate search requires an invocation budget')
        return _build_complete_candidate_plan(executors,tasks,execution_candidates,
            member_states,deadline,serial,serial_units,precedence_edges,hard_deadlines)
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
                if hard_deadlines and task.deadline is not None and finish>task.deadline:
                    continue
                introduced_makespan = max(0.0, finish - makespan)
                urgent = task.deadline is not None and task.deadline < start + 1.55 * task.service_time
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


def _build_complete_candidate_plan(executors,tasks,provider,member_states,deadline,
                                   serial,serial_units,precedence_edges,hard_deadlines):
    """Finite complete-plan enumeration inside the existing scheduler.

    The motion provider evaluates complete chains from a physical-member
    snapshot. It owns mode/transition qualification and must include all motion,
    work, terminal and return costs; this search never substitutes distance/v.
    """
    validate_executor_inputs(executors,tasks)
    predecessors=checked_predecessors(tasks,precedence_edges)
    members={m for e in executors for m in e.physical_agent_ids}
    if member_states is None or set(member_states)!=members:
        raise ValueError('candidate search requires every physical member state')
    states=copy.deepcopy(member_states)
    for member,value in states.items():
        if (len(value.get('position',()))!=3 or not all(math.isfinite(v) for v in value['position'])
                or value.get('mode') not in ('AIR','WATER','SURFACE','TRANSITION')):
            raise ValueError('invalid physical predicted state: '+member)
        nonnegative(value.get('available_from',0.),'member availability')
        value['available_from']=value.get('available_from',0.)
    participants=set(serial_units) if serial_units is not None else {e.executor_id for e in executors}
    unit_members={e.executor_id:set(e.physical_agent_ids) for e in executors}
    edges=tuple((p,t) for t,values in predecessors.items() for p in sorted(values))
    best=None;count=0;complete=True;unknown=False
    # Abstract callable oracles remain scheduling-only. The real native
    # provider always validates complete physical evidence; missing evidence
    # is UNKNOWN, never permission to fall back to the abstract path.
    checker=getattr(getattr(provider,'__self__',None),'_check_complete_plan',None)
    last_check_reason=''

    def continuations(node):
        nonlocal count,unknown
        items,remaining,snapshot,finishes,serial_release,evidence=node
        for task in sorted(remaining,key=lambda t:t.task_id):
            if not predecessors[task.task_id]<=finishes.keys():continue
            for unit in eligible_executors(executors,task):
                if any(snapshot[m].get('locked',False) for m in unit.physical_agent_ids):continue
                start=max(max(snapshot[m]['available_from'] for m in unit.physical_agent_ids),
                    unit.available_from,
                    max((finishes[p] for p in predecessors[task.task_id]),default=0.),
                    serial_release if serial and unit.executor_id in participants else 0.)
                with contextlib.closing(bounded_candidate_query(provider,
                        (unit,task,start,copy.deepcopy(snapshot),deadline),deadline)) as alternatives:
                    for alternative in alternatives:
                        if time.monotonic()>=deadline:
                            raise PlanningBudgetExceeded('planning budget exhausted')
                        count+=1
                        if not isinstance(alternative,ExecutionCandidate):raise ValueError('typed execution candidate required')
                        if alternative.status!='FEASIBLE':
                            unknown |= alternative.status=='UNKNOWN';continue
                        if alternative.activities:
                            # Reuse PlanItem rather than introducing a second
                            # workflow/coalition envelope. Times are method-local.
                            from dataclasses import replace
                            if serial:
                                unknown=True;continue  # serial worker cannot own concurrent activities
                            if any(not isinstance(a,ExecutorPlanItem) or a.task_id!=task.task_id or
                                   a.status!='PLANNED' or a.actual_finish is not None
                                   for a in alternative.activities):
                                raise ValueError('invalid cooperative activity template')
                            if not any(a.fulfills_task and a.executor_id==unit.executor_id for a in alternative.activities):
                                raise ValueError('queried work executor absent from cooperative candidate')
                            added=[replace(a,execution_id='exec-{:04d}-{}-{}'.format(len(items),task.task_id,a.execution_id),
                                planned_start=start+a.planned_start,planned_finish=start+a.planned_finish,
                                candidate_id=alternative.candidate_id) for a in alternative.activities]
                            added.sort(key=lambda a:(a.planned_start,a.execution_id))
                            releases={};firsts={}
                            for activity in added:
                                for member in activity.coalition:
                                    releases[member]=max(releases.get(member,0.),activity.planned_finish)
                                    firsts[member]=min(firsts.get(member,float('inf')),activity.planned_start)
                            if set(alternative.terminal_states)!=set(releases) or not set(releases)<=members:
                                raise ValueError('cooperative terminal states must match participating physical members')
                            if any(snapshot[m].get('locked',False) or firsts[m]<snapshot[m]['available_from'] for m in releases):
                                continue
                            finish=max(a.planned_finish for a in added)
                            if hard_deadlines and task.deadline is not None and finish>task.deadline:continue
                            prefix=items+added
                            selected={a.task_id for a in prefix}
                            try:
                                validate_executor_plan(ExecutorPlan(prefix,False,
                                    tuple((p,t) for p,t in edges if p in selected and t in selected)),executors,
                                    [t for t in tasks if t.task_id in selected])
                            except ValueError:
                                continue  # reject a conflicting candidate, not the accepted prefix
                            next_states=copy.deepcopy(snapshot)
                            for member,terminal in alternative.terminal_states.items():
                                if (len(terminal.get('position',()))!=3 or not all(math.isfinite(v) for v in terminal['position'])
                                        or terminal.get('mode') not in ('AIR','WATER','SURFACE')):
                                    raise ValueError('invalid cooperative terminal state')
                                next_states[member]=copy.deepcopy(terminal)
                                next_states[member]['available_from']=releases[member]
                                last=max((a for a in added if member in a.coalition),key=lambda a:a.planned_finish)
                                last.predicted_member_states=dict(last.predicted_member_states)
                                last.predicted_member_states[member]={k:terminal[k] for k in ('position','mode')}
                            next_finishes=dict(finishes);next_finishes[task.task_id]=finish
                            yield (prefix,tuple(t for t in remaining if t.task_id!=task.task_id),next_states,next_finishes,
                                   serial_release,evidence+[(alternative,tuple(added))])
                            continue
                        if set(alternative.terminal_states)!=set(unit.physical_agent_ids):
                            raise ValueError('candidate must update exactly its physical members')
                        if any(unit_members.get(step.executor_id)!=set(unit.physical_agent_ids) for step in alternative.steps):
                            raise ValueError('candidate step must use the reserved physical members')
                        finish=start+alternative.duration_s
                        nonnegative(finish,'candidate finish')
                        if hard_deadlines and task.deadline is not None and finish>task.deadline:continue
                        next_states=copy.deepcopy(snapshot)
                        published={}
                        for member,terminal in alternative.terminal_states.items():
                            if (len(terminal.get('position',()))!=3 or not all(math.isfinite(v) for v in terminal['position'])
                                    or terminal.get('mode') not in ('AIR','WATER','SURFACE')):
                                raise ValueError('invalid candidate terminal state')
                            next_states[member]=copy.deepcopy(terminal)
                            next_states[member]['available_from']=finish
                            published[member]={k:terminal[k] for k in ('position','mode')}
                        step=alternative.steps[0] if len(alternative.steps)==1 else None
                        service=step.service_time_s if step else 0.
                        item=ExecutorPlanItem('exec-{:04d}-{}'.format(len(items),task.task_id),task.task_id,
                            unit.executor_id,unit.physical_agent_ids,start,finish,alternative.duration_s-service,0.,service,
                            execution_steps=alternative.steps,candidate_id=alternative.candidate_id,
                            predicted_member_states=published)
                        next_finishes=dict(finishes);next_finishes[task.task_id]=finish
                        next_remaining=tuple(t for t in remaining if t.task_id!=task.task_id)
                        yield (items+[item],next_remaining,next_states,next_finishes,
                               finish if serial and unit.executor_id in participants else serial_release,
                               evidence+[(alternative,(item,))])

    # Resume alternatives lazily: complete one feasible assignment chain before
    # spending the invocation budget on sibling methods. Same finite search,
    # explicit iterator stack (no recursion depth or new online solver).
    frontier=[iter([([],tuple(tasks),states,{},0.,[])])]
    try:
        while frontier:
            if time.monotonic()>=deadline:complete=False;break
            try:
                node=next(frontier[-1])
            except StopIteration:
                frontier.pop();continue
            except PlanningBudgetExceeded:
                complete=False;unknown=True;break
            items,remaining,_,_,_,evidence=node
            if remaining:
                frontier.append(continuations(node))
            else:
                candidate_plan=ExecutorPlan(items,serial,edges)
                validate_executor_plan(candidate_plan,executors,tasks)
                physical=checker is not None or any(step.native_action is not None
                    for item in items for step in item.execution_steps) or any(c.motion_traces for c,_ in evidence)
                if physical:
                    if checker is None:
                        unknown=True;last_check_reason='COMPLETE_PLAN_CHECKER_MISSING';continue
                    try:
                        check=bounded_travel_query(checker,(candidate_plan,evidence,states,deadline),deadline)
                    except PlanningBudgetExceeded:
                        complete=False;unknown=True;break
                    if check['status']!='FEASIBLE':
                        unknown |= check['status']=='UNKNOWN';last_check_reason=check['reason'];continue
                    candidate_plan.validation_scope=check['reason']
                if best is None or candidate_plan.makespan<best.makespan:best=candidate_plan
    finally:
        for iterator in reversed(frontier):
            close=getattr(iterator,'close',None)
            if close is not None:close()
    if best is None:
        if not complete:raise PlanningBudgetExceeded('no complete feasible candidate within shared budget')
        raise ValueError('no complete candidate found'+(' (some mode/motion queries remain unknown)' if unknown else '')+
                         ('; '+last_check_reason if last_check_reason else '')+
                         '; not a proof of mathematical infeasibility')
    best.search_complete=complete and not unknown
    best.evaluated_candidates=count
    return best


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
        if item.execution_id in execution_ids:
            raise ValueError("plan execution_id must be unique")
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
        if type(item.fulfills_task) is not bool:raise ValueError('fulfills_task must be boolean')
        if item.fulfills_task and executor not in eligible_executors(executors, task):
            raise ValueError("executor {} is not eligible for task {}".format(
                item.executor_id, item.task_id))
        if item.coalition != executor.physical_agent_ids:
            raise ValueError("plan coalition must equal the unit's fixed membership")
        if item.execution_steps:
            for step in item.execution_steps:
                if step.executor_id not in by_id or by_id[step.executor_id].physical_agent_ids!=item.coalition:
                    raise ValueError('step must retain its activity physical members')
            duration=sum(s.duration_s for s in item.execution_steps)
            if not math.isclose(duration,item.travel_time+item.service_time,abs_tol=1e-6):
                raise ValueError('activity duration differs from selected steps')
        for name in ("planned_start", "planned_finish", "travel_time", "wait_time",
                     "service_time"):
            nonnegative(getattr(item, name), name)
        if not item.fulfills_task and not item.execution_steps:
            raise ValueError('support activity requires a checked execution method')
        if item.wait_time>0:
            prediction=item.native_prediction
            if (prediction.get('status')!='FEASIBLE' or not prediction.get('geometry_checked') or
                    prediction.get('pre_execution_idle_s',-1)!=item.wait_time):
                raise ValueError('planned wait requires qualified native idle prediction')
        expected = item.planned_start + item.travel_time + item.wait_time + item.service_time
        if not math.isclose(item.planned_finish, expected, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("planned_finish must equal start + travel + wait + service")
        if not item.execution_steps and item.service_time != task.service_time:
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
    if task_ids != set(task_by_id) or {i.task_id for i in plan.items if i.fulfills_task}!=task_ids:
        raise ValueError("plan must cover each supplied task with a qualified work activity")
    checked_predecessors(tasks,plan.precedence_edges)
    required_edges={(p,t.task_id) for t in tasks for p in t.predecessors}
    from dataclasses import replace
    view=replace(plan,precedence_edges=tuple(set(plan.precedence_edges)|required_edges))
    before_by_activity=activity_predecessors(view)
    item_by_id={i.execution_id:i for i in plan.items}
    for ident,values in before_by_activity.items():
        if any(item_by_id[ident].planned_start<item_by_id[p].planned_finish-1e-6 for p in values):
            raise ValueError('plan violates a task/resource/motion predecessor')
