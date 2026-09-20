import copy
import time
from dataclasses import dataclass

from mrta_python import Executor,Task,build_executor_plan
from mrta_python.models import ExecutionCandidate,ExecutionStep


@dataclass
class CandidateOracle:
    case: str

    def __call__(self,unit,task,start,states,deadline):
        member=unit.physical_agent_ids[0]
        previous=states[member]
        if self.case in ('conversion','ignore-conversion'):
            if unit.executor_id=='aav':
                conversion=10. if self.case=='conversion' else 0.
                durations=[2.,conversion,4.,conversion,2.]
                mode='AIR'
            else:durations=[5.,4.,10.,5.];mode='WATER'
            return [ExecutionCandidate(unit.executor_id,
                tuple(ExecutionStep(unit.executor_id,d,task.target_ref) for d in durations),
                {member:dict(position=(0.,0.,.8 if mode=='AIR' else -2.),mode=mode)})]
        if self.case=='budget' and unit.executor_id=='b':time.sleep(1.)
        if self.case=='unknown' and unit.executor_id=='b':
            return [ExecutionCandidate('unqualified',(),{},status='UNKNOWN',reason='MODE_NOT_QUALIFIED')]
        if self.case=='order':
            duration=(100. if previous['position'][0]==1 else 2.) if task.task_id=='B' else 1.
        elif self.case=='mode':
            duration=1. if task.task_id=='A' else 5. if previous['mode']=='WATER' else 100.
        else:duration=1.
        mode='WATER' if self.case=='mode' and task.task_id=='A' else 'AIR'
        return [ExecutionCandidate(unit.executor_id+'-'+task.task_id,
            (ExecutionStep(unit.executor_id,duration,task.target_ref),),
            {member:dict(position=(1. if task.task_id=='A' else 2.,0.,.8 if mode=='AIR' else -2.),mode=mode)})]


def state(mode='AIR',available=0.):return dict(position=(0.,0.,0.),mode=mode,available_from=available)
def task(name,**kw):return Task(name,frozenset({'AIR'}),1,0.,2000.,name,**kw)


def plan(units,tasks,states,oracle,**kw):
    return build_executor_plan(units,tasks,lambda *args:0.,initial_target_ref='start',
        execution_candidates=oracle,member_states=states,budget_s=kw.pop('budget_s',3.),**kw)


def test_complete_plan_selection_avoids_greedy_first_task_trap():
    states={'x':state()};before=copy.deepcopy(states)
    result=plan([Executor('e',('x',),frozenset({'AIR'}))],[task('A'),task('B')],states,CandidateOracle('order'))
    assert [i.task_id for i in result.items]==['B','A']
    assert result.makespan==3. and result.search_complete
    assert states==before


def test_successor_queries_see_updated_physical_mode_and_position():
    result=plan([Executor('e',('x',),frozenset({'AIR'}))],
        [task('A'),task('B',predecessors=('A',))],{'x':state()},CandidateOracle('mode'))
    assert result.makespan==6.
    assert result.items[0].predicted_member_states['x']['mode']=='WATER'
    assert result.items[1].planned_start==1.


def test_unused_late_unit_does_not_block_serial_start():
    units=[Executor('a',('x',),frozenset({'AIR'})),Executor('late',('y',),frozenset({'AIR'}),1000.)]
    result=plan(units,[task('A',required_members=('x',))],{'x':state(),'y':state(available=1000.)},CandidateOracle('simple'),serial=True)
    assert result.items[0].planned_start==0.


def test_budget_preserves_already_found_complete_candidate():
    units=[Executor('a',('x',),frozenset({'AIR'})),Executor('b',('y',),frozenset({'AIR'}))]
    result=plan(units,[task('A')],{'x':state(),'y':state()},CandidateOracle('budget'),budget_s=.5)
    assert result.items[0].executor_id=='a' and result.makespan==1.
    assert result.search_complete is False


def test_unknown_mode_is_not_mislabeled_as_proven_optimal():
    units=[Executor('a',('x',),frozenset({'AIR'})),Executor('b',('y',),frozenset({'AIR'}))]
    result=plan(units,[task('A')],{'x':state(),'y':state()},CandidateOracle('unknown'))
    assert result.items[0].executor_id=='a' and result.search_complete is False


def test_conversion_and_return_costs_change_selected_platform():
    # Synthetic oracle costs isolate scheduler semantics, not dynamics evidence.
    units=[Executor('aav',('aircraft',),frozenset({'AIR'})),Executor('uuv',('underwater',),frozenset({'AIR'}))]
    states={'aircraft':state(),'underwater':state('WATER')}
    full=plan(units,[task('A')],states,CandidateOracle('conversion'))
    omitted=plan(units,[task('A')],states,CandidateOracle('ignore-conversion'))
    assert full.items[0].executor_id=='uuv' and full.makespan==24.
    assert omitted.items[0].executor_id=='aav' and omitted.makespan==8.
    assert len(full.items[0].execution_steps)==4


def test_unselected_late_method_cannot_override_authoritative_member_state():
    units=[Executor('air',('x',),frozenset({'AIR'})),Executor('native',('x',),frozenset({'WATER'}),1000.)]
    result=plan(units,[task('A')],{'x':state()},CandidateOracle('simple'),serial=True)
    assert result.items[0].planned_start==0.
