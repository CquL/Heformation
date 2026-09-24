import time
import pytest
from mrta_python import Executor,Task,build_executor_plan
from mrta_python.executors import checked_predecessors, PlanningBudgetExceeded
from mrta_python import DelayEvent,ExecutorPlan,ExecutorPlanItem
from mrta_python.repair import process_executor_completion


def instantaneous(*args):
    return 0.


def hung(*args):
    time.sleep(60)
    return 0.


def tasks():
    return [Task('a',frozenset({'AIR'}),1,1.,100.,'p'),
            Task('b',frozenset({'WATER'}),1,1.,100.,'p',predecessors=('a',))]


def test_motion_order_can_close_a_cycle_even_when_task_graph_is_acyclic():
    assert checked_predecessors(tasks())['b']=={'a'}
    with pytest.raises(ValueError,match='cycle'):
        checked_predecessors(tasks(),[('b','a')])


def test_precedence_is_respected_between_disjoint_physical_members():
    units=[Executor('air',('aav',),frozenset({'AIR'})),
           Executor('water',('uuv',),frozenset({'WATER'}))]
    plan=build_executor_plan(units,tasks(),instantaneous,initial_target_ref='base')
    assert plan.items[1].planned_start>=plan.items[0].planned_finish


def test_hung_query_is_bounded_and_never_returns_partial_plan():
    before=time.monotonic()
    with pytest.raises(PlanningBudgetExceeded):
        build_executor_plan([Executor('air',('a',),frozenset({'AIR'}))],tasks()[:1],hung,
                            initial_target_ref='base',budget_s=.3)
    assert time.monotonic()-before<2.


def test_nested_validation_cannot_clear_outer_physical_overlap():
    units=[Executor('a',('same',),frozenset({'AIR'})),Executor('b',('same',),frozenset({'WATER'}))]
    independent=[Task('air',frozenset({'AIR'}),1,1.,100.,'p'),Task('water',frozenset({'WATER'}),1,1.,100.,'p')]
    def query(*args):
        build_executor_plan([Executor('other',('other',),frozenset({'AIR'}))],independent[:1],instantaneous,initial_target_ref='base')
        return 0.
    plan=build_executor_plan(units,independent,query,initial_target_ref='base')
    assert plan.items[1].planned_start>=plan.items[0].planned_finish


def test_parallel_result_keeps_other_running_commitments_and_repairs_successor():
    first=ExecutorPlanItem('e1','a','u1',('aav',),0.,2.,0.,0.,2.,'RUNNING')
    running=ExecutorPlanItem('e2','b','u2',('uuv',),0.,10.,0.,0.,10.,'RUNNING')
    next_=ExecutorPlanItem('e3','c','u1',('aav',),2.,4.,0.,0.,2.)
    plan=ExecutorPlan([first,running,next_],serial=False,precedence_edges=(('a','c'),))
    events={}
    event=DelayEvent('goal1','e1','a',2.,3.)
    updated,changed=process_executor_completion(plan,event,events)
    assert changed
    assert updated.items[1]==running
    assert updated.items[2].planned_start==3.
    assert plan.items[0].status=='RUNNING'
    duplicate,changed=process_executor_completion(updated,event,events)
    assert duplicate is updated and not changed


def test_parallel_delay_does_not_shift_an_unrelated_unstarted_platform():
    air=ExecutorPlanItem('air-work','air','aav',('drone_0',),0.,2.,0.,0.,2.,'RUNNING')
    water=ExecutorPlanItem('water-work','water','uuv',('uuv',),0.,10.,0.,0.,10.)
    follow=ExecutorPlanItem('air-follow','follow','aav',('drone_0',),2.,4.,0.,0.,2.)
    plan=ExecutorPlan([air,water,follow],serial=False,precedence_edges=(('air','follow'),))
    updated,changed=process_executor_completion(plan,
        DelayEvent('air-goal','air-work','air',2.,3.),{})
    assert changed
    assert updated.items[1].planned_start==0. and updated.items[1].planned_finish==10.
    assert updated.items[2].planned_start==3. and updated.items[2].planned_finish==5.


def test_late_result_moves_unstarted_cooperative_roles_together():
    from mrta_python.executors import activity_predecessors
    air_support=ExecutorPlanItem('air-support','air','usv',('usv',),0.,14.,14.,0.,0.,
        'RUNNING',candidate_id='air-method',fulfills_task=False)
    air_work=ExecutorPlanItem('air-work','air','aav',('drone_0',),0.,24.,24.,0.,0.,
        'RUNNING',candidate_id='air-method')
    water_support=ExecutorPlanItem('water-support','water','usv',('usv',),14.,317.,303.,0.,0.,
        candidate_id='water-method',fulfills_task=False)
    water_work=ExecutorPlanItem('water-work','water','uuv',('uuv',),14.,266.,252.,0.,0.,
        candidate_id='water-method')
    plan=ExecutorPlan([air_support,air_work,water_support,water_work],serial=False)
    updated,changed=process_executor_completion(plan,
        DelayEvent('support-goal','air-support','air',14.,16.27),{})
    assert changed
    assert updated.items[2].planned_start==updated.items[3].planned_start==16.27
    assert updated.items[2].planned_finish==319.27
    assert updated.items[3].planned_finish==268.27
    assert updated.items[1]==air_work and plan.items[2].planned_start==14.
    activity_predecessors(updated)
