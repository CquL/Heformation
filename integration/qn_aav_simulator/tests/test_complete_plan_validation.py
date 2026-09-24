"""Synthetic sampled-query oracles isolate global checks, not native GNC.

Each per-method path/product is feasible alone. The same physical evidence
must be rejected when joint motion or shared channel capacity makes it fail.
"""
from dataclasses import asdict,dataclass

import pytest

from mrta_python import Executor,Task,build_executor_plan
from mrta_python.executors import ExecutorTravelTimeProvider
from mrta_python.models import ExecutionCandidate,ExecutionStep,NativeActionSpec,NativeSegmentSpec
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry


@dataclass(frozen=True)
class SampledProvider(ExecutorTravelTimeProvider):
    case: str = 'crossing'

    def execution_candidates(self,unit,task,start,states,deadline):
        member=unit.physical_agent_ids[0]
        origin=states[member]['position']
        duration=2. if self.case=='crossing' else 1.5
        target=(-origin[0],0.,0.) if self.case=='crossing' else origin
        trace=tuple((start+i/100.,tuple(a+(b-a)*i/(duration*100.) for a,b in zip(origin,target)),'SURFACE')
                    for i in range(round(duration*100)+1))
        products=()
        observes=(task.task_id,) if self.case=='capacity' else ()
        native=NativeActionSpec((NativeSegmentSpec('SURFACE_PATH',(origin,target)),),'TRIM_PROPULSION',
                                observation_ids=observes)
        if observes:
            products=(dict(product_id=task.task_id+':p',request_id='sampled',goal_id=task.task_id,
                point_id=task.task_id,producer=member,generated_at=start,observed=True,
                result=dict(model='GEOMETRIC_PROXY',dwell_s=1.),required_bytes=32768),)
        return [ExecutionCandidate(task.task_id,(ExecutionStep(unit.executor_id,duration,task.target_ref,native),),
            {member:dict(position=target,mode='SURFACE')},
            motion_traces={} if self.case=='missing' else {member:trace},
            collision_radii={member:.1},generated_products=products)]


def planned(case,members,require_return=False,opaque_position=None,opaque_horizon=2.):
    scene=StaticSceneGeometry('world',-10.,.1,())
    states={m:dict(position=((-0.25 if m=='a' else 0.25) if case=='near' else
                             (-2. if m=='a' else 2.),0.,0.),
                   mode='SURFACE',available_from=0.) for m in members}
    if opaque_position is not None:
        states['held']=dict(position=opaque_position,mode='AIR',available_from=0.,locked=True,
            opaque_hold_radius_m=.5,collision_radius_m=.25,
            opaque_hold_horizon_s=opaque_horizon)
    units=[Executor(m,(m,),frozenset({'SURFACE'})) for m in members]
    tasks=[Task(m,frozenset({'SURFACE'}),1,0.,None,m,required_members=(m,)) for m in members]
    provider=SampledProvider({'start':(0.,0.,0.)},{m:1. for m in members},
        scene_geometry=scene,mother_position=(0.,5.,0.),case=case,
        return_sites={m:dict(position=state['position'],radius_m=.2)
                      for m,state in states.items()} if require_return else {})
    return build_executor_plan(units,tasks,provider,initial_target_ref='start',
        member_states=states,execution_candidates=provider.execution_candidates,budget_s=3.)


def test_separately_clear_methods_collide_only_in_the_complete_plan():
    for member in ('a','b'):
        result=planned('crossing',(member,))
        assert result.makespan==2.
        assert result.validation_scope=='NOMINAL_COMPLETE_PLAN_MOTION_AND_CAPACITY'
        assert not any(key in str(asdict(result)) for key in ('motion_traces','terminal_backend'))
    with pytest.raises(ValueError,match='PLAN_MEMBER_PATH_CONFLICT'):
        planned('crossing',('a','b'))


def test_fleet_clearance_is_not_the_smaller_static_obstacle_clearance():
    # Each centre is .5 m apart, with .1 m hull radii. This clears the
    # scene's .1 m static threshold, but misses the existing .5 m fleet rule.
    for member in ('a','b'):
        assert planned('near',(member,)).validation_scope=='NOMINAL_COMPLETE_PLAN_MOTION_AND_CAPACITY'
    with pytest.raises(ValueError,match='PLAN_MEMBER_PATH_CONFLICT'):
        planned('near',('a','b'))


def test_accepted_opaque_hold_remains_in_whole_plan_space_and_time_checks():
    plan=planned('near',('a',),opaque_position=(2.,0.,0.))
    assert plan.validation_scope=='NOMINAL_PLAN_WITH_MONITORED_OPAQUE_HOLD'
    with pytest.raises(ValueError,match='PLAN_MEMBER_PATH_CONFLICT'):
        planned('near',('a',),opaque_position=(0.,0.,0.))
    with pytest.raises(ValueError,match='PLAN_OPAQUE_HOLD_COMMITMENT_NOT_COVERED'):
        planned('near',('a',),opaque_position=(2.,0.,0.),opaque_horizon=1.)


def test_separately_deliverable_products_compete_for_one_plan_channel_budget():
    # Two independent surface senders avoid an unrelated USV resource clash.
    # At 32 KiB/s each 32 KiB payload fits 1.5 s; both plus notices do not.
    for member in ('a','b'):assert planned('capacity',(member,)).makespan==1.5
    with pytest.raises(ValueError,match='RECEIPT_NOT_COMPLETED_WITHIN_CHECKED_COMMITMENTS'):
        planned('capacity',('a','b'))


def test_real_provider_path_cannot_accept_a_complete_plan_without_motion_evidence():
    with pytest.raises(ValueError,match='unknown.*PLAN_MOTION_EVIDENCE_MISSING'):
        planned('missing',('a',))


def test_declared_return_checks_actual_model_terminal_not_only_route_label():
    with pytest.raises(ValueError,match='PLAN_REQUIRED_RETURN_NOT_REACHED'):
        planned('crossing',('a',),require_return=True)
