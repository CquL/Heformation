import pickle
import pytest
import sys
import time
from pathlib import Path
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'upstream/Fossen/src'))
from qn_aav_simulator.pvs_backend import PvsBackend
from qn_aav_simulator.experiment_verdict import StaticSceneGeometry
from mrta_python.executors import ExecutorTravelTimeProvider


class ExplicitOtter(PvsBackend):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.probe_count=0

    def step(self,*args,**kwargs):
        result=super().step(*args,**kwargs)
        self.probe_count+=1  # changes the full-state signature; no extrapolation
        return result


def config():
    return yaml.safe_load((Path(__file__).parents[1]/'config/five_scene.yaml').read_text())['scene']


def test_late_support_shifts_only_selected_participants_and_propagates_idle():
    """A valid delayed rendezvous must not be rejected as unavailable forever."""
    from mrta_python import Executor,Task,ExecutorPlan,build_executor_plan
    from mrta_python.models import NativeActionSpec,NativeSegmentSpec
    from qn_aav_simulator.task_line import load_request
    request=load_request(Path(__file__).parents[1]/'config/monitoring_request_water.yaml')
    work=Executor('uuv',('uuv',),frozenset({'WATER'}))
    support=Executor('usv',('usv',),frozenset({'SURFACE'}))
    unused=Executor('unused',('unused',),frozenset({'AIR'}),1000.)
    models={'uuv':PvsBackend('remus100',(-5.,8.,-2.)),
            'usv':PvsBackend('otter',(-5.,-8.,0.),initialization_mode='STATIC_TRIM')}
    task=Task('water',frozenset({'WATER'}),1,0.,None,'sample')
    routes={work:NativeActionSpec((NativeSegmentSpec('WATER_PATH',((-5.,8.,-2.),(0.,8.,-2.))),),
                'COAST_STOP',observation_ids=('water_sample',),terminal_wait_s=30.),
            support:NativeActionSpec((NativeSegmentSpec('SURFACE_PATH',((-5.,-8.,0.),(4.,8.,0.))),),
                'TRIM_PROPULSION')}
    states={k:dict(position=b.snapshot()['position'],mode=b.snapshot()['actual_mode'],available_from=8. if k=='usv' else 0.)
            for k,b in models.items()}
    states['unused']=dict(position=(-30.,0.,.8),mode='AIR',available_from=1000.)
    provider=ExecutorTravelTimeProvider({'start':(-5.,8.,-2.)},{'uuv':1.,'usv':1.},
        native_models=models,native_efforts={'uuv':500.,'usv':20.},scene_geometry=StaticSceneGeometry.from_mapping(config()),
        cooperative_routes={('uuv','water'):(routes,)},observation_request=request,mother_position=tuple(config()['mother_ship_position']))
    before=pickle.dumps(models)
    candidate=provider.execution_candidates(work,task,0.,states,time.monotonic()+10.)[0]
    assert candidate.status=='FEASIBLE'
    import hashlib
    for activity in candidate.activities:
        member=activity.coalition[0]
        expected=hashlib.sha256(candidate.terminal_states[member]['native_backend'].execution_state_bytes()).hexdigest()
        assert activity.native_prediction['terminal_state_digest']==expected
    plan=ExecutorPlan(list(candidate.activities),serial=False)
    assert {i.planned_start for i in plan.items}=={8.}
    assert 8.<plan.makespan<1000.
    by_member={i.executor_id:i for i in plan.items}
    assert by_member['uuv'].native_prediction['pre_execution_idle_s']==8.
    assert by_member['usv'].native_prediction['pre_execution_idle_s']==0.
    assert pickle.dumps(models)==before
    # This local release example contains no trace of the support's earlier
    # commitment or of the unused AIR member. It cannot certify a full plan.
    check=provider._check_complete_plan(plan,[(candidate,tuple(plan.items))],states,time.monotonic()+2.)
    assert check['status']=='UNKNOWN' and check['reason']=='PLAN_COMMITTED_PREFIX_MISSING'
    # With a complete initial boundary the same real native work/support
    # method reaches the mandatory whole-plan check and remains acceptable.
    complete_states={k:dict(states[k],available_from=0.) for k in models}
    complete=build_executor_plan([work,support],[task],provider,initial_target_ref='start',
        member_states=complete_states,execution_candidates=provider.execution_candidates,budget_s=10.)
    assert len(complete.items)==2 and complete.search_complete
    assert pickle.dumps(models)==before


def test_native_query_includes_coast_and_does_not_mutate_live_controller():
    backend=PvsBackend('remus100',(-5.,8.,-2.));before=pickle.dumps(backend)
    result=ExecutorTravelTimeProvider.query_native_fragment(backend,[[(-5.,8.,-2.),(0.,8.,-2.)]],
        500.,StaticSceneGeometry.from_mapping(config()),time.monotonic()+10.)
    assert result['status']=='FEASIBLE' and result['geometry_checked']
    assert result['coast_start_s'] is not None and result['terminal_s']>0
    assert abs(result['duration_s']-result['motion_s']-result['terminal_s'])<1e-9
    assert result['terminal_position'][0]>5.  # goal plane was x=0
    assert pickle.dumps(backend)==before


def test_original_harbor_remus_segment_efforts_allow_observed_return_and_safe_tail():
    """The exact scene route must qualify through the native model, not a hand loop."""
    import math
    from qn_aav_simulator.observation_coverage import LocalObservationWindow,ObstacleBox
    from qn_aav_simulator.task_line import load_request
    root=Path(__file__).parents[1]/'config'
    scene_config=yaml.safe_load((root/'five_scene_harbor.yaml').read_text())['scene']
    scene=StaticSceneGeometry.from_mapping(scene_config)
    candidate=scene_config['water_route_candidates']['seabed_samples'][0]
    paths=tuple(tuple(tuple(point) for point in segment['points']) for segment in candidate['segments'])
    backend=PvsBackend('remus100',(-5.,8.,-2.),heading_rad=-.22)
    efforts=(500.,candidate['segments'][1]['propulsion_effort'])
    result=backend.predict_native_fragment(paths,efforts,scene,time.monotonic()+10.,
        max_model_time=candidate['execution_timeout_s'])
    request=load_request(root/'monitoring_request_joint.yaml')
    obstacles=tuple(ObstacleBox(center,size) for _,kind,center,size in scene.objects if kind=='SOLID')
    window=LocalObservationWindow(request,('water_sample',),'uuv','native-segmented',obstacles)
    home=tuple(scene_config['return_sites']['uuv']['position'])
    radius=scene_config['return_sites']['uuv']['radius_m']
    observed=None;outside=False;reentered=False
    for stamp,position in result['trajectory']:
        if math.dist(position,home)>radius:outside=True
        for event in window.sample(stamp,position,'WATER',stamp):
            if event.get('point_id')=='water_sample':observed=event['generated_at']
        if outside and observed is not None and stamp>=observed and math.dist(position,home)<=radius:
            reentered=True
    assert result['status']=='FEASIBLE' and observed is not None and reentered
    assert result['duration_s']<candidate['execution_timeout_s'] and backend.steps==0


def test_original_harbor_usv_precommits_native_trim_then_moves_for_result_window():
    root=Path(__file__).parents[1]/'config'
    cfg=yaml.safe_load((root/'five_scene_harbor.yaml').read_text())['scene']
    scene=StaticSceneGeometry.from_mapping(cfg)
    site=next(site for site in cfg['communication_sites'] if site['id']=='return_support')
    start=(-10.,4.,0.);target=tuple(site['position']);wait=site['wait_before_s']
    backend=PvsBackend('otter',start,initialization_mode='STATIC_TRIM')
    result=backend.predict_native_fragment(((start,start),(start,target,start)),(20.,20.),scene,
        time.monotonic()+10.,max_model_time=180.+wait,segment_durations=(wait,0.))
    before=next(position for stamp,position in result['trajectory'] if stamp>=wait-.01)
    after=next(position for stamp,position in result['trajectory'] if stamp>=wait+20.)
    assert result['status']=='FEASIBLE' and result['duration_s']>wait
    assert abs(before[0]-start[0])<.1 and after[0]>start[0]+1.
    assert backend.steps==0


def test_otter_fixed_wait_forecast_matches_every_explicit_native_step():
    scene=StaticSceneGeometry.from_mapping(yaml.safe_load((Path(__file__).parents[1]/
        'config/five_scene_harbor.yaml').read_text())['scene'])
    start=(-10.,4.,0.);site=(-4.,8.,0.)
    paths=((start,start),(start,site,start))
    query=dict(max_model_time=380.,segment_durations=(200.,0.),include_state=True)
    fast=PvsBackend('otter',start,initialization_mode='STATIC_TRIM')
    explicit=ExplicitOtter('otter',start,initialization_mode='STATIC_TRIM')
    predicted=fast.predict_native_fragment(paths,(20.,20.),scene,time.monotonic()+10.,**query)
    stepped=explicit.predict_native_fragment(paths,(20.,20.),scene,time.monotonic()+10.,**query)
    assert predicted['status']==stepped['status']=='FEASIBLE'
    assert predicted['trajectory']==stepped['trajectory']
    assert predicted['terminal_position']==stepped['terminal_position']
    assert predicted['terminal_backend'].steps==stepped['terminal_backend'].steps
    assert pickle.dumps(predicted['terminal_backend'].vehicle)==pickle.dumps(stepped['terminal_backend'].vehicle)
    assert fast.steps==explicit.steps==0


def test_native_execution_state_signature_is_equal_for_independent_models():
    first=PvsBackend('otter',(-10.,4.,0.),initialization_mode='STATIC_TRIM')
    second=PvsBackend('otter',(-10.,4.,0.),initialization_mode='STATIC_TRIM')
    for _ in range(800):
        first.step(.01,None,0.)
        second.step(.01,None,0.)
    assert first.execution_state_bytes()==second.execution_state_bytes()
    second.vehicle.ref+=1.
    assert second.snapshot()['position']==first.snapshot()['position']
    assert first.execution_state_bytes()!=second.execution_state_bytes()


def test_clear_goal_segment_but_obstructed_coast_is_infeasible():
    cfg=config();cfg['objects'].append(dict(id='coast_rock',kind='SOLID',center=[8.,8.,-2.],size=[1.,1.,1.]))
    scene=StaticSceneGeometry.from_mapping(cfg);backend=PvsBackend('remus100',(-5.,8.,-2.))
    paths=[[(-5.,8.,-2.),(0.,8.,-2.)]]
    assert not scene.path_violation(paths[0],backend.collision_radius_m)
    result=ExecutorTravelTimeProvider.query_native_fragment(backend,paths,500.,scene,time.monotonic()+10.)
    assert result['status']=='INFEASIBLE' and 'coast_rock' in result['reason']
    assert result['duration_s']>result['coast_start_s']
    assert backend.steps==0


def test_expired_query_budget_is_unknown_not_infeasible():
    result=ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
        [[(-5.,8.,-2.),(0.,8.,-2.)]],500.,StaticSceneGeometry.from_mapping(config()),time.monotonic()-1.)
    assert result['status']=='UNKNOWN' and result['reason']=='QUERY_BUDGET_EXHAUSTED'


def test_stationary_support_uses_native_trim_steps_instead_of_invented_motion():
    backend=PvsBackend('otter',(-5.,-8.,0.),initialization_mode='STATIC_TRIM')
    result=ExecutorTravelTimeProvider.query_native_fragment(backend,
        [[(-5.,-8.,0.),(-5.,-8.,0.)]],20.,StaticSceneGeometry.from_mapping(config()),
        time.monotonic()+10.,include_state=True)
    assert result['status']=='FEASIBLE' and result['duration_s']>=4.
    assert result['terminal_backend'].steps>=400 and backend.steps==0


def test_post_result_wait_propagates_residual_coast_and_uses_remaining_budget():
    scene=StaticSceneGeometry.from_mapping(config());deadline=time.monotonic()+10.
    result=ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
        [[(-5.,8.,-2.),(0.,8.,-2.)]],500.,scene,deadline,include_state=True)
    state=result['terminal_backend'];before=pickle.dumps(state)
    waited=ExecutorTravelTimeProvider.query_native_idle(state,20.,scene,deadline)
    assert waited['status']=='FEASIBLE'
    assert waited['terminal_position'][0]>result['terminal_position'][0]+.1
    assert pickle.dumps(state)==before
    extended=ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
        [[(-5.,8.,-2.),(0.,8.,-2.)]],500.,scene,time.monotonic()+10.,terminal_wait_s=20.)
    assert extended['status']=='FEASIBLE'
    assert extended['duration_s']>=result['duration_s']+20.-.011
    assert extended['terminal_position'][0]>result['terminal_position'][0]+.1
    continued=ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
        [[(-5.,8.,-2.),(0.,8.,-2.)]],500.,scene,time.monotonic()+10.,terminal_wait_s=20.,resume=result)
    assert continued['trajectory']==extended['trajectory']
    assert continued['duration_s']==extended['duration_s']
    with pytest.raises(ValueError,match='exact motion snapshot'):
        ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
            [[(-5.,8.,-2.),(0.,8.,-2.)]],600.,scene,time.monotonic()+10.,terminal_wait_s=20.,resume=result)
    exhausted=ExecutorTravelTimeProvider.query_native_idle(state,20.,scene,time.monotonic()-1.)
    assert exhausted['status']=='UNKNOWN' and exhausted['duration_s']==0


def test_qn_initial_trim_fixed_point_matches_every_explicit_model_step():
    import copy
    from qn_aav_simulator.contracts import AgentState,ControlCmd,CommandMode,PlantStepInput,PlatformAdapterCmd
    from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
    scene=StaticSceneGeometry.from_mapping(config())
    backend=QnPythonClosedLoopBackend(dict(reference_mode='ROUTE_POSITION',initialization_mode='STATIC_TRIM',
        water_guidance_mode='LOS_VELOCITY_REFERENCE',water_horizontal_controller_mode='LOS_SURGE_YAW'))
    backend.reset(AgentState('drone_2','AAV',0.,(-30.,8.,.8),(0.,0.,0.)))
    before=pickle.dumps(backend)
    projected=backend.predict_idle(3.,scene,time.monotonic()+10.)
    stepped=copy.deepcopy(backend);state=stepped.snapshot()
    for tick in range(300):
        command=ControlCmd('read-only-idle','drone_2',tick*.01,CommandMode.DESIRED_POSITION,
            (0.,0.,0.),desired_position=state.position,desired_yaw_rad=0.)
        stepped.step(PlantStepInput(state,command,PlatformAdapterCmd('drone_2'),.01,2.,8.))
        state=stepped.snapshot((tick+1)*.01)
    stepped._idle_reference=('INITIAL_HOLD',backend._idle_reference[1],
                             backend._idle_reference[2],stepped._state)
    assert projected['status']=='FEASIBLE' and projected['reason']=='QN_EXACT_INITIAL_HOLD_FIXED_POINT'
    assert projected['duration_s']==pytest.approx(3.,abs=1e-12) and len(projected['trajectory'])==301
    assert all(position==state.position for _,position in projected['trajectory'])
    assert pickle.dumps(vars(projected['terminal_backend']))==pickle.dumps(vars(stepped))
    assert pickle.dumps(backend)==before


def test_qn_native_state_signature_distinguishes_same_position_controller_memory():
    from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
    from qn_aav_simulator.contracts import AgentState
    config=dict(reference_mode='ROUTE_POSITION',initialization_mode='STATIC_TRIM',
        water_guidance_mode='LOS_VELOCITY_REFERENCE',water_horizontal_controller_mode='LOS_SURGE_YAW')
    first=QnPythonClosedLoopBackend(config)
    second=QnPythonClosedLoopBackend(config)
    for backend in (first,second):
        backend.reset(AgentState('drone_0','AAV',0.,(-30.,6.,.8),(0.,0.,0.)))
    assert first.execution_state_bytes()==second.execution_state_bytes()
    second._last_model_acceleration=(.1,0.,0.)
    assert first.snapshot().position==second.snapshot().position
    assert first.execution_state_bytes()!=second.execution_state_bytes()


def test_qn_candidate_uses_full_state_without_reset_and_budget_exhaustion_is_unknown():
    from qn_aav_simulator.qn_python_backend import QnPythonClosedLoopBackend
    from qn_aav_simulator.contracts import AgentState
    from qn_aav_simulator.platform_execution import Segment
    from mrta_python import Executor,Task
    from mrta_python.models import NativeActionSpec,NativeSegmentSpec
    backend=QnPythonClosedLoopBackend(dict(reference_mode='ROUTE_POSITION',initialization_mode='STATIC_TRIM',
        water_guidance_mode='LOS_VELOCITY_REFERENCE',water_horizontal_controller_mode='LOS_SURGE_YAW'))
    backend.reset(AgentState('drone_2','AAV',0.,(-30.,6.,-.6),(0.,0.,0.)))
    before=pickle.dumps(backend)
    scene=StaticSceneGeometry.from_mapping(config())
    segments=(Segment('WATER_PATH',((-30.,6.,-.6),(-30.,6.,-.6)),.1),)
    expired=backend.predict_native_fragment(segments,scene,time.monotonic()-1.)
    assert expired['status']=='UNKNOWN' and expired['duration_s']==0.
    route=NativeActionSpec((NativeSegmentSpec('WATER_PATH',segments[0].points,.1),),'FIXED_REFERENCE')
    provider=ExecutorTravelTimeProvider({'sample':segments[0].points[-1]},{'native_3':1.},
        native_routes={('native_3','sample'):(route,)},native_models={'drone_2':backend},scene_geometry=scene)
    candidates=provider.execution_candidates(Executor('native_3',('drone_2',),frozenset({'WATER'})),
        Task('sample',frozenset({'WATER'}),1,0.,None,'sample'),0.,
        {'drone_2':dict(position=backend.snapshot().position,mode='WATER',available_from=0.)},time.monotonic()+10.)
    completed=candidates[0]
    assert completed.status=='FEASIBLE' and completed.duration_s>=4.1
    assert completed.terminal_states['drone_2']['mode']=='WATER'
    assert completed.terminal_states['drone_2']['native_backend'] is not backend
    assert 'trajectory' not in completed.steps[0].native_prediction
    assert pickle.dumps(backend)==before
    # Actual qn samples must reach the complete-plan checker, including the
    # initial boundary and actual medium; a nominal terminal alone is not enough.
    from mrta_python import build_executor_plan
    unit=Executor('native_3',('drone_2',),frozenset({'WATER'}))
    task=Task('sample',frozenset({'WATER'}),1,0.,None,'sample')
    plan=build_executor_plan([unit],[task],provider,initial_target_ref='sample',
        member_states={'drone_2':dict(position=backend.snapshot().position,mode='WATER',available_from=0.)},
        execution_candidates=provider.execution_candidates,budget_s=10.)
    assert plan.validation_scope=='NOMINAL_COMPLETE_PLAN_MOTION_AND_CAPACITY'
    assert completed.motion_traces['drone_2'][0]==(0.,backend.snapshot().position,'WATER')
    assert completed.motion_traces['drone_2'][-1][2]=='WATER'
    assert pickle.dumps(backend)==before
