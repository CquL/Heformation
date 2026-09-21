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


def config():
    return yaml.safe_load((Path(__file__).parents[1]/'config/five_scene.yaml').read_text())['scene']


def test_native_query_includes_coast_and_does_not_mutate_live_controller():
    backend=PvsBackend('remus100',(-5.,8.,-2.));before=pickle.dumps(backend)
    result=ExecutorTravelTimeProvider.query_native_fragment(backend,[[(-5.,8.,-2.),(0.,8.,-2.)]],
        500.,StaticSceneGeometry.from_mapping(config()),time.monotonic()+10.)
    assert result['status']=='FEASIBLE' and result['geometry_checked']
    assert result['coast_start_s'] is not None and result['terminal_s']>0
    assert abs(result['duration_s']-result['motion_s']-result['terminal_s'])<1e-9
    assert result['terminal_position'][0]>5.  # goal plane was x=0
    assert pickle.dumps(backend)==before


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
