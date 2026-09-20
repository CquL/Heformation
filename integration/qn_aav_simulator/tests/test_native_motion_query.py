import pickle
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


def test_post_result_wait_propagates_residual_coast_and_uses_remaining_budget():
    scene=StaticSceneGeometry.from_mapping(config());deadline=time.monotonic()+10.
    result=ExecutorTravelTimeProvider.query_native_fragment(PvsBackend('remus100',(-5.,8.,-2.)),
        [[(-5.,8.,-2.),(0.,8.,-2.)]],500.,scene,deadline,include_state=True)
    state=result['terminal_backend'];before=pickle.dumps(state)
    waited=ExecutorTravelTimeProvider.query_native_idle(state,20.,scene,deadline)
    assert waited['status']=='FEASIBLE'
    assert waited['terminal_position'][0]>result['terminal_position'][0]+.1
    assert pickle.dumps(state)==before
    exhausted=ExecutorTravelTimeProvider.query_native_idle(state,20.,scene,time.monotonic()-1.)
    assert exhausted['status']=='UNKNOWN' and exhausted['duration_s']==0
