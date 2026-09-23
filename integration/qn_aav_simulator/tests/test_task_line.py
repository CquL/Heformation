"""The task line: request -> plan tasks, staged retest, group-phase verdict."""

from pathlib import Path

import pytest

from qn_aav_simulator.monitoring_request import (
    InterestPoint, MonitoringRequest, ObservationRequirement, ObservationTask,
    SurveyRegion, expand,
)
from qn_aav_simulator.observation_coverage import (
    CoverageResult, PointObservation, evaluate_coverage, record_delivery,
)
from qn_aav_simulator.task_line import (
    FormationPhase, formation_similarity, formation_interval_metrics,
    load_formation_phase, load_request, request_centres, retest_tasks, to_plan_tasks,
)

REQUEST_PATH = (Path(__file__).resolve().parents[1] / "config"
                / "monitoring_request_coastal.yaml")


def test_regional_request_keeps_all_domains_and_points_before_method_selection():
    import time,yaml
    from dataclasses import replace
    from types import SimpleNamespace
    from mrta_python import Executor
    from mrta_python.executors import ExecutorTravelTimeProvider
    from qn_aav_simulator.monitoring_request import regional_requirements
    from qn_aav_simulator.task_line import request_native_methods,build_request_executor_plan
    cfg=Path(__file__).parents[1]/'config'
    request=load_request(cfg/'monitoring_request_joint.yaml')
    water=request.regions[1]
    request=replace(request,regions=(request.regions[0],replace(water,interest_points=water.interest_points+
        (InterestPoint('nearby',(-.5,8.,-2.)),))))
    tasks=regional_requirements(request)
    assert len(tasks)==2 and {t.required_capabilities for t in tasks}=={frozenset({'AIR'}),frozenset({'WATER'})}
    assert all(t.deadline is None and not t.required_members for t in tasks)
    units=[Executor('uuv',('uuv',),frozenset({'WATER'})),Executor('usv',('usv',),frozenset({'SURFACE'}))]
    states={'uuv':dict(position=(-5.,8.,-2.),mode='WATER',available_from=0.),'usv':dict(position=(-5.,-8.,0.),mode='SURFACE',available_from=0.)}
    models={'uuv':SimpleNamespace(model='remus100',terminal_behavior='COAST_STOP'),
            'usv':SimpleNamespace(model='otter',terminal_behavior='TRIM_PROPULSION')}
    scene=yaml.safe_load((cfg/'five_scene.yaml').read_text())['scene']
    homes={m:dict(position=value['position'],radius_m=.2) for m,value in states.items()}
    generated,methods=request_native_methods(request,scene,units,states,models,time.monotonic()+1.,
                                             return_sites=homes)
    assert len(generated)==2  # unsupported AIR was not silently removed
    choices=next(iter(methods.values()))
    routes={m[units[0]].segments[0].points for m in choices}
    assert len(routes)>1  # candidate anchors/tours, not one greedy fixed view
    assert all(set(m[units[0]].observation_ids)=={'water_sample','nearby'} for m in choices)
    provider=ExecutorTravelTimeProvider({'start':(-5.,8.,-2.)},{'uuv':1.,'usv':1.},native_models=models)
    with pytest.raises(ValueError,match='no eligible executor'):
        build_request_executor_plan(request,scene,units,provider,states,return_sites=homes)


def test_return_requirement_is_loaded_and_native_methods_have_checked_return_tails(tmp_path):
    import yaml
    import time
    from types import SimpleNamespace
    from mrta_python import Executor
    from qn_aav_simulator.task_line import request_native_methods
    raw=yaml.safe_load((Path(__file__).parents[1]/'config/monitoring_request_water.yaml').read_text())
    raw['return_required']=True
    path=tmp_path/'request.yaml';path.write_text(yaml.safe_dump(raw))
    request=load_request(path)
    assert request.return_required is True
    units=[Executor('uuv',('uuv',),frozenset({'WATER'})),Executor('usv',('usv',),frozenset({'SURFACE'}))]
    models={'uuv':SimpleNamespace(model='remus100',terminal_behavior='COAST_STOP'),
            'usv':SimpleNamespace(model='otter',terminal_behavior='TRIM_PROPULSION')}
    states={'uuv':dict(position=(-5.,8.,-2.),mode='WATER'),
            'usv':dict(position=(-5.,-8.,0.),mode='SURFACE')}
    scene=yaml.safe_load((Path(__file__).parents[1]/'config/five_scene_harbor.yaml').read_text())['scene']
    with pytest.raises(ValueError,match='return destinations must be declared'):
        request_native_methods(request,scene,units,states,models,time.monotonic()+1.)
    homes={m:dict(position=value['position'],radius_m=.2) for m,value in states.items()}
    _,methods=request_native_methods(request,scene,units,states,models,time.monotonic()+1.,
                                     return_sites=homes)
    assert methods
    for choice in next(iter(methods.values())):
        for unit,fragment in choice.items():
            member=unit.physical_agent_ids[0]
            assert fragment.segments[0].points[-1]==states[member]['position']
    original={'uuv':dict(position=(-6.,8.,-2.),radius_m=.2),
              'usv':dict(position=(-6.,-8.,0.),radius_m=.2)}
    _,repaired=request_native_methods(request,scene,units,states,models,time.monotonic()+1.,
                                      return_sites=original)
    for choice in next(iter(repaired.values())):
        for unit,fragment in choice.items():
            assert fragment.segments[0].points[-1]==original[unit.physical_agent_ids[0]]['position']
    from mrta_python.executors import ExecutorTravelTimeProvider
    from qn_aav_simulator.task_line import build_request_executor_plan
    changed={key:dict(value,available_from=1.) for key,value in states.items()}
    provider=ExecutorTravelTimeProvider({'start':states['uuv']['position']},{'uuv':1.,'usv':1.},
        native_models=models,native_efforts={'uuv':500.,'usv':20.})
    missing_scene=dict(scene);missing_scene.pop('return_sites')
    with pytest.raises(ValueError,match='return destinations must be declared'):
        build_request_executor_plan(request,missing_scene,units,provider,changed)


def test_missing_deadline_reaches_request_expansion_and_both_planners(tmp_path):
    import yaml
    from mrta_python import Agent,Executor,build_plan,build_executor_plan
    raw=yaml.safe_load(REQUEST_PATH.read_text())
    raw.pop('deadline_s')
    path=tmp_path/'request.yaml';path.write_text(yaml.safe_dump(raw))
    request=load_request(path)
    assert request.deadline_s is None
    tasks=to_plan_tasks(expand(request))
    assert tasks and all(t.deadline is None for t in tasks)
    # No artificial large deadline, including hard-deadline scheduling mode.
    unit=Executor('a',('d',),frozenset({'AIR'}))
    plan=build_executor_plan([unit],tasks,lambda *args:1.,initial_target_ref='start',hard_deadlines=True)
    assert len(plan.items)==len(tasks)
    from dataclasses import replace
    fixed=build_plan([Agent('d'+str(i),frozenset({'AIR'})) for i in range(7)],
                     [replace(t,required_agent_count=7) for t in tasks],
                     lambda *args:1.,initial_target_ref='start')
    assert len(fixed.items)==len(tasks)

REQ = ObservationRequirement(footprint_radius_m=2.5, min_dwell_s=1.0,
                             cruise_altitude_m=0.8,
                             max_distance_from_altitude_m=3.0)
SLOTS = {"d0": (0.0, 0.0, 0.0), "d1": (0.0, -2.0, 0.0), "d2": (0.0, 2.0, 0.0)}
CENTRE = (-24.0, 6.0, 0.8)


def shape_positions(offset=(0.0, 0.0, 0.0), deform=None):
    positions = {}
    for member, slot in SLOTS.items():
        shifted = tuple(slot[i] + (deform.get(member, (0.0, 0.0, 0.0))[i]
                                   if deform else 0.0) for i in range(3))
        positions[member] = tuple(CENTRE[i] + shifted[i] + offset[i] for i in range(3))
    return positions


def coverage_with(observed_ids, delivered=False):
    result = CoverageResult()
    for point_id in ("a1", "a2"):
        result.points[point_id] = PointObservation(
            point_id, point_id in observed_ids,
            "d0" if point_id in observed_ids else None, 1.5, "test")
    if delivered:
        record_delivery(result, observed_ids)
    return result


def test_the_frozen_request_loads_and_expands_one_task_per_region():
    request = load_request(REQUEST_PATH)
    tasks = expand(request, online_capabilities=("AIR", "SURFACE"))
    assert [task.region_id for task in tasks] == ["zone_A", "zone_B", "zone_C"]
    # two points per region, inside one footprint: one waypoint each
    assert [len(task.covers) for task in tasks] == [2, 2, 2]


def test_the_frozen_formation_phase_loads():
    phase = load_formation_phase(REQUEST_PATH)
    assert phase.phase_id == "formation_transfer"
    assert phase.required_agent_count == 3
    assert phase.interest_point_ids == ()


def test_a_missing_request_field_is_an_error_not_a_default(tmp_path):
    path = tmp_path / "request.yaml"
    path.write_text("request_id: x\nregions: []\n")
    with pytest.raises(ValueError, match="missing"):
        load_request(path)


def test_observation_tasks_become_planner_tasks_with_named_references():
    request = load_request(REQUEST_PATH)
    tasks = expand(request, online_capabilities=("AIR", "SURFACE"))
    planner_tasks = to_plan_tasks(tasks)
    centres = request_centres(tasks)
    assert [task.task_id for task in planner_tasks] == [task.task_id for task in tasks]
    assert set(centres) == {task.task_id for task in tasks}
    assert centres[tasks[0].task_id] == tasks[0].target
    # observation tasks do not claim to need a larger unit
    assert all(task.allow_larger_unit is False for task in planner_tasks)


def retest_request():
    region = SurveyRegion("zone_A", "SURFACE", (-30.0, 2.5, 0.0), (-26.0, 5.5, 0.0),
                          (InterestPoint("a1", (-29.0, 3.2, 0.0)),
                           InterestPoint("a2", (-20.0, 4.4, 0.0))))
    return MonitoringRequest("r", (region,), REQ, frozenset({"AIR"}), 4.0, 900.0)


def test_a_retest_is_not_created_before_the_result_is_received():
    """Staged release: no data yet means nothing to decide a retest from."""
    request = retest_request()
    tasks = expand(request)
    coverage = coverage_with(("a1",))
    retests = retest_tasks(request, tasks, coverage, {"a1": 1.0, "a2": 1.0},
                           delivery_recorded=False, already_retested=False)
    assert retests == ()


def test_a_retest_targets_only_the_uncovered_points_once_delivery_is_recorded():
    request = retest_request()
    tasks = expand(request)
    coverage = coverage_with(("a1",))
    retests = retest_tasks(request, tasks, coverage, {"a1": 1.0, "a2": 1.0},
                           delivery_recorded=True, already_retested=False)
    assert len(retests) == 1
    assert retests[0].covers == ("a2",)
    assert retests[0].task_id.startswith("retest-zone_A")


def test_a_second_retest_round_is_never_generated():
    request = retest_request()
    tasks = expand(request)
    coverage = coverage_with(("a1",))
    assert retest_tasks(request, tasks, coverage, {"a1": 1.0, "a2": 1.0},
                        delivery_recorded=True, already_retested=True) == ()


def test_full_coverage_produces_no_retest():
    request = retest_request()
    tasks = expand(request)
    coverage = coverage_with(("a1", "a2"))
    assert retest_tasks(request, tasks, coverage, {"a1": 1.0, "a2": 1.0},
                        delivery_recorded=True, already_retested=False) == ()


def test_received_missing_water_report_keeps_underwater_retest_required():
    request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    coverage=CoverageResult(points={
        'air_sample':PointObservation('air_sample',True,'drone_1',1.,'received'),
        'water_sample':PointObservation('water_sample',False,None,0.,'missing')},
        delivered_point_ids=frozenset({'air_sample'}))
    weights={'air_sample':1.,'water_sample':1.}
    assert retest_tasks(request,(),coverage,weights,
                        delivery_recorded=False,already_retested=False)==()
    extra=retest_tasks(request,(),coverage,weights,
                       delivery_recorded=True,already_retested=False)
    assert len(extra)==1 and extra[0].covers==('water_sample',)
    assert extra[0].required_capabilities==frozenset({'WATER'})
    assert retest_tasks(request,(),coverage,weights,
                        delivery_recorded=True,already_retested=True)==()


def test_source_metric_is_rotation_translation_scale_invariant():
    positions = {m: (4 - 3*v[1], 6 + 3*v[0], 2 + 3*v[2]) for m, v in SLOTS.items()}
    assert formation_similarity(positions, SLOTS) == pytest.approx(0)


def test_interval_records_transient_deformation_without_inventing_threshold():
    good = shape_positions()
    bad = shape_positions(deform={"d1": (1.0, 1.0, 0.0)})
    result = formation_interval_metrics([(0, good), (.1, bad), (.2, good)], SLOTS,
                                         sample_timeout_s=.25)
    assert result["max_similarity_error"] > 0
    assert result["business_shape_verdict"] == "NOT_DEFINED"
    assert "complete" not in result


@pytest.mark.parametrize("stamps", [[0], [0, 999], [1, 0], [0, 0]])
def test_missing_or_unordered_interval_is_not_perfect_formation(stamps):
    with pytest.raises(ValueError):
        formation_interval_metrics([(t, shape_positions()) for t in stamps], SLOTS,
                                     sample_timeout_s=.25)


def test_unknown_points_and_revoked_gates_are_rejected(tmp_path):
    import yaml
    raw = yaml.safe_load(REQUEST_PATH.read_text())
    path = tmp_path / "bad.yaml"
    raw["formation_phase"]["interest_point_ids"] = ["undefined"]
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="undefined"):
        load_formation_phase(path)
    raw["formation_phase"]["interest_point_ids"] = []
    raw["formation_phase"]["corridor_half_width_m"] = 1.5
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="unsupported"):
        load_formation_phase(path)
