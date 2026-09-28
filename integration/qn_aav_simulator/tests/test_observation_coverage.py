"""Observation coverage: what counts as observed, and separately as delivered."""

import pytest

from qn_aav_simulator.monitoring_request import ObservationRequirement
from qn_aav_simulator.observation_coverage import (
    ObstacleBox, ObservationSample, evaluate_coverage, record_delivery,
)

REQ = ObservationRequirement(footprint_radius_m=2.5, min_dwell_s=1.0,
                             cruise_altitude_m=0.8, max_distance_from_altitude_m=3.0)
POINT = {"p": (-28.0, 4.0, 0.0)}
WEIGHTS = {"p": 1.0}


def test_measured_map_cannot_clear_or_observe_through_a_first_hit():
    from qn_aav_simulator.observation_coverage import LocalSurveyMap
    local = LocalSurveyMap(.25)
    origin, rock, behind = (.125, .125, .125), (1.125, .125, .125), (2.125, .125, .125)
    local.integrate(origin, (rock,), (True,), 1.)
    assert local.state((.625, .125, .125)) == 'FREE'
    assert local.state(rock) == 'OCCUPIED'
    assert local.state(behind) == 'UNKNOWN'
    # Even an inconsistent later no-hit ray cannot erase the static obstacle.
    local.integrate(origin, (behind,), (False,), 2.)
    assert local.covered_ids({'rock': rock, 'behind': behind}) == ('rock',)
    assert local.state(rock) == 'OCCUPIED' and not local.observed(behind)
    assert local.hit_points() == (rock,)


def test_observed_centreline_does_not_authorize_unknown_body_clearance():
    from qn_aav_simulator.observation_coverage import LocalSurveyMap
    local = LocalSurveyMap(.25)
    origin, goal = (.125, .125, .125), (1.125, .125, .125)
    local.integrate(origin, (goal,), (False,), 1.)
    assert local.segment_clear(origin, goal, 0.)
    assert not local.segment_clear(origin, goal, .2)
    assert local.next_target(origin, goal, .2) is None
    assert not local.segment_clear(origin, (2.125, .125, .125), 0.)


def test_forbidden_policy_blocks_motion_but_not_measurement_rays():
    from qn_aav_simulator.observation_coverage import LocalSurveyMap
    local = LocalSurveyMap(.25)
    origin, goal = (.125, .125, .125), (2.125, .125, .125)
    local.declare_free_box((-1., -1., -1.), (3., 1., 1.))
    local.declare_forbidden_box((.75, -.25, -.25), (1.25, .5, .5))
    assert not local.observed((1., .125, .125))  # policy alone is no measurement
    local.integrate(origin, [goal], [False], 1., mode='AIR')
    assert local.observed(goal, 'AIR') and local.observed((1., .125, .125), 'AIR')
    assert local.state((1., .125, .125)) == 'FREE' and not local.hit_points()
    assert not local.segment_clear(origin, goal, .1)
    assert local.segment_clear((.125, -.5, .125), (2.125, -.5, .125), .1)


def test_local_map_short_targets_route_around_a_new_measured_obstacle():
    from qn_aav_simulator.observation_coverage import LocalSurveyMap
    local = LocalSurveyMap(.25)
    origin, goal = (.125, .125, .125), (2.125, .125, .125)
    endpoints = [((x + .5) * .25, (y + .5) * .25, (z + .5) * .25)
                 for x in range(-2, 12) for y in range(-4, 5) for z in range(-2, 3)]
    local.integrate(origin, endpoints, [False] * len(endpoints), 1.)
    local.integrate(origin, [(1.125, .125, .125)], [True], 2.)
    current, route = origin, []
    for _ in range(12):
        target = local.next_target(current, goal, .05, .5)
        assert target is not None and local.segment_clear(current, target, .05)
        route.append(target)
        current = target
        if current == goal:
            break
    assert current == goal
    assert any(abs(point[1] - origin[1]) > .1 for point in route)


def test_mapping_products_require_measured_cell_and_do_not_fabricate_dwell():
    from pathlib import Path
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.observation_coverage import LocalObservationWindow, LocalSurveyMap
    request = load_request(Path(__file__).parents[1] / 'config/monitoring_request_joint.yaml')
    window = LocalObservationWindow(request, ['water_sample'], 'uuv', 'mapping-goal')
    point = window.points['water_sample'][0]
    local = LocalSurveyMap(.25)
    assert not window.sample_mapping(local, 'WATER', 1.)
    local.integrate((point[0] - 1., point[1], point[2]), [point], [True], 2., mode='WATER')
    assert not window.sample_mapping(local, 'WATER', 1.9)  # future measurement
    assert not window.sample_mapping(local, 'AIR', 2.)    # wrong operating mode
    events = window.sample_mapping(local, 'WATER', 2.)
    assert len(events) == 1 and events[0]['goal_id'] == 'mapping-goal'
    assert events[0]['result']['model'] == 'MAPPING_PROXY'
    assert events[0]['result']['observed_state'] == 'OCCUPIED'
    assert events[0]['result']['dwell_s'] == 0.
    assert 'received_at' not in events[0]
    assert not window.sample_mapping(local, 'WATER', 3.)  # no duplicate product


def test_air_scan_cannot_become_a_water_product_after_mode_transition():
    from pathlib import Path
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.observation_coverage import LocalObservationWindow, LocalSurveyMap
    request = load_request(Path(__file__).parents[1] / 'config/monitoring_request_joint.yaml')
    window = LocalObservationWindow(request, ['water_sample'], 'drone_0', 'water-mapping-goal')
    point = window.points['water_sample'][0]
    local = LocalSurveyMap(.25)
    local.integrate((point[0] - 1., point[1], 1.), [point], [True], 1., mode='AIR')
    assert local.observed(point) and local.observed(point, 'AIR')
    assert not local.observed(point, 'WATER')
    assert not window.sample_mapping(local, 'WATER', 2.)
    # A conflicting WATER miss through the old occupied endpoint cannot
    # manufacture a new WATER hit. A genuine repeat first hit below can.
    local.integrate((point[0] - 1., point[1], point[2]),
                    [(point[0] + 1., point[1], point[2])], [False], 3., mode='WATER')
    assert not window.sample_mapping(local, 'WATER', 3.)
    local.integrate((point[0] - 1., point[1], point[2]), [point], [True], 4., mode='WATER')
    events = window.sample_mapping(local, 'WATER', 4.)
    assert len(events) == 1 and events[0]['result']['source_mode'] == 'WATER'
    assert local.covered_ids({'cell': point}, mode='WATER') == ('cell',)


def test_mapping_report_cell_requires_every_clipped_fine_voxel_not_just_centre():
    from pathlib import Path
    from dataclasses import replace
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.monitoring_request import InterestPoint, SurveyRegion, mapping_cell_samples
    from qn_aav_simulator.observation_coverage import LocalObservationWindow, LocalSurveyMap
    base = load_request(Path(__file__).parents[1] / 'config/monitoring_request_joint.yaml')
    point = InterestPoint('cell', (.5, .5, -2.), 1.)
    region = SurveyRegion('water', 'UNDERWATER', (-2., -2., -2.), (2., 2., -2.),
        (point,), shape='CIRCLE', center=(0., 0., -2.), radius_m=2., coverage_resolution_m=1.)
    request = replace(base, regions=(region,), execution_mode='ONLINE_MAPPING')
    window = LocalObservationWindow(request, ['cell'], 'uuv', 'region-goal')
    local = LocalSurveyMap(.25)
    cells = mapping_cell_samples((0., 0.), 2., point.position)
    assert len(cells) == 16 and window.mapping_cells['cell'] == cells
    local.integrate((.625, .625, -2.), [(.626, .626, -2.)], [False], 1., mode='WATER')
    assert local.observed(point.position, 'WATER')
    assert not window.sample_mapping(local, 'WATER', 1.)
    assert len(window.unobserved_mapping_points(local, 'WATER')) == 15
    local.integrate((.5, .5, -3.), cells, [False] * len(cells), 2., mode='WATER')
    events = window.sample_mapping(local, 'WATER', 2.)
    assert len(events) == 1 and events[0]['result']['sampled_voxel_count'] == 16
    assert not window.unobserved_mapping_points(local, 'WATER')


def test_native_observation_requires_fresh_active_domain_samples_and_emits_once():
    from pathlib import Path
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.observation_coverage import LocalObservationWindow
    request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    window=LocalObservationWindow(request,['water_sample'],'uuv','accepted-goal')
    point=(0.,8.,-2.)
    assert not window.sample(0.,point,'WATER',100.,valid=False)
    assert not window.sample(.1,point,'WATER',100.1)
    assert not window.sample(.5,point,'WATER',100.5)  # missing samples reset dwell
    assert not window.sample(.6,point,'AIR',100.6)  # wrong physical medium resets
    events=[]
    for i in range(7,19):events.extend(window.sample(i/10.,point,'WATER',100.+i/10.))
    assert len(events)==1 and events[0]['required_bytes']==32768
    assert events[0]['producer']=='uuv' and events[0]['observed'] is True
    assert events[0]['result']['dwell_s']==pytest.approx(1.)
    assert 'received_at' not in events[0]
    report=window.terminal_report(102.)
    assert report['event_type']=='OBSERVATION_TERMINAL'
    assert report['point_ids']==report['observed_ids']==['water_sample']


def test_missing_observation_has_a_distinct_received_report_without_a_business_product():
    from pathlib import Path
    from qn_aav_simulator.task_line import load_request
    from qn_aav_simulator.observation_coverage import LocalObservationWindow
    request=load_request(Path(__file__).parents[1]/'config/monitoring_request_joint.yaml')
    window=LocalObservationWindow(request,['water_sample'],'uuv','missed-goal')
    assert not window.sample(0.,(10.,8.,-2.),'WATER',100.)
    report=window.terminal_report(101.)
    assert report['point_ids']==['water_sample'] and report['observed_ids']==[]
    assert report['product_id']=='missed-goal:terminal'
    assert 'required_bytes' not in report


def dwell_samples(member="m0", start=0.0, end=1.2, step=0.2, xy=(-28.0, 4.0),
                  speed=0.0, z=0.8):
    rows = []
    t = start
    while t <= end + 1e-9:
        rows.append(ObservationSample(member, t, (xy[0], xy[1], z)))
        t += step
    return rows


def test_a_point_inside_the_footprint_for_long_enough_is_observed():
    result = evaluate_coverage(dwell_samples(), POINT, REQ)
    assert result.points["p"].observed
    assert result.observed_fraction(WEIGHTS) == pytest.approx(1.0)


def test_a_point_outside_the_footprint_is_not_observed():
    result = evaluate_coverage(dwell_samples(xy=(-10.0, 4.0)), POINT, REQ)
    assert not result.points["p"].observed
    assert result.observed_fraction(WEIGHTS) == 0.0


def test_dwell_shorter_than_required_is_not_observed():
    """Presence is not observation: the window must be held."""
    result = evaluate_coverage(dwell_samples(start=0.0, end=0.4), POINT, REQ)
    assert not result.points["p"].observed


def test_dwell_cannot_be_stitched_together_across_members():
    """Three members at 0.6 s each must not add up to 1.8 s.

    CARIC takes the best single observation; the plan requires one member to
    satisfy the condition continuously.
    """
    rows = []
    for index, member in enumerate(("a", "b", "c")):
        rows += dwell_samples(member=member, start=index * 1.0, end=index * 1.0 + 0.6,
                              step=0.2)
    result = evaluate_coverage(rows, POINT, REQ)
    assert not result.points["p"].observed


def test_a_stale_gap_breaks_the_continuous_run():
    """A gap longer than the sample timeout restarts the dwell count."""
    rows = dwell_samples(start=0.0, end=0.6, step=0.2)
    rows += dwell_samples(start=3.0, end=3.6, step=0.2)
    result = evaluate_coverage(rows, POINT, REQ, sample_timeout_s=0.5)
    assert not result.points["p"].observed


def test_a_blocked_line_of_sight_is_not_an_observation():
    wall = ObstacleBox((-28.0, 4.0, 0.4), (1.0, 1.0, 0.8))
    result = evaluate_coverage(dwell_samples(), POINT, REQ, obstacles=[wall])
    assert not result.points["p"].observed


def test_an_obstacle_beside_the_line_of_sight_does_not_block():
    beside = ObstacleBox((-26.0, 4.0, 0.4), (1.0, 1.0, 1.0))
    result = evaluate_coverage(dwell_samples(), POINT, REQ, obstacles=[beside])
    assert result.points["p"].observed


def test_observed_point_counts_once_across_members():
    """Deduplication: one point contributes once, regardless of member count."""
    rows = dwell_samples(member="slow", speed=0.0)
    rows += dwell_samples(member="fast", speed=3.0)
    result = evaluate_coverage(rows, POINT, REQ)
    assert result.points["p"].member_id == "slow"
    assert result.observed_fraction(WEIGHTS) == pytest.approx(1.0)
    assert "image quality unverified" in result.points["p"].reason


def test_vertical_range_matches_the_expansion_geometry():
    assert evaluate_coverage(dwell_samples(z=3), POINT, REQ).points["p"].observed
    assert not evaluate_coverage(dwell_samples(z=3.01), POINT, REQ).points["p"].observed


def test_unordered_samples_cannot_manufacture_dwell():
    with pytest.raises(ValueError, match="strictly increase"):
        evaluate_coverage(list(reversed(dwell_samples())), POINT, REQ)


def test_observed_is_not_the_same_as_delivered():
    """C_delivered needs the result to have been received."""
    result = evaluate_coverage(dwell_samples(), POINT, REQ)
    assert result.observed_fraction(WEIGHTS) == pytest.approx(1.0)
    assert result.delivered_fraction(WEIGHTS) == 0.0
    assert result.undelivered(WEIGHTS) == ("p",)
    record_delivery(result, ["p"])
    assert result.delivered_fraction(WEIGHTS) == pytest.approx(1.0)
    assert result.undelivered(WEIGHTS) == ()


def test_a_receipt_for_an_unobserved_point_is_not_a_delivered_observation():
    """The invariant lives in the result, not only in the caller.

    Nothing was observed, yet the point is recorded as delivered.  C_delivered
    must not move: a receipt for something never observed is not a delivered
    observation, and relying on callers never to record one would be a promise
    rather than an invariant.
    """
    result = evaluate_coverage([], POINT, REQ)
    record_delivery(result, ["p"])
    assert result.observed_fraction(WEIGHTS) == 0.0
    assert result.delivered_fraction(WEIGHTS) == 0.0
    assert result.delivered_point_observations(WEIGHTS) == ()


def test_an_observed_point_still_counts_once_delivered():
    result = evaluate_coverage(dwell_samples(), POINT, REQ)
    record_delivery(result, ["p"])
    assert result.delivered_fraction(WEIGHTS) == pytest.approx(1.0)
    assert result.delivered_point_observations(WEIGHTS) == ("p",)


def test_uncovered_reports_the_points_that_failed():
    result = evaluate_coverage(dwell_samples(xy=(-10.0, 4.0)), POINT, REQ)
    assert result.uncovered(WEIGHTS) == ("p",)
