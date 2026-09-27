"""Request expansion: the number of observation tasks comes from coverage.

The plan is explicit that two interest points must not automatically produce two
waypoints, and that a request asking for something no online platform can do must
be refused rather than quietly trimmed.
"""

from pathlib import Path
import pytest

from qn_aav_simulator.monitoring_request import (
    InterestPoint, MonitoringRequest, ObservationRequirement, SurveyRegion,
    UnsupportedRequirement, circle_joint_mission_mappings, circle_sweep_points,
    expand, observation_candidates, unsupported_reasons,
)

REQUIREMENT = ObservationRequirement(footprint_radius_m=2.5, min_dwell_s=1.0,
                                     cruise_altitude_m=0.8)


def region(region_id="A", kind="SURFACE", points=None):
    points = points or []
    return SurveyRegion(region_id, kind, (-30.0, 0.0, 0.0), (-20.0, 10.0, 0.0),
                        tuple(points))


def request(regions, **kwargs):
    payload = dict(request_id="req-1", regions=tuple(regions), requirement=REQUIREMENT,
                   required_capabilities=frozenset({"AIR"}), service_time_s=4.0,
                   deadline_s=600.0)
    payload.update(kwargs)
    return MonitoringRequest(**payload)


def point(point_id, x, y, z=0.0, weight=1.0):
    return InterestPoint(point_id, (x, y, z), weight)


def test_circle_sweep_is_boustrophedon_and_stays_inside_selection():
    points=circle_sweep_points((3.,4.),4.,0.,2.,'p')
    assert 5<len(points)<64
    assert all((p.position[0]-3.)**2+(p.position[1]-4.)**2<=16.+1e-8 for p in points)
    rows={round(p.position[1],8):[] for p in points}
    for p in points:rows[round(p.position[1],8)].append(p.position[0])
    directions=[values[-1]-values[0] for values in rows.values() if len(values)>1]
    assert directions and all(a*b<0 for a,b in zip(directions,directions[1:]))


def test_circle_selection_generates_one_request_scene_pair_without_fixed_routes():
    import yaml
    root=Path(__file__).parents[1]/'config'
    base_request=yaml.safe_load((root/'monitoring_request_offshore.yaml').read_text())
    base_scene=yaml.safe_load((root/'five_scene_offshore.yaml').read_text())
    request,scene=circle_joint_mission_mappings(base_request,base_scene,(3.,4.),3.)
    selected=scene['scene']['selected_monitoring_area']
    assert selected['center'][:2]==[3.,4.] and selected['radius_m']==3.
    assert 'water_route_candidates' not in scene['scene'] and 'air_route_via' not in scene['scene']
    assert 1<=len(scene['scene']['transition_sites'])<=8
    assert len(scene['scene']['communication_sites'])==1
    assert all(region['shape']=='CIRCLE' for region in request['regions'] if region['region_id']!='offshore_aav_water')
    air_points=next(region['interest_points'] for region in request['regions']
                    if region['region_id']=='offshore_air')
    assert all(min((site['position'][0]-point['position'][0])**2+
                       (site['position'][1]-point['position'][1])**2
                   for point in air_points)>=1.0**2
               for site in scene['scene']['transition_sites'])


def test_two_points_inside_one_footprint_become_one_task():
    """A footprint covers both, so flying twice would be redundant."""
    tasks = expand(request([region(points=[point("a", -28.0, 4.0),
                                           point("b", -27.0, 4.5)])]))
    assert len(tasks) == 1
    assert tasks[0].covers == ("a", "b")


def test_points_beyond_one_footprint_become_separate_tasks():
    tasks = expand(request([region(points=[point("a", -28.0, 4.0),
                                           point("b", -20.0, 4.0)])]))
    assert len(tasks) == 2
    assert sorted(coverage for task in tasks for coverage in task.covers) == ["a", "b"]


def test_the_greedy_choice_takes_the_candidate_covering_most_points():
    """Three points in a line 2 m apart: the middle one covers the other two."""
    candidates = observation_candidates(
        region(points=[point("a", -28.0, 4.0), point("b", -26.5, 4.0),
                       point("c", -25.0, 4.0)]), REQUIREMENT)
    assert sorted(candidates["b"]) == ["a", "b", "c"]


def test_expansion_is_order_independent():
    points = [point("a", -28.0, 4.0), point("b", -26.5, 4.0),
              point("c", -25.0, 4.0), point("d", -20.0, 4.0)]
    forward = expand(request([region(points=points)]))
    backward = expand(request([region(points=list(reversed(points)))]))
    assert [t.covers for t in forward] == [t.covers for t in backward]


def test_a_point_outside_the_declared_observation_band_is_refused():
    """Every point can be observed by flying to it, unless it is out of band.

    A point is always its own candidate anchor, so the expansion can only fail
    when that anchor is not a valid observation position either - here a surface
    point far below the cruise altitude, outside max_distance_from_altitude_m.
    Being unable to cover a point must be an error, not a silent omission.
    """
    far_below = InterestPoint("deep", (-28.0, 4.0, -6.0), 1.0)
    with pytest.raises(UnsupportedRequirement, match="no declared footprint"):
        expand(request([region(points=[far_below])]))


def test_an_underwater_region_without_an_endpoint_is_refused_not_skipped():
    req = request([region("U", "UNDERWATER", [point("u", -24.0, -10.0, -3.0)])])
    with pytest.raises(UnsupportedRequirement, match="UNDERWATER"):
        expand(req, online_capabilities=("AIR",))


def test_relay_delivery_without_a_surface_endpoint_is_refused():
    req = request([region(points=[point("a", -28.0, 4.0)])], requires_relay_delivery=True)
    with pytest.raises(UnsupportedRequirement, match="SURFACE"):
        expand(req, online_capabilities=("AIR",))


def test_an_air_only_request_is_accepted_with_air_endpoints():
    tasks = expand(request([region(points=[point("a", -28.0, 4.0)])]),
                   online_capabilities=("AIR", "SURFACE"))
    assert len(tasks) == 1 and tasks[0].required_capabilities == frozenset({"AIR"})


def test_observation_tasks_do_not_ask_for_a_larger_unit_by_default():
    tasks = expand(request([region(points=[point("a", -28.0, 4.0)])]))
    assert all(task.allow_larger_unit is False for task in tasks)


def test_reasons_are_reported_for_every_unexecutable_part():
    req = request([region("U", "UNDERWATER", [point("u", -24.0, -10.0, -3.0)])],
                  requires_relay_delivery=True)
    reasons = unsupported_reasons(req, ("AIR",))
    assert any("UNDERWATER" in reason for reason in reasons)
    assert any("SURFACE" in reason for reason in reasons)
