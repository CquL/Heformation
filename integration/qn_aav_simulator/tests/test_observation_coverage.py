"""Observation coverage: what counts as observed, and separately as delivered."""

import pytest

from qn_aav_simulator.monitoring_request import ObservationRequirement
from qn_aav_simulator.observation_coverage import (
    ObstacleBox, ObservationSample, evaluate_coverage, record_delivery,
)

# blur_tolerance is deliberately tight: at 1 m/s and 20 ms the member moves
# 0.02 m, so a 0.005 m tolerance makes the blur term bite, as it would with a
# real exposure.
REQ = ObservationRequirement(footprint_radius_m=2.5, min_dwell_s=1.0,
                             cruise_altitude_m=0.8, exposure_s=0.02,
                             blur_tolerance_m=0.005, nominal_standoff_m=0.8,
                             max_distance_from_altitude_m=3.0)
POINT = {"p": (-28.0, 4.0, 0.0)}
WEIGHTS = {"p": 1.0}


def dwell_samples(member="m0", start=0.0, end=1.2, step=0.2, xy=(-28.0, 4.0),
                  speed=0.0, z=0.8):
    rows = []
    t = start
    while t <= end + 1e-9:
        rows.append(ObservationSample(member, t, (xy[0], xy[1], z), speed))
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


def test_the_best_score_across_members_is_taken_once():
    """Deduplication: one point contributes once, from the best member."""
    rows = dwell_samples(member="slow", speed=0.0)
    rows += dwell_samples(member="fast", speed=3.0)
    result = evaluate_coverage(rows, POINT, REQ)
    assert result.points["p"].member_id == "slow"
    assert result.observed_fraction(WEIGHTS) == pytest.approx(1.0)
    # observing from the nominal standoff scores full marks once motion is slow
    assert result.points["p"].score == pytest.approx(1.0)


def test_fast_motion_reduces_the_score():
    slow = evaluate_coverage(dwell_samples(speed=0.0), POINT, REQ).points["p"].score
    fast = evaluate_coverage(dwell_samples(speed=5.0), POINT, REQ).points["p"].score
    assert slow == pytest.approx(1.0)
    assert fast < slow


def test_a_distant_observation_scores_lower_than_a_close_one():
    near = evaluate_coverage(dwell_samples(z=0.8), POINT, REQ).points["p"].score
    far = evaluate_coverage(dwell_samples(z=3.0), POINT, REQ).points["p"].score
    assert far < near


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
