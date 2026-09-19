"""plan.md P1 semantics: three independent verdicts and sample accounting."""

import math

import pytest
from dataclasses import replace

from qn_aav_simulator.experiment_verdict import (
    DISCRETE_SAMPLED, MemberSample, SAFETY_FAIL, SAFETY_NOT_VERIFIED, SAFETY_PASS,
    TASK_FAIL, TASK_PASS, VALIDITY_INCOMPLETE, VALIDITY_INVALID, VALIDITY_VALID,
    box_sample_points, box_signed_distance, box_surface_clearance, build_ledger,
    compute_metrics, decide, evaluate_safety,
)

BOX_CENTER = (-23.0, 0.0, 0.5)
BOX_SIZE = (1.0, 1.0, 1.2)
RADIUS = 0.25


def box_safety(samples, clearance, required=0.2):
    return evaluate_safety(
        samples, obstacle_clearances=[5.0], platform_radius_m=RADIUS,
        box_clearance_m=clearance, box_center=BOX_CENTER, box_size=BOX_SIZE,
        required_box_clearance_m=required)


def test_box_signed_distance_is_positive_outside_and_negative_inside():
    assert box_signed_distance((-23.0, 0.0, 0.5), BOX_CENTER, BOX_SIZE) == pytest.approx(-0.5)
    assert box_signed_distance((-24.0, 0.0, 0.5), BOX_CENTER, BOX_SIZE) == pytest.approx(0.5)
    # On the +x face the distance is zero.
    assert box_signed_distance((-22.5, 0.0, 0.5), BOX_CENTER, BOX_SIZE) == pytest.approx(0.0)


def test_surface_clearance_subtracts_the_radius_exactly_once():
    assert box_surface_clearance(
        (-24.0, 0.0, 0.5), BOX_CENTER, BOX_SIZE, RADIUS) == pytest.approx(0.25)
    # The two hold positions leave 0.25 m against a 0.20 m requirement: 5 cm.
    for slot in ((-24.0, 0.0, 0.5), (-22.0, 0.0, 0.5)):
        assert box_surface_clearance(slot, BOX_CENTER, BOX_SIZE, RADIUS) - 0.2 == \
            pytest.approx(0.05)


def test_box_sampling_covers_the_declared_extent():
    points = box_sample_points(BOX_CENTER, BOX_SIZE, 0.2)
    for axis, (center, size) in enumerate(zip(BOX_CENTER, BOX_SIZE)):
        values = [point[axis] for point in points]
        assert min(values) == pytest.approx(center - 0.5 * size)
        assert max(values) == pytest.approx(center + 0.5 * size)


def test_box_clearance_below_requirement_fails_safety():
    passing = box_safety(settled_series(), 0.25)
    assert passing.outcome == SAFETY_PASS
    assert passing.obstacle_check == "CHECKED"
    failing = box_safety(settled_series(), -0.05)
    assert failing.outcome == SAFETY_FAIL
    assert any("box surface clearance" in reason for reason in failing.reasons)


def test_no_declared_box_is_reported_as_not_applicable():
    safety = evaluate_safety(settled_series(), obstacle_clearances=[5.0])
    assert safety.outcome == SAFETY_PASS
    assert safety.obstacle_check == "NOT_APPLICABLE"
    assert safety.box_clearance_m is None

SLOTS = {
    0: (0.0, 0.0, 0.0), 1: (1.7321, -1.0, 0.0), 2: (0.0, -2.0, 0.0),
    3: (-1.7321, -1.0, 0.0), 4: (-1.7321, 1.0, 0.0), 5: (0.0, 2.0, 0.0),
    6: (1.7321, 1.0, 0.0),
}


def settled_series(count=20, *, period=0.05, drift_index=None, drift=1.0,
                   used_reference=None):
    """Settled states whose used reference is, by default, their own slot."""
    samples = {}
    for agent_id, slot in SLOTS.items():
        series = []
        for index in range(count):
            offset = drift if drift_index == index else 0.0
            series.append(MemberSample(
                agent_id=agent_id,
                ros_time_s=index * period,
                position=(slot[0], slot[1] + offset, slot[2]),
                world_velocity=(0.0, 0.0, 0.0),
                used_reference_position=(
                    slot if used_reference is None else used_reference),
                used_reference_velocity=(0.0, 0.0, 0.0),
                target_position=slot,
                model_time_s=index * period))
        samples[agent_id] = series
    return samples


def ledger_for(samples, period=0.05):
    grid = sorted({sample.ros_time_s
                   for series in samples.values() for sample in series})
    return build_ledger(sorted(SLOTS), grid, samples, expected_period_s=period)


def verdict_for(samples, **overrides):
    scale = 1.0
    metrics = compute_metrics(samples, scale=scale, relative_slots=SLOTS)
    safety = overrides.pop("safety", evaluate_safety(
        samples, obstacle_clearances=[1.0]))
    arguments = dict(
        epsilon_p=0.5, epsilon_v=0.25, motion_completed=True,
        adoption_state="ADOPTED", model_hold_satisfied=True,
        time_alignment_ok=True, metrics=metrics,
        ledger=ledger_for(samples), safety=safety, hold_duration_s=1.0)
    arguments.update(overrides)
    return decide(**arguments)


def test_unproven_reference_adoption_is_incomplete_not_valid():
    """plan.md item 11: an unproven adoption must never read as VALID."""
    verdict = verdict_for(settled_series(), adoption_state="UNCONFIRMED")
    assert verdict.task_outcome == TASK_FAIL
    assert verdict.experiment_validity == VALIDITY_INCOMPLETE
    assert "REFERENCE_ADOPTION_UNCONFIRMED" in " ".join(verdict.reasons)


def test_collision_in_transit_keeps_task_pass_and_fails_safety():
    """plan.md P1: arriving after a collision is PASS/FAIL, not PASS/PASS."""
    samples = settled_series()
    samples[0][5] = MemberSample(
        agent_id=0, ros_time_s=samples[0][5].ros_time_s,
        position=SLOTS[1], world_velocity=(0.0, 0.0, 0.0),
        used_reference_position=(1.0, 0.0, 0.5),
        used_reference_velocity=(0.0, 0.0, 0.0),
        target_position=SLOTS[0], model_time_s=samples[0][5].model_time_s)
    safety = evaluate_safety(samples, obstacle_clearances=[1.0])
    verdict = verdict_for(samples, safety=safety)
    assert safety.outcome == SAFETY_FAIL
    assert verdict.task_outcome == TASK_PASS
    assert verdict.safety_outcome == SAFETY_FAIL
    assert safety.evidence_kind == DISCRETE_SAMPLED


def test_missing_clearance_samples_are_not_verified_not_passed():
    samples = settled_series()
    safety = evaluate_safety(samples, obstacle_clearances=None)
    assert safety.outcome == SAFETY_NOT_VERIFIED
    assert verdict_for(samples, safety=safety).safety_outcome == SAFETY_NOT_VERIFIED


def test_no_declared_obstacle_is_not_a_missing_verification():
    """Without an obstacle there is no check to verify, which is not the same
    as a check that could not be performed."""
    samples = settled_series()
    safety = evaluate_safety(samples, obstacle_clearances=None,
                             obstacle_check_expected=False)
    assert safety.outcome == SAFETY_PASS
    assert safety.obstacle_check == "NOT_APPLICABLE"


def test_missing_samples_are_counted_instead_of_silently_dropped():
    samples = settled_series()
    del samples[4]
    ledger = ledger_for(samples)
    assert ledger.expected_sample_count == 20 * 7
    assert ledger.valid_sample_count == 20 * 6
    assert ledger.valid_sample_ratio == pytest.approx(6.0 / 7.0)
    assert ledger.alignment_failure_count == 20
    verdict = verdict_for(samples, ledger=ledger)
    assert verdict.experiment_validity == VALIDITY_INVALID
    assert "valid sample ratio" in " ".join(verdict.reasons)


def test_expected_grid_is_independent_of_the_samples_that_were_received():
    """A hole in the middle of a run must appear in the ledger.

    The grid comes from the experiment window and the nominal period, so the
    samples that never arrived still occupy their expected slots.  Deriving the
    grid from the received samples would make both the hole and the ratio
    disappear.  One nominal period is the matching tolerance, so a hole of four
    samples removes three grid points (the two edges are still covered).
    """
    samples = settled_series(count=20)
    period = 0.05
    for series in samples.values():
        del series[8:12]
    grid = [index * period for index in range(20)]
    ledger = build_ledger(sorted(SLOTS), grid, samples, expected_period_s=period)
    assert ledger.expected_sample_count == 20 * 7
    assert ledger.valid_sample_count == 18 * 7
    assert ledger.alignment_failure_count == 2 * 7
    assert ledger.valid_sample_ratio == pytest.approx(0.9)
    assert ledger.max_middle_gap_s == pytest.approx(0.15)
    assert ledger.leading_missing_s == pytest.approx(0.0)
    assert ledger.trailing_missing_s == pytest.approx(0.0)


def test_a_late_but_present_monitor_tick_is_not_a_missing_sample():
    """Monitor jitter must not be reported as absent data.

    The loop runs late here (60 ms ticks against a 50 ms nominal period).  Every
    observation is real, so the ledger reports full coverage and exposes the
    slower achieved period separately instead of blaming alignment.
    """
    samples = settled_series(count=8)
    nominal = 0.05
    for series in samples.values():
        for index, sample in enumerate(series):
            series[index] = replace(sample, ros_time_s=0.06 * index,
                                    state_stamp_s=0.06 * index)
    grid = [index * nominal for index in range(8)]
    ledger = build_ledger(sorted(SLOTS), grid, samples, expected_period_s=nominal)
    assert ledger.valid_sample_ratio == pytest.approx(1.0)
    assert ledger.alignment_failure_count == 0
    assert ledger.observed_period_s == pytest.approx(0.06)


def test_ledger_separates_leading_and_trailing_missing_samples():
    samples = settled_series(count=20)
    period = 0.05
    for series in samples.values():
        del series[:3]
        del series[-2:]
    grid = [index * period for index in range(20)]
    ledger = build_ledger(sorted(SLOTS), grid, samples, expected_period_s=period)
    assert ledger.leading_missing_s == pytest.approx(2 * period)
    assert ledger.trailing_missing_s == pytest.approx(period)
    assert ledger.max_middle_gap_s == pytest.approx(period)


def test_ledger_matches_on_the_state_stamp_not_the_monitor_tick():
    """The sample's own message time decides alignment, not the tick time."""
    samples = settled_series(count=4)
    period = 0.05
    for series in samples.values():
        for index, sample in enumerate(series):
            # Every tick ran late; the observation timestamps are what count.
            series[index] = replace(sample, ros_time_s=100.0 + index,
                                    state_stamp_s=index * period)
    grid = [index * period for index in range(4)]
    ledger = build_ledger(sorted(SLOTS), grid, samples, expected_period_s=period)
    assert ledger.valid_sample_ratio == pytest.approx(1.0)
    assert ledger.alignment_failure_count == 0


def test_model_time_hold_shorter_than_requested_fails_the_task():
    verdict = verdict_for(settled_series(), model_hold_satisfied=False)
    assert verdict.task_outcome == TASK_FAIL
    assert verdict.experiment_validity == VALIDITY_INVALID
    assert "model-time hold" in " ".join(verdict.reasons)


def test_mid_run_excursion_is_not_hidden_by_a_final_sample_metric():
    """A single settled end state must not summarise a whole run.

    ``final_slot_error_m`` is a last-sample metric, so a transient excursion in
    the middle of the run does not appear in it.  The worst-case metrics
    (tracking error, hold velocity, windowed maximum in the action server) are
    what carry the excursion, which is why the three are reported separately.
    """
    samples = settled_series(count=20, drift_index=7, drift=0.75)
    metrics = compute_metrics(samples, scale=1.0, relative_slots=SLOTS)
    assert metrics.final_slot_error_m == pytest.approx(0.0, abs=1e-9)
    samples[3][7] = MemberSample(
        agent_id=3, ros_time_s=samples[3][7].ros_time_s,
        position=(SLOTS[3][0] + 0.75, SLOTS[3][1], SLOTS[3][2]),
        world_velocity=(0.0, 0.0, 0.0),
        used_reference_position=(SLOTS[3][0], SLOTS[3][1], SLOTS[3][2]),
        used_reference_velocity=(0.0, 0.0, 0.0),
        target_position=SLOTS[3], model_time_s=samples[3][7].model_time_s)
    metrics = compute_metrics(samples, scale=1.0, relative_slots=SLOTS)
    assert metrics.final_slot_error_m == pytest.approx(0.0, abs=1e-9)
    assert metrics.trajectory_tracking_error_m == pytest.approx(0.75)


def test_tracking_error_is_reported_against_the_used_reference():
    samples = settled_series(used_reference=(2.0, 0.0, 0.5))
    metrics = compute_metrics(samples, scale=1.0, relative_slots=SLOTS)
    expected = max(
        math.dist(slot, (2.0, 0.0, 0.5)) for slot in SLOTS.values())
    assert metrics.trajectory_tracking_error_m == pytest.approx(expected)
    assert metrics.final_slot_error_m == pytest.approx(0.0, abs=1e-9)


def test_aligned_settled_run_is_valid_and_passing():
    verdict = verdict_for(settled_series())
    assert verdict.task_outcome == TASK_PASS
    assert verdict.safety_outcome == SAFETY_PASS
    assert verdict.experiment_validity == VALIDITY_VALID
    assert not verdict.reasons
    assert math.isfinite(verdict.metrics.final_slot_error_m)


def test_leaving_the_air_domain_invalidates_the_experiment():
    """Reaching the slots in water dynamics is not a valid AIR result."""
    verdict = verdict_for(
        settled_series(), air_domain_ok=False,
        air_domain_detail="model domain: max_medium_flag=1.000")
    assert verdict.task_outcome == TASK_PASS
    assert verdict.safety_outcome == SAFETY_PASS
    assert verdict.experiment_validity == VALIDITY_INVALID
    assert not verdict.air_domain_ok
    assert "model domain" in " ".join(verdict.reasons)


def test_surface_clearance_subtracts_the_platform_envelope():
    """Two members 0.6 m apart with 0.25 m radii are 0.1 m apart in surface terms."""
    samples = settled_series(count=2)
    samples[0][0] = MemberSample(
        agent_id=0, ros_time_s=samples[0][0].ros_time_s, position=(0.0, 0.0, 0.5),
        world_velocity=(0.0, 0.0, 0.0), target_position=(0.0, 0.0, 0.5))
    samples[1][0] = MemberSample(
        agent_id=1, ros_time_s=samples[1][0].ros_time_s, position=(0.6, 0.0, 0.5),
        world_velocity=(0.0, 0.0, 0.0), target_position=(0.6, 0.0, 0.5))
    centre_only = evaluate_safety(samples, obstacle_clearances=[5.0],
                                  required_inter_agent_clearance_m=0.5)
    surface = evaluate_safety(samples, obstacle_clearances=[5.0],
                              required_inter_agent_clearance_m=0.5,
                              platform_radius_m=0.25)
    assert centre_only.outcome == SAFETY_PASS
    assert surface.outcome == SAFETY_FAIL
    assert surface.min_inter_agent_surface_clearance_m == pytest.approx(0.1)


def test_observed_surface_violation_fails_safety_even_when_slots_are_met():
    safety = evaluate_safety(settled_series(), obstacle_clearances=[5.0],
                             surface_clearance_violation=True,
                             surface_detail="member below the surface plane")
    assert safety.outcome == SAFETY_FAIL
    verdict = verdict_for(settled_series(), safety=safety)
    assert verdict.task_outcome == TASK_PASS
    assert verdict.safety_outcome == SAFETY_FAIL
