"""plan.md P1 semantics: three independent verdicts and sample accounting."""

import math

import pytest

from qn_aav_simulator.experiment_verdict import (
    DISCRETE_SAMPLED, MemberSample, SAFETY_FAIL, SAFETY_NOT_VERIFIED, SAFETY_PASS,
    TASK_FAIL, TASK_PASS, VALIDITY_INCOMPLETE, VALIDITY_INVALID, VALIDITY_VALID,
    build_ledger, compute_metrics, decide, evaluate_safety,
)

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
