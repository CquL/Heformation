from dataclasses import replace

import pytest

from qn_aav_simulator.formation_monitor import (
    DEFAULT_RELATIVE_SLOTS, GroupCompletionMonitor, OdometrySample,
    validate_configuration, validate_target,
)


def monitor(hold=0.5, **kwargs):
    return GroupCompletionMonitor((5.0, 2.0, 0.5), hold, 10.0, **kwargs)


def samples(subject, now):
    return {agent_id: OdometrySample(now, target, (0.0, 0.0, 0.0))
            for agent_id, target in subject.targets.items()}


def test_all_seven_members_must_hold_their_own_scaled_slots():
    subject = monitor()
    snapshot = subject.evaluate(10.0, samples(subject, 10.0))
    assert snapshot.phase == "HOLDING"
    assert snapshot.terminal_state is None
    assert subject.targets[1] == pytest.approx((8.4642, 0.0, 0.5))
    for now in (10.1, 10.2, 10.3, 10.4):
        assert subject.evaluate(now, samples(subject, now)).terminal_state is None
    final = subject.evaluate(10.5, samples(subject, 10.5))
    assert final.terminal_state == "SUCCEEDED"
    assert final.hold_elapsed == pytest.approx(0.5)
    assert final.min_inter_agent_distance == pytest.approx(4.0, abs=1e-4)


@pytest.mark.parametrize("failure", ["position", "velocity"])
def test_one_unsettled_member_resets_the_entire_hold_window(failure):
    subject = monitor(0.3)
    subject.evaluate(10.0, samples(subject, 10.0))
    subject.evaluate(10.1, samples(subject, 10.1))
    changed = samples(subject, 10.2)
    if failure == "position":
        p = changed[6].position
        changed[6] = replace(changed[6], position=(p[0] + 0.51, p[1], p[2]))
    else:
        changed[6] = replace(changed[6], velocity=(0.0, 0.0, 0.26))
    assert subject.evaluate(10.2, changed).phase == "MOVING"
    snapshot = subject.evaluate(10.3, samples(subject, 10.3))
    assert snapshot.hold_started == 10.3
    for now in (10.4, 10.5):
        assert subject.evaluate(now, samples(subject, now)).terminal_state is None
    assert subject.evaluate(10.61, samples(subject, 10.61)).terminal_state == "SUCCEEDED"


@pytest.mark.parametrize("problem", ["missing", "old", "future", "nan", "wrong_frame"])
def test_unusable_odometry_cannot_complete_a_hold(problem):
    subject = monitor(0.2)
    subject.evaluate(10.0, samples(subject, 10.0))
    changed = samples(subject, 10.2)
    if problem == "missing":
        del changed[4]
    elif problem == "old":
        changed[4] = replace(changed[4], stamp=9.9)
    elif problem == "future":
        changed[4] = replace(changed[4], stamp=10.3)
    elif problem == "nan":
        changed[4] = replace(changed[4], velocity=(float("nan"), 0.0, 0.0))
    else:
        changed[4] = replace(changed[4], frame_id="map")
    result = subject.evaluate(10.2, changed)
    assert result.terminal_state == "ABORTED"
    assert result.reason == GroupCompletionMonitor.ODOMETRY_TIMEOUT
    assert result.stale_agent_ids == (4,)
    assert result.hold_started is None
    assert subject.evaluate(10.3, samples(subject, 10.3)) is result


def test_monitoring_gap_restarts_hold_even_if_latest_samples_are_fresh():
    subject = monitor(0.5)
    subject.evaluate(10.0, samples(subject, 10.0))
    result = subject.evaluate(10.6, samples(subject, 10.6))
    assert result.terminal_state is None
    assert result.hold_started == 10.6
    assert result.hold_elapsed == 0.0


def test_execution_timeout_cannot_be_overridden_by_arrival():
    subject = monitor(1.0, execution_timeout=0.5)
    subject.evaluate(10.0, samples(subject, 10.0))
    subject.evaluate(10.2, samples(subject, 10.2))
    result = subject.evaluate(10.5, samples(subject, 10.5))
    assert result.terminal_state == "ABORTED"
    assert result.reason == GroupCompletionMonitor.EXECUTION_TIMEOUT


def test_cancellation_prevents_success_and_never_becomes_a_completion():
    subject = monitor(0.1)
    subject.evaluate(10.0, samples(subject, 10.0))
    result = subject.evaluate(10.1, samples(subject, 10.1), cancelled=True)
    assert result.terminal_state == "PREEMPTED"
    assert result.hold_started is None
    assert subject.evaluate(10.2, samples(subject, 10.2)).terminal_state == "PREEMPTED"


def test_new_action_does_not_inherit_a_previous_hold_or_timeout():
    first = monitor(0.1)
    first.evaluate(10.0, {})
    second = monitor(0.1)
    snapshot = second.evaluate(10.0, samples(second, 10.0))
    assert snapshot.terminal_state is None
    assert snapshot.hold_started == 10.0


def test_zero_duration_still_requires_seven_fresh_settled_members():
    subject = monitor(0.0)
    assert subject.evaluate(10.0, samples(subject, 10.0)).terminal_state == "SUCCEEDED"
    subject = monitor(0.0)
    assert subject.evaluate(10.0, {}).reason == GroupCompletionMonitor.ODOMETRY_TIMEOUT


def test_clock_regression_aborts_instead_of_extending_the_hold():
    subject = monitor()
    subject.evaluate(10.2, samples(subject, 10.2))
    result = subject.evaluate(10.1, samples(subject, 10.1))
    assert result.reason == GroupCompletionMonitor.EXECUTION_TIMEOUT


@pytest.mark.parametrize("frame,center,hold", [
    ("map", (0, 0, 0.5), 1), ("world", (0, 0, 1.0), 1),
    ("world", (float("nan"), 0, 0.5), 1),
    ("world", (0, 0, float("nan")), 1),
    ("world", (0, float("inf"), 0.5), 1),
    ("world", (0, 0, 0.5), -1), ("world", (0, 0, 0.5), float("inf")),
])
def test_invalid_air_targets_are_rejected(frame, center, hold):
    with pytest.raises(ValueError):
        validate_target(frame, center, hold)


def test_only_the_fixed_seven_member_air_configuration_is_valid():
    slots = {str(key): list(value) for key, value in DEFAULT_RELATIVE_SLOTS.items()}
    assert validate_configuration(list(range(7)), slots, 2, 0.5, 0.25, 0.25, 180)[1] == (1.7321, -1, 0)
    with pytest.raises(ValueError):
        validate_configuration([0, 1, 2, 3, 4, 5, 5], slots, 2, 0.5, 0.25, 0.25, 180)
    slots["6"][2] = 0.1
    with pytest.raises(ValueError):
        validate_configuration(list(range(7)), slots, 2, 0.5, 0.25, 0.25, 180)
