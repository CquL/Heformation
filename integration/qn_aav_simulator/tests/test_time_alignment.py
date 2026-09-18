"""P0.4: cumulative model/ROS drift on a common ROS grid."""

import pytest

from qn_aav_simulator.time_alignment import (
    ModelTimeHistory, ModelTimeSample, TimeAlignmentMonitor,
)


def monitor(agents=("a", "b"), **kwargs):
    return TimeAlignmentMonitor(list(agents), **kwargs)


def feed(subject, agent, times, model=lambda t: t):
    for value in times:
        subject.note_sample(ModelTimeSample(agent, value, model(value), value))


def test_aligned_agents_pass_the_default_gate():
    subject = monitor(baseline_seconds=5.0, grid_step_s=0.5)
    for step in range(11):
        now = step * 0.5
        subject.note_sample(ModelTimeSample("a", now, now))
        subject.note_sample(ModelTimeSample("b", now, now))
    report = subject.report()
    assert report.within_thresholds
    assert report.duration_s == pytest.approx(5.0)
    assert report.max_cross_agent_drift_s == pytest.approx(0.0)
    assert report.alignment_failure_count == 0


def test_a_lagging_agent_fails_both_drift_and_rate():
    subject = monitor(baseline_seconds=5.0, grid_step_s=0.5)
    for step in range(11):
        now = step * 0.5
        subject.note_sample(ModelTimeSample("a", now, now * 0.5))
        subject.note_sample(ModelTimeSample("b", now, now))
    report = subject.report()
    assert not report.within_thresholds
    assert report.max_abs_model_ros_drift_s > 0.05
    assert report.max_cross_agent_drift_s > 0.05
    assert any("rate" in reason for reason in report.reasons)


def test_missing_samples_are_reported_not_dropped():
    subject = monitor(baseline_seconds=2.0, grid_step_s=0.5)
    for step in range(5):
        now = step * 0.5
        subject.note_sample(ModelTimeSample("a", now, now))
        if step != 3:
            subject.note_sample(ModelTimeSample("b", now, now))
    report = subject.report()
    assert report.alignment_failure_count == 1
    assert report.valid_sample_count == 9
    assert report.expected_sample_count == 10
    assert not report.within_thresholds


def test_baseline_shorter_than_thirty_seconds_is_not_qualified():
    subject = monitor(baseline_seconds=30.0, grid_step_s=1.0)
    for step in range(6):
        now = float(step)
        subject.note_sample(ModelTimeSample("a", now, now))
        subject.note_sample(ModelTimeSample("b", now, now))
    report = subject.report()
    assert report.baseline_complete is False
    assert not report.within_thresholds


def test_constant_model_clock_offset_is_not_counted_as_drift():
    """e_i(t) is cumulative divergence; a fixed offset cancels at t0."""
    subject = monitor(baseline_seconds=2.0, grid_step_s=0.5)
    for step in range(5):
        now = step * 0.5
        subject.note_sample(ModelTimeSample("a", now, now + 0.02))
        subject.note_sample(ModelTimeSample("b", now, now + 0.02))
    report = subject.report()
    assert report.max_abs_model_ros_drift_s == pytest.approx(0.0, abs=1e-9)
    assert report.within_thresholds


def test_time_drift_reference_displacement_uses_measured_speed():
    subject = monitor(baseline_seconds=2.0, grid_step_s=0.5)
    for step in range(5):
        now = step * 0.5
        subject.note_sample(ModelTimeSample("a", now, now * 1.01))
        subject.note_sample(ModelTimeSample("b", now, now * 1.01))
        subject.note_reference_speed(1.0)
    report = subject.report()
    assert report.reference_speed_mps == pytest.approx(1.0)
    assert report.induced_reference_displacement_m == pytest.approx(
        0.02 * 1.0, abs=1e-6)
    assert report.worst_model_ros_rate == pytest.approx(1.01, abs=1e-9)


def test_tail_past_the_common_window_is_not_counted_as_missing():
    """The grid runs to the latest sample; the uncovered tail is not loss.

    Diagnostics stop at slightly different times per agent.  Trimming the
    compared window to the last time covered by every agent keeps the final
    grid point from being reported as "missing agents" (which used to make the
    pre-task baseline permanently unqualified).
    """
    subject = monitor(baseline_seconds=2.0, grid_step_s=0.1)
    for index in range(45):          # 0.0 .. 2.2 s
        now = round(index * 0.05, 6)
        subject.note_sample(ModelTimeSample("a", now, now))
        if now <= 2.15 + 1e-9:       # agent b stops 50 ms earlier
            subject.note_sample(ModelTimeSample("b", now, now))
    report = subject.report()
    assert report.alignment_failure_count == 0
    assert report.valid_sample_count == report.expected_sample_count
    assert report.duration_s == pytest.approx(2.15, abs=0.11)


def test_history_returns_nearest_model_time_within_the_window():
    history = ModelTimeHistory(["a"], horizon_s=10.0)
    for step in range(20):
        history.note(ModelTimeSample("a", step * 0.1, step * 0.1))
    assert history.model_time_at("a", 1.0, 0.06) == pytest.approx(1.0)
    assert history.model_time_at("a", 1.3, 0.06) == pytest.approx(1.3)
    assert history.model_time_at("a", 50.0, 0.06) is None


def test_out_of_order_history_is_rejected():
    history = ModelTimeHistory(["a"])
    history.note(ModelTimeSample("a", 1.0, 1.0))
    with pytest.raises(ValueError):
        history.note(ModelTimeSample("a", 0.5, 0.5))
