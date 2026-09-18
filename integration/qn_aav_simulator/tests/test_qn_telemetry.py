"""P0.2: outer-step reference usage and model-time bookkeeping."""

import math

import pytest

from qn_aav_simulator.qn_telemetry import (
    CommandSnapshot, ModelClock, ReferenceUsageTracker,
)


def snapshot(step, position, trajectory_id=1, stamp=0.0, velocity=(0, 0, 0),
             acceleration=(0, 0, 0)):
    return CommandSnapshot(
        agent_id="drone_0", outer_step_index=step, received_ros_time_s=stamp,
        stamp_s=stamp, trajectory_id=trajectory_id, trajectory_flag=1,
        position=position, velocity=velocity, acceleration=acceleration, yaw_rad=0.0)


def test_used_reference_velocity_uses_the_outer_step_not_the_sub_step():
    tracker = ReferenceUsageTracker("drone_0")
    tracker.reset((0.0, 0.0, 0.5))
    first = tracker.record(snapshot(0, (0.1, 0.0, 0.5)), 0.1, 0.0)
    second = tracker.record(snapshot(1, (0.4, 0.0, 0.5)), 0.1, 0.1)
    # The reset position is the qn backend's previous reference, exactly as in
    # backend.step(), so even the first outer step has a well-defined derivative.
    assert first.velocity == pytest.approx((1.0, 0.0, 0.0))
    assert first.velocity_was_derived is True
    assert second.velocity == pytest.approx((3.0, 0.0, 0.0))
    assert second.used_outer_step == 1
    assert second.model_interval_start_s == pytest.approx(0.1)
    assert second.model_interval_end_s == pytest.approx(0.2)


def test_zero_motion_reference_is_not_reported_as_directly_consumed():
    tracker = ReferenceUsageTracker("drone_0")
    tracker.reset((1.0, 2.0, 0.5))
    usage = tracker.record(snapshot(0, (1.0, 2.0, 0.5)), 0.02, 0.0)
    assert usage.velocity == (0.0, 0.0, 0.0)
    assert usage.velocity_was_derived is True


def test_out_of_order_snapshot_is_rejected():
    tracker = ReferenceUsageTracker("drone_0")
    tracker.reset((0.0, 0.0, 0.5))
    tracker.record(snapshot(0, (0.0, 0.0, 0.5)), 0.1, 0.0)
    with pytest.raises(ValueError):
        tracker.record(snapshot(0, (0.1, 0.0, 0.5)), 0.1, 0.1)


def test_nonpositive_outer_step_is_rejected():
    tracker = ReferenceUsageTracker("drone_0")
    tracker.reset((0.0, 0.0, 0.5))
    with pytest.raises(ValueError):
        tracker.record(snapshot(0, (0.0, 0.0, 0.5)), 0.0, 0.0)


def test_model_clock_counts_outer_and_integration_steps_separately():
    clock = ModelClock()
    clock.advance(0.01, 0.001, 10)
    clock.advance(0.01, 0.001, 10)
    assert clock.model_time_s == pytest.approx(0.02)
    assert clock.outer_step_count == 2
    assert clock.integration_step_count == 20
    with pytest.raises(ValueError):
        clock.advance(0.011, 0.001, 10)
    with pytest.raises(ValueError):
        clock.advance(0.01, 0.001, 0)


def test_model_time_never_comes_from_ros_time():
    clock = ModelClock()
    assert clock.model_time_s == 0.0
    clock.advance(0.5, 0.001, 500)
    assert clock.model_time_s == pytest.approx(0.5)
    assert math.isclose(clock.model_time_s, 0.5)
