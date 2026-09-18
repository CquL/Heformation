"""P0.3: adoption is decided by trajectory ownership, never by timestamps."""

from qn_aav_simulator.trajectory_adoption import (
    ADOPTED, REFERENCE_ADOPTION_UNCONFIRMED, TrajectoryAdoptionTracker,
)


def test_first_dispatch_adopts_the_dispatched_trajectory_without_prior_command():
    """The first dispatch of a run has no pre-dispatch PositionCommand.

    The qn is already tracking its synthetic hold reference (id 0) and reports
    it as ``source_trajectory_id``; that observable id is the prior reference.
    Adoption succeeds only once the qn consumes a trajectory that was first
    observed after dispatch.
    """
    tracker = TrajectoryAdoptionTracker([0])
    tracker.note_qn_source(0, 0, 1000, 10.0, 9.99)
    tracker.note_group_goal("/formation_action_server")
    tracker.begin_dispatch("T1", "goal-1", 10.5)
    assert not tracker.verdict().adopted

    tracker.note_position_command(0, 1, 10.6, 10.6)
    tracker.note_qn_source(0, 1, 1010, 10.7, 10.6)
    verdict = tracker.verdict()
    assert verdict.state == ADOPTED
    evidence = verdict.per_agent[0]
    assert evidence.pre_dispatch_qn_source_trajectory_id == 0
    assert evidence.adopted_trajectory_id == 1
    assert evidence.adopted_used_outer_step == 1010
    assert evidence.stamp_only_evidence_seen is False


def test_refreshed_stamp_on_the_old_trajectory_is_not_adoption():
    """The traj server republishes endpoint-hold commands at 100 Hz."""
    tracker = TrajectoryAdoptionTracker([0])
    tracker.note_position_command(0, 3, 0.5, 0.5)
    tracker.note_qn_source(0, 3, 100, 1.0, 1.0)
    tracker.note_group_goal("/formation_action_server")
    tracker.begin_dispatch("T2", "goal-2", 1.5)
    # Same trajectory id, fresh stamp, qn keeps using it.
    tracker.note_position_command(0, 3, 1.6, 1.6)
    tracker.note_qn_source(0, 3, 105, 1.7, 1.6)
    verdict = tracker.verdict()
    assert verdict.state == REFERENCE_ADOPTION_UNCONFIRMED
    assert verdict.per_agent[0].stamp_only_evidence_seen is True


def test_qn_source_never_dispatched_for_this_execution_is_not_adoption():
    tracker = TrajectoryAdoptionTracker([0])
    tracker.note_qn_source(0, 0, 10, 1.0, 1.0)
    tracker.note_group_goal("/formation_action_server")
    tracker.begin_dispatch("T3", "goal-3", 1.5)
    tracker.note_position_command(0, 7, 1.6, 1.6)
    tracker.note_qn_source(0, 9, 20, 1.7, 1.6)
    verdict = tracker.verdict()
    assert verdict.state == REFERENCE_ADOPTION_UNCONFIRMED
    assert "newly owned source_trajectory_id" in \
        verdict.per_agent[0].failure_reason


def test_group_goal_must_be_published_exactly_once():
    tracker = TrajectoryAdoptionTracker([0])
    tracker.note_qn_source(0, 0, 10, 1.0, 1.0)
    tracker.begin_dispatch("T1", "goal-1", 1.5)
    tracker.note_position_command(0, 1, 1.6, 1.6)
    tracker.note_qn_source(0, 1, 20, 1.7, 1.6)
    verdict = tracker.verdict()
    assert verdict.state == REFERENCE_ADOPTION_UNCONFIRMED
    assert verdict.group_goal_publish_count == 0
