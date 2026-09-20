import pytest
from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment, actual_mode, validate_fragment
from qn_aav_simulator.platform_execution import DomainHistory, PlannerAcknowledgement


def test_ownership_does_not_assign_medium_and_rejects_stale_successors():
    owner=ReferenceOwnership()
    assert owner.claim('enter','PLATFORM',0)[0]
    assert actual_mode(0.)=='AIR'
    assert not owner.accepts('AIR_SWARM',1)
    assert not owner.claim('other','AIR_SWARM',1)[0]
    assert owner.finish('enter',True)
    assert owner.claim('air-next','AIR_SWARM',1)[0]
    assert not owner.claim('enter','PLATFORM',2)[0]
    assert not owner.finish('enter',False)
    assert not owner.locked


def test_unverified_terminal_locks_all_sources():
    owner=ReferenceOwnership()
    owner.claim('water','PLATFORM',0)
    owner.finish('water',False)
    assert owner.locked
    assert not owner.claim('new','AIR_SWARM',1)[0]


def test_fragment_requires_actual_entry_mode_and_connected_segments():
    enter=Segment('ENTER_WATER',((0,0,.5),(0,0,-.5)),10.)
    exit_=Segment('EXIT_WATER',((0,0,-.5),(0,0,.5)),10.)
    qualified={'ENTER_WATER','EXIT_WATER'}
    assert validate_fragment([enter,exit_],'AIR','FIXED_REFERENCE',qualified)=='AIR'
    assert enter.reference(5)==(0.,0.,0.)
    with pytest.raises(ValueError):
        validate_fragment([enter],'WATER','FIXED_REFERENCE',qualified)
    with pytest.raises(ValueError):
        validate_fragment([enter],'AIR','UNDEFINED',qualified)
    with pytest.raises(ValueError):
        validate_fragment([enter],'AIR','FIXED_REFERENCE',set())


def test_authorized_transition_preserves_history_without_erasing_real_faults():
    history=DomainHistory()
    air=frozenset({'AIR'})
    transition=frozenset({'AIR','TRANSITION','WATER'})
    history.observe(.8,0.,air)
    history.observe(-.6,1.,transition)
    history.observe(-.6,1.,frozenset({'WATER'}))
    history.observe(.8,0.,air)
    assert history.min_height==-.6 and history.max_medium==1.
    assert history.air_min_height==.8 and not history.violation
    history.observe(0.,.5,air)
    assert history.air_violation and history.violation
    history.observe(-.6,1.,transition)
    history.observe(.8,0.,air)
    assert history.air_violation  # A later valid mode cannot clear the fault.


def test_ack_requires_fresh_matching_epoch_and_cannot_clear_latched_fault():
    ack=PlannerAcknowledgement()
    v=dict(handover_enabled='true',reference_generation='2',ordinary_reference_paused='true',latched='false')
    ack.note(v,10.)
    assert ack.matches(2,True,10.1,1.)
    assert not ack.matches(3,True,10.1,1.)
    assert not ack.matches(2,False,10.1,1.)
    assert not ack.matches(2,True,12.,1.)
    ack.note(dict(v,latched='true'),12.)
    ack.note(v,12.1)
    assert not ack.matches(2,True,12.2,1.)


def test_air_height_still_checked_if_a_mode_field_is_inconsistent():
    history=DomainHistory()
    history.observe(-.5,0.,frozenset({'AIR'}),air_floor=.085)
    assert history.air_violation and history.violation


def test_repeated_fault_does_not_regenerate_stop_or_release_owner():
    owner=ReferenceOwnership()
    owner.claim('native','PLATFORM',0)
    owner.latch_fault()
    assert owner.generation==2 and owner.locked
    owner.latch_fault()
    assert owner.generation==2
    owner.finish('native',False)
    assert not owner.claim('air','AIR_SWARM',2)[0]
