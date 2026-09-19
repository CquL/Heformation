import pytest
from qn_aav_simulator.platform_execution import ReferenceOwnership, Segment, actual_mode, validate_fragment


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
