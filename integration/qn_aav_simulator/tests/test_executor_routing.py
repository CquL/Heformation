"""Dispatch-side routing and the active-unit mutual-exclusion invariant.

A unit and a group unit may own the same physical AAVs; they may not be occupied
at the same time.  These tests pin both halves down without ROS.
"""

import pytest

from qn_aav_simulator.executor_routing import (
    ExecutionUnit, conflicting_active_unit, dispatchable_units, load_routing,
    unit_for_coalition,
)


def fleet_configuration():
    return [
        {"executor_id": "aav_1", "physical_agent_ids": ["drone_0"],
         "capabilities": ["AIR"], "action_endpoint": "/aav_1/formation_action"},
        {"executor_id": "aav_2", "physical_agent_ids": ["drone_1"],
         "capabilities": ["AIR"], "action_endpoint": "/aav_2/formation_action"},
        {"executor_id": "aav_3", "physical_agent_ids": ["drone_2"],
         "capabilities": ["AIR"], "action_endpoint": "/aav_3/formation_action"},
        {"executor_id": "aav_formation", "physical_agent_ids": ["drone_0", "drone_1", "drone_2"],
         "capabilities": ["AIR"], "action_endpoint": "/aav_formation/formation_action"},
        {"executor_id": "usv_1", "physical_agent_ids": ["usv_1"],
         "capabilities": ["SURFACE"], "action_endpoint": None},
    ]


def routing():
    return load_routing(fleet_configuration(), default_members=[],
                        default_initial_target_ref="harbor")


def test_the_default_configuration_is_the_single_seven_member_unit():
    units = load_routing(None, default_members=["drone_%d" % i for i in range(7)],
                         default_initial_target_ref="start")
    assert set(units) == {"aav_formation"}
    assert units["aav_formation"].members() == {"drone_%d" % i for i in range(7)}
    assert units["aav_formation"].dispatchable


def test_a_unit_without_an_endpoint_is_never_dispatchable():
    """Planning for a platform is allowed; dispatching to it is not."""
    units = routing()
    assert units["usv_1"].dispatchable is False
    assert "usv_1" not in [unit.executor_id for unit in dispatchable_units(units)]


def test_static_overlap_between_a_unit_and_the_group_is_allowed():
    units = routing()
    assert units["aav_1"].members() & units["aav_formation"].members() == {"drone_0"}
    assert len(dispatchable_units(units)) == 4


def test_a_coalition_maps_to_the_unit_that_owns_exactly_those_members():
    units = routing()
    assert unit_for_coalition(units, ["drone_0"]).executor_id == "aav_1"
    assert unit_for_coalition(units, ["drone_1"]).executor_id == "aav_2"
    assert unit_for_coalition(units, ["drone_0", "drone_1", "drone_2"]).executor_id == "aav_formation"


def test_an_unowned_coalition_has_no_unit_rather_than_a_wrong_one():
    units = routing()
    assert unit_for_coalition(units, ["drone_9"]) is None
    assert unit_for_coalition(units, ["drone_0", "drone_1"]) is None


def test_a_shared_member_blocks_concurrent_occupancy():
    units = routing()
    conflict = conflicting_active_unit(units, ["aav_formation"], "aav_1")
    assert conflict is not None and conflict.executor_id == "aav_formation"
    conflict = conflicting_active_unit(units, ["aav_1"], "aav_formation")
    assert conflict is not None and conflict.executor_id == "aav_1"


def test_disjoint_units_do_not_block_each_other():
    units = routing()
    assert conflicting_active_unit(units, ["aav_1"], "aav_2") is None
    assert conflicting_active_unit(units, ["aav_2", "aav_3"], "aav_1") is None


def test_reoccupying_the_same_unit_is_not_a_conflict_with_itself():
    units = routing()
    assert conflicting_active_unit(units, ["aav_1"], "aav_1") is None


def test_an_unknown_active_unit_is_an_error_not_a_silent_pass():
    units = routing()
    with pytest.raises(ValueError, match="unknown active executor_id"):
        conflicting_active_unit(units, ["does_not_exist"], "aav_1")
    with pytest.raises(ValueError, match="unknown executor_id"):
        conflicting_active_unit(units, [], "does_not_exist")


def test_membership_is_compared_as_a_set_not_a_sequence():
    units = routing()
    assert unit_for_coalition(units, ["drone_2", "drone_0", "drone_1"]).executor_id == "aav_formation"
