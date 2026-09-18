"""plan.md M3: offline five-platform resource cases.

The target fleet is 3 AAV + 1 USV + 1 UUV.  These cases exercise resource
qualification, position/cost and occupancy at the planning layer only: the
USV and UUV have no execution endpoint in this round, so nothing here claims a
physical multi-unit closed loop.  The AAVs are modelled as three fixed
single-platform units; the online seven-member formation is a separate
configuration and is not derived from this one.
"""

import pytest

from mrta_python import (
    Executor, ExecutorTravelTimeProvider, Task, build_executor_plan,
    validate_executor_plan,
)

CENTERS = {"harbor": (0.0, 0.0, 0.5), "buoy": (6.0, 0.0, 0.5),
           "reef": (30.0, 0.0, 0.5), "dock": (-4.0, 0.0, 0.5)}

SPEEDS = {"usv_1": 2.0, "uuv_1": 1.0, "aav_1": 3.0, "aav_2": 3.0, "aav_3": 3.0}


def fleet():
    """The five target platforms as five offline execution units."""
    return [
        Executor("usv_1", ("usv_1",), frozenset({"SURFACE"}), 0.0, 2.0),
        Executor("uuv_1", ("uuv_1",), frozenset({"UNDERWATER"}), 0.0, 1.0),
        Executor("aav_1", ("aav_1",), frozenset({"AIR"}), 0.0, 3.0,
                 initial_target_ref="harbor"),
        Executor("aav_2", ("aav_2",), frozenset({"AIR"}), 0.0, 3.0,
                 initial_target_ref="harbor"),
        Executor("aav_3", ("aav_3",), frozenset({"AIR"}), 4.0, 3.0,
                 initial_target_ref="dock"),
    ]


def travel():
    return ExecutorTravelTimeProvider(CENTERS, SPEEDS)


def task(name, capability, service=4.0, deadline=1000.0, target="buoy", agents=1):
    return Task(name, frozenset({capability}), agents, service, deadline, target)


def test_case_one_only_the_usv_is_qualified_for_a_surface_task():
    units = fleet()
    survey = task("surface_survey", "SURFACE")
    plan = build_executor_plan(units, [survey], travel(), initial_target_ref="harbor")
    assert plan.assignments == {"surface_survey": "usv_1"}
    assert plan.items[0].coalition == ("usv_1",)
    validate_executor_plan(plan, units, [survey])


def test_case_two_three_aavs_are_qualified_but_cost_differently():
    """Same capability, different availability and start position."""
    units = fleet()
    survey = task("air_survey", "AIR", target="reef")
    plan = build_executor_plan(units, [survey], travel(), initial_target_ref="harbor")
    # aav_1/aav_2 start at the harbor, aav_3 is away at the dock and later.
    assert plan.assignments == {"air_survey": "aav_1"}
    assert plan.items[0].planned_start == pytest.approx(0.0)
    assert plan.items[0].travel_time == pytest.approx(10.0)
    validate_executor_plan(plan, units, [survey])


def test_case_three_two_air_tasks_either_queue_or_split():
    """Two AIR tasks: the plan must be feasible whichever way it resolves."""
    units = fleet()
    tasks = [task("air_a", "AIR", target="buoy"), task("air_b", "AIR", target="reef")]
    plan = build_executor_plan(units, tasks, travel(), initial_target_ref="harbor")
    validate_executor_plan(plan, units, tasks)
    queues = plan.executor_queues
    assert set(plan.assignments.values()) <= {"aav_1", "aav_2", "aav_3"}
    # No unit overlaps its own tasks, and each task is allocated exactly once.
    for executor_id, queue in queues.items():
        assert len(set(queue)) == len(queue)
        finishes = [plan.item(execution_id).planned_finish for execution_id in queue]
        assert finishes == sorted(finishes)
    starts = sorted(item.planned_start for item in plan.items)
    assert starts[0] == pytest.approx(0.0)


def test_the_usv_and_uuv_are_never_given_air_tasks():
    units = fleet()
    tasks = [task("air_a", "AIR"), task("air_b", "AIR"), task("dive", "UNDERWATER")]
    plan = build_executor_plan(units, tasks, travel(), initial_target_ref="harbor")
    assert plan.assignments["dive"] == "uuv_1"
    assert set(plan.assignments.values()) - {"uuv_1"} <= {"aav_1", "aav_2", "aav_3"}
    validate_executor_plan(plan, units, tasks)


def test_an_infeasible_capability_is_rejected_before_allocating():
    units = fleet()
    with pytest.raises(ValueError, match="no eligible executor"):
        build_executor_plan(units, [task("mine_hunt", "UNDERWATER_AUV")], travel(),
                            initial_target_ref="harbor")
