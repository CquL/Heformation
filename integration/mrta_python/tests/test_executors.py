"""plan.md P3: offline executor resource selection and plan-level competition."""

import pytest

from mrta_python import (
    Executor, ExecutorTravelTimeProvider, Task, build_executor_plan,
    eligible_executors, validate_executor_inputs, validate_executor_plan,
)

CENTERS = {"base": (0.0, 0.0, 0.5), "near": (2.0, 0.0, 0.5),
           "far": (10.0, 0.0, 0.5)}


def task(name, capabilities=("AIR",), agents=2, service=4.0, deadline=1000.0,
         target="near"):
    return Task(name, frozenset(capabilities), agents, service, deadline, target)


def provider(speeds):
    return ExecutorTravelTimeProvider(CENTERS, speeds)


def air_executor(executor_id, members, capabilities=("AIR",), available=0.0, speed=1.0):
    return Executor(executor_id, members, frozenset(capabilities), available, speed)


def scenario_one_only_a_is_eligible():
    """A task only unit A can do must go to A even though B is idle."""
    a = air_executor("A", ("d0", "d1"), capabilities=("AIR",), speed=1.0)
    b = air_executor("B", ("d2", "d3"), capabilities=("WATER",), speed=5.0)
    plan = build_executor_plan([a, b], [task("survey")], provider({"A": 1.0, "B": 5.0}),
                               initial_target_ref="base")
    return plan, a, b


def test_only_the_eligible_executor_is_selected():
    plan, _a, _b = scenario_one_only_a_is_eligible()
    item = plan.items[0]
    assert item.executor_id == "A"
    assert item.coalition == ("d0", "d1")
    assert plan.executor_queues == {"A": (item.execution_id,)}
    assert plan.assignments == {"survey": "A"}


def test_ineligible_executor_cannot_be_forced_into_the_plan():
    plan, a, b = scenario_one_only_a_is_eligible()
    plan.items[0].executor_id = "B"
    plan.items[0].coalition = b.physical_agent_ids
    with pytest.raises(ValueError, match="not eligible"):
        validate_executor_plan(plan, [a, b], [task("survey")])
    assert eligible_executors([a, b], task("survey")) == (a,)


def test_lower_cost_wins_between_two_eligible_executors():
    """Both capable; the faster unit has the smaller introduced makespan."""
    a = air_executor("A", ("d0", "d1"), speed=1.0)
    b = air_executor("B", ("d2", "d3"), speed=2.0)
    plan = build_executor_plan([a, b], [task("survey", target="far")],
                               provider({"A": 1.0, "B": 2.0}), initial_target_ref="base")
    item = plan.items[0]
    assert item.executor_id == "B"
    assert item.travel_time == pytest.approx(5.0)
    assert item.planned_finish == pytest.approx(9.0)
    # The losing unit stays completely free.
    assert "A" not in plan.executor_queues


def test_selection_compares_the_same_task_between_units():
    """Same task, two capable units, opposite travel cost ordering."""
    a = air_executor("A", ("d0", "d1"), available=0.0, speed=1.0)
    b = air_executor("B", ("d2", "d3"), available=0.0, speed=2.0)
    travel = provider({"A": 1.0, "B": 2.0})
    assert travel("A", "base", "far") == 10.0
    assert travel("B", "base", "far") == 5.0
    plan = build_executor_plan([a, b], [task("survey", target="far")], travel,
                               initial_target_ref="base")
    assert plan.items[0].executor_id == "B"


def test_two_tasks_compete_for_one_executor_and_serialize():
    """Only A is eligible, so both tasks queue on A instead of overlapping.

    base->near is 2 m and near->far is 8 m at the unit's 1 m/s, so the second
    task cannot start before the first one has finished on the shared unit.
    """
    a = air_executor("A", ("d0", "d1"), speed=1.0)
    b = air_executor("B", ("d2", "d3"), capabilities=("WATER",), speed=10.0)
    tasks = [task("first", target="near"), task("second", target="far")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 10.0}),
                               initial_target_ref="base")
    by_task = {item.task_id: item for item in plan.items}
    first, second = by_task["first"], by_task["second"]
    assert {first.executor_id, second.executor_id} == {"A"}
    assert (first.planned_start, first.planned_finish) == (0.0, pytest.approx(6.0))
    assert second.planned_start == pytest.approx(first.planned_finish)
    assert second.planned_finish == pytest.approx(18.0)
    assert plan.makespan == pytest.approx(18.0)
    assert plan.executor_queues["A"] == (first.execution_id, second.execution_id)
    validate_executor_plan(plan, [a, b], tasks)


def test_competition_is_avoided_when_a_second_eligible_unit_exists():
    a = air_executor("A", ("d0", "d1"), speed=1.0)
    b = air_executor("B", ("d2", "d3"), speed=1.0)
    tasks = [task("first", target="near"), task("second", target="near")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    assert plan.assignments == {"first": "A", "second": "B"}
    assert plan.makespan == pytest.approx(6.0)
    assert all(item.planned_start == 0.0 for item in plan.items)
    validate_executor_plan(plan, [a, b], tasks)


def test_available_from_delays_only_the_owning_executor():
    a = air_executor("A", ("d0", "d1"), available=4.0, speed=1.0)
    b = air_executor("B", ("d2", "d3"), available=0.0, speed=1.0)
    tasks = [task("early", target="near"), task("late", target="far")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    by_task = {item.task_id: item for item in plan.items}
    assert by_task["early"].executor_id == "B"
    assert by_task["early"].planned_start == 0.0
    assert by_task["late"].executor_id == "A"
    assert by_task["late"].planned_start == pytest.approx(4.0)
    validate_executor_plan(plan, [a, b], tasks)


def test_service_time_and_required_size_are_respected():
    a = air_executor("A", ("d0",), speed=1.0)
    b = air_executor("B", ("d1", "d2"), speed=1.0)
    plan = build_executor_plan([a, b], [task("pair", agents=2)], provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    assert plan.items[0].executor_id == "B"
    assert plan.items[0].coalition == ("d1", "d2")


def test_infeasible_task_is_rejected_before_allocating():
    a = air_executor("A", ("d0", "d1"), speed=1.0)
    with pytest.raises(ValueError, match="no eligible executor"):
        build_executor_plan([a], [task("dive", capabilities=("UNDERWATER",))],
                            provider({"A": 1.0}), initial_target_ref="base")


def test_duplicate_membership_and_bad_units_are_rejected():
    a = air_executor("A", ("d0", "d1"))
    shared = air_executor("B", ("d1", "d2"))
    with pytest.raises(ValueError, match="belongs to both"):
        build_executor_plan([a, shared], [task("survey")], provider({"A": 1.0, "B": 1.0}),
                            initial_target_ref="base")
    with pytest.raises(ValueError, match="duplicate executor_id"):
        build_executor_plan([a, air_executor("A", ("d4", "d5"))], [task("survey")],
                            provider({"A": 1.0}), initial_target_ref="base")
    with pytest.raises(ValueError, match="nominal_speed_mps must be positive"):
        validate_executor_inputs([air_executor("C", ("d6",), speed=0.0)],
                                 [task("survey")])


def test_executor_selection_is_input_order_independent():
    a = air_executor("A", ("d0", "d1"), speed=1.0)
    b = air_executor("B", ("d2", "d3"), speed=1.0)
    tasks = [task("one", target="near"), task("two", target="far")]
    travel = provider({"A": 1.0, "B": 1.0})
    forward = build_executor_plan([a, b], tasks, travel, initial_target_ref="base")
    reverse = build_executor_plan([b, a], list(reversed(tasks)), travel,
                                  initial_target_ref="base")
    assert [item.execution_id for item in forward.items] == \
           [item.execution_id for item in reverse.items]
    assert forward.assignments == reverse.assignments
