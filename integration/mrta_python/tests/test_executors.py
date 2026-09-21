"""plan.md P3: offline executor resource selection and plan-level competition."""

import math

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


def test_one_business_task_has_overlapping_work_and_support_but_joint_edges_stay_acyclic():
    from dataclasses import replace
    from mrta_python import ExecutorPlan,ExecutorPlanItem
    from mrta_python.models import ExecutionStep
    from mrta_python.executors import activity_predecessors
    uuv=air_executor('uuv',('uuv',),capabilities=('WATER',))
    usv=air_executor('usv',('usv',),capabilities=('SURFACE',))
    business=task('sample',capabilities=('WATER',),agents=1,service=0.)
    work=ExecutorPlanItem('observe','sample','uuv',('uuv',),0.,10.,10.,0.,0.,
        execution_steps=(ExecutionStep('uuv',10.,'sample'),))
    support=ExecutorPlanItem('support','sample','usv',('usv',),4.,8.,4.,0.,0.,
        execution_steps=(ExecutionStep('usv',4.,'meeting'),),fulfills_task=False)
    plan=ExecutorPlan([work,support],serial=False)
    validate_executor_plan(plan,[uuv,usv],[business])
    assert plan.task_start_times=={'sample':0.} and plan.task_finish_times=={'sample':10.}
    assert plan.assignments=={'sample':('usv','uuv')}
    with pytest.raises(ValueError,match='cycle'):
        activity_predecessors(replace(plan,activity_edges=(('observe','support'),('support','observe'))))
    duplicate=replace(support,execution_id='conflict',executor_id='uuv',coalition=('uuv',),
        execution_steps=(ExecutionStep('uuv',4.,'meeting'),))
    with pytest.raises(ValueError,match='overlap'):
        validate_executor_plan(ExecutorPlan([work,duplicate],serial=False),[uuv,usv],[business])


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
    """One unit's availability does not delay another unit's task."""
    a = Executor("A", ("d0", "d1"), frozenset({"AIR", "DEEP"}), 4.0, 1.0)
    b = Executor("B", ("d2", "d3"), frozenset({"AIR", "SHALLOW"}), 0.0, 1.0)
    only_a = Task("only_a", frozenset({"AIR", "DEEP"}), 2, 4.0, 1000.0, "near")
    only_b = Task("only_b", frozenset({"AIR", "SHALLOW"}), 2, 4.0, 1000.0, "near")
    plan = build_executor_plan([a, b], [only_a, only_b],
                               provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    by_task = {item.task_id: item for item in plan.items}
    assert by_task["only_a"].executor_id == "A"
    assert by_task["only_a"].planned_start == pytest.approx(4.0)
    assert by_task["only_b"].executor_id == "B"
    assert by_task["only_b"].planned_start == pytest.approx(0.0)
    validate_executor_plan(plan, [a, b], [only_a, only_b])


def test_each_executor_keeps_its_own_current_position():
    """A unit keeps its own transition reference, not another unit's target."""
    a = Executor("A", ("d0", "d1"), frozenset({"AIR"}), available_from=4.0,
                 nominal_speed_mps=1.0, initial_target_ref="near")
    b = air_executor("B", ("d2", "d3"), available=0.0, speed=1.0)
    tasks = [task("early", target="near"), task("late", target="near")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    by_task = {item.task_id: item for item in plan.items}
    assert by_task["early"].executor_id == "B"
    # A starts next to "near" and only becomes available at t=4, so it takes the
    # remaining task without paying a transit it never made.
    assert by_task["late"].executor_id == "A"
    assert by_task["late"].travel_time == pytest.approx(0.0)
    assert by_task["late"].planned_start == pytest.approx(4.0)
    validate_executor_plan(plan, [a, b], tasks)


def test_a_stationary_unit_is_not_charged_for_another_unit_travel():
    """Counter-example for the shared-transition-position error.

    A flies a long urgent mission to P; B stays at the start.  B's own task next
    to the start must still cost its real transit, not A's distance from P.
    """
    centers = {"start": (0.0, 0.0, 0.5), "P": (100.0, 0.0, 0.5),
               "Q": (1.0, 0.0, 0.5)}
    travel = ExecutorTravelTimeProvider(centers, {"A": 1.0, "B": 1.0})
    a = Executor("A", ("d0",), frozenset({"AIR", "LONG"}), 0.0, 1.0)
    b = Executor("B", ("d1",), frozenset({"AIR", "SHORT"}), 0.0, 1.0)
    long_task = Task("TA", frozenset({"AIR", "LONG"}), 1, 4.0, 5.0, "P")
    short_task = Task("TB", frozenset({"AIR", "SHORT"}), 1, 4.0, 1000.0, "Q")
    plan = build_executor_plan([a, b], [long_task, short_task], travel,
                               initial_target_ref="start")
    by_task = {item.task_id: item for item in plan.items}
    assert by_task["TA"].executor_id == "A"
    assert by_task["TB"].executor_id == "B"
    # B never moved: 1 m at 1 m/s.  The shared-position error made this 99 s.
    assert by_task["TB"].travel_time == pytest.approx(1.0)
    assert by_task["TB"].planned_start == pytest.approx(0.0)


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
    with pytest.raises(ValueError, match="duplicate executor_id"):
        build_executor_plan([a, air_executor("A", ("d4", "d5"))], [task("survey")],
                            provider({"A": 1.0}), initial_target_ref="base")
    with pytest.raises(ValueError, match="nominal_speed_mps must be positive"):
        validate_executor_inputs([air_executor("C", ("d6",), speed=0.0)],
                                 [task("survey")])
    with pytest.raises(ValueError, match="duplicate physical agent"):
        validate_executor_inputs([air_executor("D", ("d7", "d7"))], [task("survey")])


def test_static_membership_may_overlap_across_units():
    """A single-platform unit and a group unit may own the same agents.

    They are two ways of using one fleet, not two fleets.  Registering them
    together is legal; what is illegal is occupying both at once, which is
    checked on the plan.
    """
    single = air_executor("aav_1", ("d0",), speed=1.0)
    formation = air_executor("aav_formation", ("d0", "d1", "d2"), speed=1.0)
    validate_executor_inputs([single, formation], [task("survey", agents=1)])
    shared = Executor("aav_2", ("d1",), frozenset({"AIR"}), 0.0, 1.0)
    validate_executor_inputs([single, shared, formation], [task("survey", agents=1)])


def test_overlapping_units_may_not_be_occupied_at_the_same_time():
    """Mutual exclusion on the plan, not on registration."""
    single = air_executor("aav_1", ("d0",), speed=1.0)
    formation = air_executor("aav_formation", ("d0", "d1", "d2"), speed=1.0)
    one = task("survey", agents=1)
    three = Task("group", frozenset({"AIR"}), 3, 4.0, 1000.0, "far",
                 allow_larger_unit=True)
    # Two tasks that must run back to back on units sharing d0: legal because
    # the plan serialises them.
    serial = build_executor_plan([single, formation], [one, three],
                                 provider({"aav_1": 1.0, "aav_formation": 1.0}),
                                 initial_target_ref="base")
    for item in serial.items:
        assert item.executor_id in {"aav_1", "aav_formation"}
    # The same two tasks forced to overlap in time must be rejected.
    overlapping = type(serial)()
    a = type(serial.items[0])("e1", "survey", "aav_1", ("d0",), 0.0, 6.0, 2.0, 0.0, 4.0)
    b = type(serial.items[0])("e2", "group", "aav_formation",
                              ("d0", "d1", "d2"), 3.0, 9.0, 2.0, 0.0, 4.0)
    overlapping.items = [a, b]
    with pytest.raises(ValueError, match="overlap in time"):
        validate_executor_plan(overlapping, [single, formation], [one, three])


def test_a_single_platform_task_does_not_go_to_the_group_unit_by_default():
    """Larger and equally capable is not a reason to hand over a one-member task."""
    single = air_executor("aav_1", ("d0",), speed=1.0)
    formation = air_executor("aav_formation", ("d0", "d1", "d2"), speed=5.0)
    one = task("survey", agents=1)
    assert [e.executor_id for e in eligible_executors([single, formation], one)] == ["aav_1"]
    # With the opt-in the group becomes a candidate.
    opted_in = Task("survey2", frozenset({"AIR"}), 1, 4.0, 1000.0, "near",
                    allow_larger_unit=True)
    assert {e.executor_id for e in eligible_executors([single, formation], opted_in)} == {
        "aav_1", "aav_formation"}
    plan = build_executor_plan([single, formation], [one],
                               provider({"aav_1": 1.0, "aav_formation": 5.0}),
                               initial_target_ref="base")
    assert plan.assignments["survey"] == "aav_1"


def test_group_travel_is_set_by_the_last_member_not_the_average_centre():
    """Scattered members must pay for the reassembly, not a mean position."""
    formation = air_executor("aav_formation", ("d0", "d1", "d2"), speed=1.0)
    slots = {"d0": (0.0, 0.0, 0.0), "d1": (0.0, -2.0, 0.0), "d2": (0.0, 2.0, 0.0)}
    centres = {"base": (0.0, 0.0, 0.5), "goal": (10.0, 0.0, 0.5)}
    # d2 is far from where its slot at the goal is; the group finishes when it
    # arrives, so the estimate must be its travel time.
    positions = {"d0": (0.0, 0.0, 0.5), "d1": (0.0, -2.0, 0.5), "d2": (10.0, 20.0, 0.5)}
    scattered = ExecutorTravelTimeProvider(centres, {"aav_formation": 1.0},
                                          member_slots={"aav_formation": slots},
                                          member_positions=positions)
    grouped = ExecutorTravelTimeProvider(centres, {"aav_formation": 1.0},
                                        member_slots={"aav_formation": slots},
                                        member_positions={})
    assert scattered("aav_formation", "base", "goal") > grouped("aav_formation", "base", "goal")
    assert scattered("aav_formation", "base", "goal") == pytest.approx(18.0)


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


# --- serial execution (task line) ------------------------------------------

def test_parallel_mode_still_starts_two_disjoint_units_together():
    """The default is unchanged: the offline cases keep their parallelism."""
    a = air_executor("A", ("d0",), speed=1.0)
    b = air_executor("B", ("d1",), speed=1.0)
    tasks = [task("first", agents=1, target="near"), task("second", agents=1, target="near")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base")
    starts = sorted(item.planned_start for item in plan.items)
    assert starts == [0.0, 0.0]


def test_serial_mode_serialises_two_disjoint_units():
    """The task line runs one unit at a time, shared members or not."""
    a = air_executor("A", ("d0",), speed=1.0)
    b = air_executor("B", ("d1",), speed=1.0)
    tasks = [task("first", agents=1, target="near"), task("second", agents=1, target="near")]
    plan = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base", serial=True)
    ordered = sorted(plan.items, key=lambda item: item.planned_start)
    assert ordered[0].planned_start == 0.0
    assert ordered[1].planned_start == pytest.approx(ordered[0].planned_finish)


def test_a_late_unit_this_task_does_not_use_does_not_delay_it():
    """Taking the maximum over every unit would stall on an unused resource."""
    a = air_executor("A", ("d0",), available=0.0, speed=1.0)
    late_b = air_executor("B", ("d1",), available=100.0, speed=1.0)
    plan = build_executor_plan([a, late_b], [task("first", agents=1, target="near")],
                               provider({"A": 1.0, "B": 1.0}),
                               initial_target_ref="base", serial=True)
    assert plan.items[0].executor_id == "A"
    assert plan.items[0].planned_start == 0.0


def test_units_outside_the_participants_do_not_hold_the_serial_clock():
    """A platform with no execution endpoint is planned for, never dispatched."""
    a = air_executor("A", ("d0",), speed=1.0)
    b = air_executor("B", ("d1",), speed=1.0)
    tasks = [task("first", agents=1, target="far"), task("second", agents=1, target="near")]
    without = build_executor_plan([a, b], tasks, provider({"A": 1.0, "B": 1.0}),
                                  initial_target_ref="base", serial=True,
                                  serial_units=set())
    starts = sorted(item.planned_start for item in without.items)
    assert starts == [0.0, 0.0]


def test_member_predicted_positions_advance_with_each_scheduled_item():
    """A group task must be costed from where its members are predicted to be.

    aav_1 flies to P first; the group task that follows has to pay for d0 coming
    from P, not from the group's old centre.
    """
    single = air_executor("aav_1", ("d0",), speed=1.0)
    group = air_executor("aav_formation", ("d0", "d1", "d2"), speed=1.0)
    centres = {"base": (0.0, 0.0, 0.0), "P": (10.0, 0.0, 0.0), "Q": (0.0, 20.0, 0.0)}
    slots = {"d0": (0.0, 0.0, 0.0), "d1": (0.0, -2.0, 0.0), "d2": (0.0, 2.0, 0.0)}
    travel = ExecutorTravelTimeProvider(
        centres, {"aav_1": 1.0, "aav_formation": 1.0},
        member_slots={"aav_formation": slots}, member_positions={})
    one = task("survey", agents=1, target="P")
    three = Task("gather", frozenset({"AIR"}), 3, 4.0, 1000.0, "Q",
                 allow_larger_unit=True)
    plan = build_executor_plan([single, group], [one, three], travel,
                               initial_target_ref="base", serial=True)
    by_task = {item.task_id: item for item in plan.items}
    assert by_task["survey"].executor_id == "aav_1"
    assert by_task["gather"].executor_id == "aav_formation"
    # d0 must travel from P to its slot at Q: sqrt(10^2 + 20^2), not 20.
    assert by_task["gather"].travel_time == pytest.approx(math.sqrt(500.0))
