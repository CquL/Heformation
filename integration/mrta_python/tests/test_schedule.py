from dataclasses import replace

import pytest

from mrta_python import Agent, Task, TravelTimeProvider, build_plan, validate_plan


def agents(available=0.0):
    return [Agent("drone_{}".format(i), frozenset({"AIR"}), available if i == 6 else 0.0)
            for i in range(7)]


def task(name, service=1.0, deadline=100.0, target="start"):
    return Task(name, frozenset({"AIR"}), 7, service, deadline, target)


def test_b_hand_calculation_dynamic_v9_order_and_times():
    travel = TravelTimeProvider({"start": (0, 0, 0.5), "far": (8, 0, 0.5),
                                 "near": (3, 0, 0.5)}, 1.0)
    tasks = [task("A", 1, target="near"), task("C", 2, 10, "far"),
             task("B", 4), task("U", 4, 6)]
    plan = build_plan(agents(2), tasks, travel, initial_target_ref="start", seed=0)
    assert [(i.task_id, i.planned_start, i.travel_time, i.service_time, i.planned_finish)
            for i in plan.items] == [
                ("U", 2, 0, 4, 6), ("B", 6, 0, 4, 10),
                ("C", 10, 8, 2, 20), ("A", 20, 5, 1, 26)]
    assert all(i.wait_time == 0 and len(i.coalition) == 7 for i in plan.items)
    assert plan.task_finish_times["C"] > 10
    assert plan.makespan == 26
    validate_plan(plan, agents(2), tasks)


def test_seed_breaks_complete_ties_and_is_input_order_independent():
    travel = TravelTimeProvider({"start": (0, 0, 0.5)}, 1)
    tasks = [task(name) for name in ("a", "b", "c")]
    def order(seed, inputs):
        return [i.task_id for i in build_plan(agents(), inputs, travel,
                initial_target_ref="start", seed=seed).items]
    assert order(0, tasks) == ["a", "c", "b"]
    assert order(0, list(reversed(tasks))) == order(0, tasks)
    assert order(1, tasks) == ["b", "a", "c"]


def test_provider_can_be_plain_callable_and_start_reference_is_explicit():
    calls = []
    def travel(executor, source, destination):
        calls.append((executor, source, destination))
        return 2.5
    plan = build_plan(agents(3), [task("one", target="destination")], travel,
                      initial_target_ref="chosen_start")
    assert calls == [(tuple(a.id for a in agents()), "chosen_start", "destination")]
    assert plan.items[0].planned_start == 3
    assert plan.items[0].planned_finish == 6.5


def test_travel_provider_uses_center_euclidean_distance():
    travel = TravelTimeProvider({"a": (0, 0, 0.5), "b": (3, 4, 0.5)}, 2)
    assert travel(("executor",), "a", "b") == 2.5


@pytest.mark.parametrize("bad", [0, -1, float("nan"), float("inf")])
def test_invalid_travel_speed_is_rejected(bad):
    with pytest.raises(ValueError):
        TravelTimeProvider({"start": (0, 0, 0.5)}, bad)


@pytest.mark.parametrize("bad_travel", [-1, float("nan"), float("inf")])
def test_bad_custom_provider_result_is_rejected(bad_travel):
    with pytest.raises(ValueError):
        build_plan(agents(), [task("one")], lambda *args: bad_travel,
                   initial_target_ref="start")


def test_incompatible_member_or_wrong_fixed_coalition_is_rejected():
    travel = TravelTimeProvider({"start": (0, 0, 0.5)})
    bad_agents = agents()
    bad_agents[0] = replace(bad_agents[0], capabilities=frozenset({"UUV"}))
    for inputs, tasks in [(bad_agents, [task("one")]), (agents()[:6], [task("one")]),
                          (agents(), [replace(task("one"), required_agent_count=6)])]:
        with pytest.raises(ValueError):
            build_plan(inputs, tasks, travel, initial_target_ref="start")


def test_zero_duration_and_distance_do_not_divide_by_zero():
    plan = build_plan(agents(), [task("zero", 0)], lambda *args: 0,
                      initial_target_ref="start")
    assert plan.items[0].planned_finish == 0
    validate_plan(plan)


def test_initial_availability_enters_deadline_reward():
    # Equal increments; availability changes which deadline flag is urgent.
    tasks = [task("long", 3, 100), task("urgent", 2, 4, "far")]
    travel = TravelTimeProvider({"start": (0, 0, 0.5), "far": (1, 0, 0.5)}, 1)
    early = build_plan(agents(), tasks, travel, initial_target_ref="start")
    late = build_plan(agents(2), tasks, travel, initial_target_ref="start")
    assert early.items[0].task_id == "long"
    assert late.items[0].task_id == "urgent"
