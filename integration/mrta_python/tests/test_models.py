from dataclasses import fields, replace

import pytest

from mrta_python import Agent, Plan, PlanItem, Task, build_plan, validate_plan


COALITION = tuple("drone_{}".format(i) for i in range(7))


def item():
    return PlanItem("e1", "T1", COALITION, 2, 7, 3, 0, 2)


def test_only_items_is_stored_and_all_views_follow_it():
    plan = Plan([item()])
    assert [field.name for field in fields(Plan)] == ["items"]
    assert plan.robot_queues == {a: ("e1",) for a in COALITION}
    assert plan.coalitions == {"e1": COALITION}
    assert plan.task_start_times == {"T1": 2}
    assert plan.task_finish_times == {"T1": 7}
    assert plan.waiting_times == {"e1": 0}
    plan.items.append(replace(item(), execution_id="e2", task_id="T2", planned_start=7, planned_finish=12))
    assert plan.robot_queues["drone_0"] == ("e1", "e2")
    assert plan.makespan == 12
    validate_plan(plan)


@pytest.mark.parametrize("changes", [
    {"planned_finish": 8}, {"travel_time": -1}, {"wait_time": 1},
    {"planned_start": float("nan")}, {"service_time": float("inf")},
    {"coalition": COALITION[:-1]}, {"coalition": ("drone_0",) * 7},
    {"status": "unknown"}, {"execution_id": ""},
])
def test_plan_validator_rejects_invalid_time_identity_and_domain(changes):
    with pytest.raises(ValueError):
        validate_plan(Plan([replace(item(), **changes)]))


def test_plan_validator_rejects_overlap_and_duplicate_identifiers():
    for other in [replace(item(), execution_id="e2", task_id="T2"),
                  replace(item(), task_id="T2", planned_start=7, planned_finish=12),
                  replace(item(), execution_id="e2", planned_start=7, planned_finish=12)]:
        with pytest.raises(ValueError):
            validate_plan(Plan([item(), other]))


def test_empty_plan_and_duplicate_input_ids():
    agents = [Agent(a, frozenset({"AIR"})) for a in COALITION]
    task = Task("T", frozenset({"AIR"}), 7, 1, 10, "start")
    assert build_plan(agents, [], lambda *args: 0, initial_target_ref="start") == Plan()
    assert Plan().makespan == 0
    for bad_agents, tasks in [(agents[:-1] + [agents[0]], [task]), (agents, [task, task])]:
        with pytest.raises(ValueError):
            build_plan(bad_agents, tasks, lambda *args: 0, initial_target_ref="start")
