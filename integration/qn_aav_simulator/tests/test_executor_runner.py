"""Task-line dispatch contract using native Action boundary doubles."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

from test_formation_action_server import server_module
from mrta_python import ExecutorPlan, ExecutorPlanItem
from qn_aav_simulator.executor_routing import load_routing, dispatchable_units
from qn_aav_simulator.task_line import load_request, load_formation_phase
from qn_aav_simulator.monitoring_request import expand


@pytest.fixture
def runner_module(server_module, monkeypatch):
    msg = sys.modules["qn_aav_simulator.msg"]
    for name in ("FormationActionGoal", "FormationActionResult"):
        monkeypatch.setattr(msg, name, object, raising=False)
    def goal():
        return SimpleNamespace(formation_center=SimpleNamespace(
            header=SimpleNamespace(), point=SimpleNamespace()))
    monkeypatch.setattr(msg, "FormationGoal", goal, raising=False)
    monkeypatch.setattr(server_module.rospy, "Duration", lambda t: t, raising=False)
    path = Path(__file__).resolve().parents[1] / "scripts/formation_mission_runner.py"
    spec = importlib.util.spec_from_file_location("runner_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_runner(module, tmp_path):
    runner = module.MissionRunner.__new__(module.MissionRunner)
    runner.output = tmp_path
    entries = [{"executor_id": "aav_"+str(i+1), "physical_agent_ids": ["drone_"+str(i)],
                "capabilities": ["AIR"], "action_endpoint": "/selected_"+str(i)+"/formation_action"}
               for i in range(3)]
    entries.append({"executor_id": "group", "physical_agent_ids": ["drone_0", "drone_1", "drone_2"],
                    "capabilities": ["AIR"], "action_endpoint": "/selected_group/action"})
    runner.routing = load_routing(entries, default_members=(), default_initial_target_ref="start")
    runner.units = dispatchable_units(runner.routing)
    runner.fleet = ["drone_0", "drone_1", "drone_2"]
    runner.member_slots = {u.executor_id: {m: (0, 0, 0) for m in u.physical_agent_ids} for u in runner.units}
    runner.member_slots["group"] = {"drone_0": (0, 0, 0), "drone_1": (0, -2, 0), "drone_2": (0, 2, 0)}
    runner._actual_positions = lambda: {"drone_0": (-30, 6, .8), "drone_1": (-30, 4, .8), "drone_2": (-30, 8, .8)}
    path = Path(__file__).resolve().parents[1] / "config/monitoring_request_coastal.yaml"
    runner.request = load_request(path)
    runner.formation_phase = load_formation_phase(path)
    runner.speed, runner.seed, runner.epoch = 1.5, 0, 0
    runner.active_executor_ids = set()
    runner.final_events = {}
    runner.metrics = {"executions": [], "results_received": []}
    runner._save_executor = lambda: None
    runner._wait_executor_ready = lambda unit: None
    return runner


def test_request_builds_one_serial_plan_including_assembly_and_transfer(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    plan = runner._executor_plan(expand(runner.request), include_formation=True)
    assert len(plan.items) == 5
    assert all(b.planned_start >= a.planned_finish for a, b in zip(plan.items, plan.items[1:]))
    assert [i.executor_id for i in plan.items[-2:]] == ["group", "group"]
    assert plan.items[0].coalition == ("drone_1",)
    assert len({i.execution_id for i in plan.items}) == len(plan.items)


def test_unconfirmed_request_never_dispatches(runner_module, tmp_path, monkeypatch):
    runner = make_runner(runner_module, tmp_path)
    runner._dispatch_executor_item = lambda _: pytest.fail("dispatch without confirmation")
    monkeypatch.setattr("builtins.input", lambda _: "no")
    runner._run_executor()
    assert runner.metrics["status"] == "NOT_CONFIRMED"
    assert not runner.active_executor_ids


def dispatch_setup(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    runner.plan = runner._executor_plan(expand(runner.request))
    item = runner.plan.items[0]
    unit = runner.routing[item.executor_id]
    runner.server_nodes = {unit.executor_id: "/selected_server"}
    return runner, item, unit


def test_timeout_keeps_members_reserved(runner_module, tmp_path, monkeypatch):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    sent = []
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda g, **_: sent.append(g),
        wait_for_result=lambda _: False, get_state=lambda: 1)}
    times = iter([0, 1000])
    monkeypatch.setattr(runner_module.time, "monotonic", lambda: next(times))
    with pytest.raises(RuntimeError, match="result timeout"):
        runner._dispatch_executor_item(item)
    assert len(sent) == 1
    assert runner.active_executor_ids == {unit.executor_id}
    assert item.status == "RUNNING"  # outer handler records UNKNOWN_LOCKED


def test_selected_client_and_successful_result_release_next_item(runner_module, tmp_path):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    result = SimpleNamespace(task_id=item.execution_id, goal_id="native-goal", reason=0,
        task_outcome=1, safety_outcome=1, experiment_validity=1,
        evidence_file="result.json", actual_finish_time=SimpleNamespace(to_sec=lambda: 20))
    (tmp_path / "result.json").write_text(json.dumps({"goal_id": "native-goal",
        "accepted_for_dispatch": True, "resource_released": True}))
    sent = []
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda g, **_: sent.append((unit.action_endpoint,g)),
        wait_for_result=lambda _: True, get_state=lambda: 3, get_result=lambda: result)}
    runner.native_result = lambda _: ("native-goal", SimpleNamespace(status=SimpleNamespace(status=3)))
    runner._receive_observations = lambda *_: None
    runner._refresh_executor_timing = lambda _: None
    runner._dispatch_executor_item(item)
    assert sent[0][0] == "/selected_1/formation_action"
    assert not runner.active_executor_ids
    assert runner.plan.items[0].status == "COMPLETED"
    assert runner.plan.items[1].planned_start >= 20


@pytest.mark.parametrize("field,value", [("safety_outcome",2), ("experiment_validity",2),
                                        ("reason",6), ("task_outcome",2), ("task_id","old")])
def test_bad_result_cannot_release(runner_module, field, value):
    result = SimpleNamespace(task_id="new", goal_id="goal", reason=0, task_outcome=1,
                             safety_outcome=1, experiment_validity=1)
    setattr(result, field, value)
    assert not runner_module.MissionRunner._release_result_ok(3, result, "new")


def test_serial_completion_repairs_disjoint_successor_and_deduplicates():
    from mrta_python import DelayEvent
    from mrta_python.repair import process_executor_completion
    first = ExecutorPlanItem('e1', 't1', 'u1', ('a',), 0, 2, 1, 0, 1, 'RUNNING')
    second = ExecutorPlanItem('e2', 't2', 'u2', ('b',), 2, 5, 2, 0, 1)
    plan = ExecutorPlan([first, second])
    event = DelayEvent('result', 'e1', 't1', 2, 7)
    events = {}
    updated, changed = process_executor_completion(plan, event, events)
    assert changed and updated.items[1].planned_start == 7
    assert updated.items[1].planned_finish == 10
    assert plan.items[0].status == 'RUNNING'
    repeated, changed = process_executor_completion(updated, event, events)
    assert repeated is updated and not changed
    with pytest.raises(ValueError, match='different terminal result'):
        process_executor_completion(updated, DelayEvent('other', 'e1', 't1', 2, 8), events)


def test_actual_member_position_changes_later_group_cost(runner_module, tmp_path):
    runner = make_runner(runner_module, tmp_path)
    runner.plan = runner._executor_plan(expand(runner.request), include_formation=True)
    for item in runner.plan.items[:-2]:
        item.status = 'COMPLETED'
    runner.plan_revision = 0
    runner.metrics['plan_history'] = []
    # AAV1 was actually left at P; other members remain at their own initial positions.
    runner._actual_positions = lambda: {'drone_0': (-10, 6, .8),
        'drone_1': (-30, 4, .8), 'drone_2': (-30, 8, .8)}
    runner._refresh_executor_timing(50)
    assert runner.plan.items[-2].travel_time == pytest.approx(20 / 1.5)
    assert runner.plan.items[-2].planned_start >= 50


def test_failed_native_result_is_received_but_does_not_release(runner_module, tmp_path):
    runner, item, unit = dispatch_setup(runner_module, tmp_path)
    result = SimpleNamespace(task_id=item.execution_id, goal_id="failed-goal", reason=6,
        task_outcome=2, safety_outcome=2, experiment_validity=2, evidence_file="failure.json")
    (tmp_path / "failure.json").write_text(json.dumps({"goal_id": "failed-goal",
        "reason_text": "inter-agent surface clearance below limit"}))
    runner.clients = {unit.executor_id: SimpleNamespace(send_goal=lambda *a, **k: None,
        wait_for_result=lambda _: True, get_state=lambda: 4, get_result=lambda: result)}
    with pytest.raises(RuntimeError, match="cannot release"):
        runner._dispatch_executor_item(item)
    assert runner.active_executor_ids == {unit.executor_id}
    assert runner.metrics["results_received"] == [item.task_id]
    assert runner.metrics["executions"][0]["result"] == "NOT_ACCEPTED"
    assert not runner.metrics["executions"][0]["plan_updated"]
    assert "inter-agent surface clearance" in runner.metrics["executions"][0]["detail"]
