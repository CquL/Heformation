from copy import deepcopy
from dataclasses import replace

import pytest

from mrta_python import DelayEvent, Plan, PlanItem, plan_repair, process_completion, validate_plan


COALITION = tuple("drone_{}".format(i) for i in range(7))


def plan():
    return Plan([
        PlanItem("e1", "T1", COALITION, 0, 10, 1, 0, 9, "RUNNING"),
        PlanItem("e2", "T2", COALITION, 10, 15, 2, 0, 3),
        PlanItem("e3", "T3", COALITION, 15, 20, 2, 0, 3),
    ])


def event(actual=13, event_id="d1"):
    return DelayEvent(event_id, "e1", "T1", 10, actual)


def test_completion_returns_new_plan_and_protects_completed_timing():
    original = plan()
    before = deepcopy(original)
    ledger = {}
    updated, changed = process_completion(original, event(), ledger)
    assert original == before
    assert updated is not original and changed
    assert updated.items[0] == replace(original.items[0], status="COMPLETED")
    assert [(i.planned_start, i.planned_finish) for i in updated.items] == [(0, 10), (13, 18), (18, 23)]
    assert ledger == {"e1": event()}
    validate_plan(updated)


def test_duplicate_event_and_execution_with_new_event_id_are_noops():
    ledger = {}
    updated, _ = process_completion(plan(), event(), ledger)
    for duplicate in (event(), event(event_id="different_delivery_id")):
        result, changed = process_completion(updated, duplicate, ledger)
        assert result is updated and not changed
        assert ledger == {"e1": event()}


def test_second_completion_uses_repaired_dispatch_finish():
    ledger = {}
    updated, _ = process_completion(plan(), event(), ledger)
    updated.items[1].status = "RUNNING"
    next_event = DelayEvent("d2", "e2", "T2", 18, 20)
    result, changed = process_completion(updated, next_event, ledger)
    assert changed
    assert result.items[1] == replace(updated.items[1], status="COMPLETED")
    assert result.items[2].planned_start == 20  # +3, then +2; not +3, then +5.
    assert result.items[2].planned_finish == 25
    assert result.items[0] == updated.items[0]


@pytest.mark.parametrize("actual", [9, 10, 10.05, 10.1])
def test_no_repair_for_early_zero_or_tolerated_completion(actual, monkeypatch):
    import mrta_python.repair as repair_module
    def unexpected(*args, **kwargs):
        raise AssertionError("repair must not be called within tolerance")
    monkeypatch.setattr(repair_module, "plan_repair", unexpected)
    original = plan()
    ledger = {}
    updated, changed = process_completion(original, event(actual), ledger, tolerance=0.1)
    assert not changed
    assert updated.items[0].status == "COMPLETED"
    assert updated.items[1:] == original.items[1:]
    assert "e1" in ledger


def test_large_delay_propagates_without_hard_deadline_failure():
    updated, changed = process_completion(plan(), event(1000), {})
    assert changed and updated.items[-1].planned_finish == 1010
    assert updated.items[0].planned_finish == 10


@pytest.mark.parametrize("bad_event", [
    replace(event(), execution_id="missing"), replace(event(), task_id="wrong"),
    replace(event(), planned_finish=9), replace(event(), actual_finish=float("nan")),
    replace(event(), actual_finish=float("inf")), replace(event(), actual_finish=-1),
    replace(event(), event_id=""),
])
def test_invalid_event_is_atomic_and_does_not_consume_id(bad_event):
    original = plan()
    before = deepcopy(original)
    ledger = {}
    with pytest.raises(ValueError):
        process_completion(original, bad_event, ledger)
    assert original == before and ledger == {}


def test_conflicting_terminal_result_is_rejected_without_mutation():
    ledger = {}
    updated, _ = process_completion(plan(), event(), ledger)
    before = deepcopy(updated)
    for conflict in (event(14), event(14, "different")):
        with pytest.raises(ValueError):
            process_completion(updated, conflict, ledger)
    assert updated == before and ledger == {"e1": event()}


def test_event_id_reused_for_another_execution_is_rejected():
    ledger = {}
    updated, _ = process_completion(plan(), event(), ledger)
    updated.items[1].status = "RUNNING"
    with pytest.raises(ValueError):
        process_completion(updated, DelayEvent("d1", "e2", "T2", 18, 20), ledger)
    assert set(ledger) == {"e1"}


def test_unstarted_task_or_started_successor_is_not_repaired():
    original = plan()
    original.items[0].status = "PLANNED"
    with pytest.raises(ValueError):
        process_completion(original, event(), {})
    original.items[0].status = "RUNNING"
    original.items[1].status = "RUNNING"
    with pytest.raises(ValueError):
        process_completion(original, event(), {})


def test_pure_repair_preserves_history_and_does_not_create_wait():
    original = plan()
    original.items[0].status = "COMPLETED"
    before = deepcopy(original)
    updated, changed = plan_repair(original, event())
    assert changed and original == before
    assert updated.items[0] == original.items[0]
    assert all(i.wait_time == 0 for i in updated.items)


def test_last_completion_only_updates_status_and_records_outcome():
    original = Plan(plan().items[:1])
    ledger = {}
    updated, changed = process_completion(original, event(), ledger)
    assert not changed
    assert updated.items[0] == replace(original.items[0], status="COMPLETED")
    assert ledger["e1"].actual_finish == 13


def test_stale_dispatch_finish_is_rejected_after_prior_repair():
    ledger = {}
    updated, _ = process_completion(plan(), event(), ledger)
    updated.items[1].status = "RUNNING"
    with pytest.raises(ValueError):
        process_completion(updated, DelayEvent("d2", "e2", "T2", 15, 20), ledger)
    assert set(ledger) == {"e1"}


def test_exact_tolerance_boundary_does_not_trigger_float_roundoff_repair(monkeypatch):
    import mrta_python.repair as repair_module
    def unexpected(*args, **kwargs):
        raise AssertionError("1.1 - 1.0 must not exceed the intended 0.1-second tolerance")
    monkeypatch.setattr(repair_module, "plan_repair", unexpected)
    original = Plan([PlanItem("e1", "T1", COALITION, 0, 1, 0, 0, 1, "RUNNING")])
    updated, changed = process_completion(original, DelayEvent("d1", "e1", "T1", 1, 1.1), {}, tolerance=0.1)
    assert not changed and updated.items[0].status == "COMPLETED"
