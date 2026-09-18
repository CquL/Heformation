"""Completion-driven temporal repair; no task reallocation or invented wait."""

from dataclasses import replace
import math
from typing import MutableMapping, Tuple

from .models import DelayEvent, Plan, PlanItem
from .validation import NUMERICAL_TOLERANCE, identifier, nonnegative, validate_plan


def _event_item(plan: Plan, event: DelayEvent) -> PlanItem:
    for name in ("event_id", "execution_id", "task_id"):
        identifier(getattr(event, name), name)
    nonnegative(event.planned_finish, "event planned_finish")
    nonnegative(event.actual_finish, "event actual_finish")
    try:
        item = plan.item(event.execution_id)
    except KeyError as error:
        raise ValueError("unknown execution_id: {}".format(event.execution_id)) from error
    if item.task_id != event.task_id:
        raise ValueError("event task_id does not match its execution_id")
    if not math.isclose(item.planned_finish, event.planned_finish,
                        rel_tol=0.0, abs_tol=NUMERICAL_TOLERANCE):
        raise ValueError("event planned_finish differs from the dispatched plan")
    if event.actual_finish < item.planned_start - NUMERICAL_TOLERANCE:
        raise ValueError("actual_finish precedes planned_start")
    return item


def _copy_plan(plan: Plan) -> Plan:
    return Plan([replace(item) for item in plan.items])


def _same_result(first: DelayEvent, second: DelayEvent) -> bool:
    return (first.execution_id == second.execution_id and first.task_id == second.task_id
            and first.planned_finish == second.planned_finish
            and first.actual_finish == second.actual_finish)


def plan_repair(plan: Plan, event: DelayEvent) -> Tuple[Plan, bool]:
    """Return a new plan with successor release times repaired exactly once.

    The caller performs event deduplication (see process_completion). The
    completed item's planned timing stays historical; actual_finish provides
    the release boundary. Internal slots are derived from the member queue.
    """
    validate_plan(plan)
    item = _event_item(plan, event)
    if item.status != "COMPLETED":
        raise ValueError("repair requires the delayed execution to be COMPLETED")
    if event.delay_seconds <= 0:
        return plan, False
    representative_robot = item.coalition[0]
    queue = plan.robot_queues[representative_robot]
    delayed_slot = queue.index(item.execution_id)  # Internal zero-based slot.
    successor_ids = queue[delayed_slot + 1:]
    if any(plan.item(execution_id).status != "PLANNED" for execution_id in successor_ids):
        raise ValueError("repair cannot shift a successor that has already started or terminated")
    updated = _copy_plan(plan)
    release = event.actual_finish
    changed = False
    for execution_id in successor_ids:
        successor = updated.item(execution_id)
        start = max(successor.planned_start, release)
        finish = start + successor.travel_time + successor.wait_time + successor.service_time
        changed = changed or start != successor.planned_start or finish != successor.planned_finish
        successor.planned_start, successor.planned_finish = start, finish
        release = finish
    validate_plan(updated)
    return updated, changed


def process_completion(plan: Plan, event: DelayEvent,
                       final_events: MutableMapping[str, DelayEvent], *,
                       tolerance: float = 0.1,
                       repair_timing: bool = True) -> Tuple[Plan, bool]:
    """Validate one terminal result, freeze history, then gate temporal repair.

    final_events belongs to the caller, keyed by execution_id. Its event IDs
    are the processed_events set; no execution ledger is stored in Plan.
    A new delivery ID for the same execution/result remains a duplicate.
    The bool reports changes to remaining timing, not the COMPLETED transition.
    Invalid/conflicting events change neither the input plan nor the ledger.

    ``repair_timing=False`` is the repair-off control of Test C: the completion
    event is still validated, deduplicated and registered exactly once, but the
    successor timing is deliberately left untouched.  Neither mode changes when
    the next action is dispatched: both wait for the real previous completion.
    """
    nonnegative(tolerance, "tolerance")
    validate_plan(plan)
    item = _event_item(plan, event)
    for execution_id, previous in final_events.items():
        if execution_id != previous.execution_id:
            raise ValueError("final_events key does not match execution_id")
        if previous.event_id == event.event_id and not _same_result(previous, event):
            raise ValueError("event_id was already used for a different terminal result")
    previous = final_events.get(event.execution_id)
    if previous is not None:
        if not _same_result(previous, event):
            raise ValueError("execution already has a different terminal result")
        return plan, False
    if item.status not in ("RUNNING", "COMPLETED"):
        raise ValueError("completion requires a dispatched execution")
    updated = _copy_plan(plan)
    updated.item(event.execution_id).status = "COMPLETED"
    changed = False
    # Compare the threshold as a timestamp to avoid cancellation turning an
    # exact boundary such as 1.1 - 1.0 into a delay slightly greater than 0.1.
    if repair_timing and event.actual_finish > event.planned_finish + tolerance:
        updated, changed = plan_repair(updated, event)
    final_events[event.execution_id] = event
    return updated, changed
