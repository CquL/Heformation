# Calvo v9 restricted-domain Python port

This project-owned port implements the enabled v9 reward terms and a temporal
repair subset from `upstream/Calvo-MRTA`. It is not a complete MATLAB-equivalent
reproduction. The original MIT notice is retained in `LICENSE`.

## Fixed domain

- Exactly seven agents execute every task as one unchanged coalition.
- `Relayability=0`, `nf=1`, `N=required_agent_count=7`, and `Te=service_time`.
- Compatibility means each member contains all required capabilities. For an
  accepted task, `nc=7`, so the v9 ratio `N/nc` is always 1.
- Deadline (`tmax`) is soft. Late tasks stay allocated; deadline overruns are
  reported by the experiment layer, not treated as repair infeasibility.
- Battery is a non-binding fixed assumption. Recharge, fragmentation, relay
  tasks, and dynamic coalition size are disabled and absent from public models.
- There is no task release time or external synchronization partner. Planned
  waiting is zero. Positive-wait absorption is deferred until a later domain
  supplies a real source of waiting; this port rejects positive planned waits.
- No v13 reallocation or arbitrary precedence optimization is implemented.

## Planning and time

```python
from mrta_python import Agent, Task, TravelTimeProvider, build_plan

agents = [Agent("drone_{}".format(i), frozenset({"AIR"})) for i in range(7)]
tasks = [Task("survey", frozenset({"AIR"}), 7, 5.0, 100.0, "A")]
travel = TravelTimeProvider({"start": (0, 0, 0.5), "A": (3, 0, 0.5)}, 1.5)
plan = build_plan(agents, tasks, travel, initial_target_ref="start", seed=0)
```

`build_plan` also accepts any callable with signature
`travel_time_provider(executor, from_target_ref, to_target_ref) -> seconds`.
It never reads the provider's internal location data. The default provider uses
formation-center Euclidean distance divided by nominal formation speed.

All plan times are seconds relative to one mission epoch. `planned_start` is
the start of group resource occupancy and travel. Every item obeys:

```text
planned_finish = planned_start + travel_time + wait_time + service_time
```

The initial virtual queue finish is `M=max(agent.available_from)`. Thereafter
`M` is the latest allocated finish. A candidate finishes at `M+travel+service`,
so its introduced makespan is `max(0, candidate_finish-M)`. Availability is not
also charged as travel or waiting. This initial-availability mapping extends
upstream's zero-initialized queue; no nonzero-availability MATLAB parity is claimed.

Each allocation round uses the v9 descending lexicographic reward:

```text
(deadline < M + 1.55*service,
 1/introduced_makespan,
 1/introduced_joint_wait,
 service,
 N/nc,
 1/mean_travel)
```

The implementation uses the mathematically equivalent ascending key
`(-urgent, introduced_makespan, 0, -service, -1, travel)`. This preserves the
zero-cost/infinite-reciprocal ordering without division by zero. It does not
add earliest-deadline-first or actual-lateness tie breakers. Tasks are sorted
by ID, shuffled with a local `random.Random(seed)` every round, then stably
compared. Record the seed and Python version; this is reproducible within the
Python implementation, not the original MATLAB `rng('shuffle')` sequence.

Source anchors in `upstream/Calvo-MRTA/`:

- `src/heuristicTaskAllocator.m:467-562`: candidate computation and six terms.
- `src/heuristicTaskAllocator.m:760-776,993-1012`: joint waiting and makespan.
- `scripts/checkSolution.m:109-117`: deadlines are no longer hard constraints.
- `src/planRepair.m:44-116,284-311,388-401`: temporal repair and protected slots.

`Plan` stores only `items`. `robot_queues` maps agent IDs to execution-ID tuples;
`coalitions` and `waiting_times` use execution IDs; `task_start_times` and
`task_finish_times` use task IDs. These properties and `makespan` are derived
afresh from `items`. Task IDs and execution IDs must be unique in this domain.
`validate_plan(plan, agents, tasks)` additionally checks input compatibility,
availability and complete task coverage; `validate_plan(plan)` checks the plan
alone, including finite nonnegative times, fixed membership and no overlap.

## Completion and repair

```python
from mrta_python import DelayEvent, process_completion

final_events = {}  # Owned by this mission runner; execution_id -> DelayEvent.
item = plan.items[0]
item.status = "RUNNING"
dispatched_finish = item.planned_finish
# Dispatch the Action, then record its native GoalID against this execution_id.
event = DelayEvent("completion-1", item.execution_id, item.task_id,
                   dispatched_finish, dispatched_finish + 2.0)
plan, remaining_timing_changed = process_completion(
    plan, event, final_events, tolerance=0.1)
```

The planner generates deterministic execution IDs that remain stable throughout
the mission. ActionClient generates native GoalIDs independently. The runner
observes native ActionGoal/ActionResult envelopes and records a one-to-one
execution-ID/GoalID association; the strings need not be identical. After each
returned plan replaces the previous one, look up items again rather than
retaining an old PlanItem reference. Capture the current planned finish when dispatching;
after an earlier repair, comparing against the original mission's finish would
count the old delay twice.

`process_completion` validates identifiers and dispatch timing, returns a new
Plan with the item marked `COMPLETED`, then calls `plan_repair` only for a delay
above tolerance. The input Plan is unchanged. The completed item's planned
timing remains historical; its actual finish is stored in the immutable event
and sets the next resource-release boundary. Each unstarted successor uses:

```text
start' = max(old_start, predecessor_release)
finish' = start' + travel + service
```

Order and coalition remain fixed. No service time, Agent availability, or
invented waiting is added. Internal zero-based slots come from derived member
queues; one representative member locates the group event, and its delay is
applied once. This is the temporal subset, not a general MATLAB repair port.
The runner must finish processing the result before starting the next Action.
Repair rejects a successor that has already started or terminated.

`final_events` is the sole runtime terminal-result registry. Its values' event
IDs form the processed-event set; no such set is stored in Plan. Same-result
redelivery is a no-op even with a different event ID. Reused IDs with different
content, conflicting final results for one execution, unknown executions,
stale dispatch finish times and invalid timestamps raise `ValueError` without
changing the plan or registry. Tolerated, zero and negative delays record the
completion but do not call repair. The return bool describes successor timing
changes, so the final task can complete late with `False` and no plan-time edit.

`plan_repair(plan, event) -> (Plan, bool)` is the lower-level temporal operation
for an already `COMPLETED` item; the runtime should use `process_completion`.
Actual mission duration comes from terminal events, whereas `Plan.makespan`
remains a statistic of planned timings.

## Installation and checks

The package supports Python 3.8+ with no runtime dependencies and imports as
`mrta_python`. A regular install from the repository root is:

```bash
python3 -m venv /tmp/mrta-python-env
/tmp/mrta-python-env/bin/python -m pip install ./integration/mrta_python
/tmp/mrta-python-env/bin/python -c 'import mrta_python; print(mrta_python.__file__)'
```

Run focused tests using an environment with pytest installed. Disabling plugin
autoload prevents unrelated host ROS pytest plugins from affecting these tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -B -m pytest -q -p no:cacheprovider integration/mrta_python/tests
```

Test B's independent hand calculation uses speed 1 m/s, start x=0, initial
group availability 2 seconds and all targets at z=0.5:

| Task | Target x | Service | Deadline | Expected start | Expected finish |
|---|---:|---:|---:|---:|---:|
| U | 0 | 4 | 6 | 2 | 6 |
| B | 0 | 4 | 100 | 6 | 10 |
| C | 8 | 2 | 10 | 10 | 20 |
| A | 3 | 1 | 100 | 20 | 26 |

U wins the initial urgency term; B beats A on service after a makespan tie;
C becomes urgent at M=10 and finishes late without being rejected. Other tests
cover reproducible ties, capability rejection, plan views, temporal invariants,
history protection, terminal-result conflicts and repeated-delay prevention.
These checks do not establish ROS motion completion or full MATLAB equivalence.

## Observed repair behaviour in the ROS mission (Test C)

`./scripts/docker_test_qn_formation_action.sh mission 1.5 <dir> on|off` runs the
`A -> B -> Return` chain with `planRepair` enabled or disabled and records both the
plan history and the actual dispatch times. The two runs below differ **only** in
`repair_mode`:

| Run | `plan_updated` | `plan_updated_actions` | `updated_plan_used` | `dispatch_changed` |
| --- | --- | --- | --- | --- |
| `experiments/20260918-mission-e` (on) | `true` | `T1`, `T2` | `true` | `T2`, `T3` |
| `experiments/20260918-mission-f` (off) | `false` | — | `true` | `T2`, `T3` |

Both runs waited for each action's real completion before dispatching the next one
(T2 dispatched 0.14 s after T1 finished, T3 0.17 s after T2, in the repair-off run);
repair-off never skipped a real completion to manufacture a difference. Every
`DelayEvent` is registered once (`events == processed_events`, three events), each
plan history entry keeps the pre-repair plan (`plan_before`), and the dispatch of
each successor matches the `planned_start` of the plan the runner last read.

`dispatch_changed` is therefore **not** a measure of repair's effect in this serial
single-resource scenario: it is `true` whenever the actual release of the resource
lands away from the plan's own `planned_start`, which happens with or without
repair. Repair's effect is visible in `plan_updated` and in the repaired
`planned_start` values, not in whether the physical dispatch moved.

## Offline executor layer (plan.md P3)

`executors.py` adds the offline multi-resource model that plan.md places after
P0–P2. It is a **planning-layer** addition: it selects which pre-defined execution
unit runs a task and how tasks compete for one unit. It does not split the
seven-member physical formation the ROS mission uses, and it claims no
multi-coalition physical closed loop.

| Field | Meaning |
| --- | --- |
| `executor_id` | the bookable unit |
| `physical_agent_ids` | the unit's fixed membership; never split or reshuffled |
| `capabilities` | what the unit can do |
| `available_from` | when the unit becomes bookable |
| `nominal_speed_mps` | the unit's travel speed, so two eligible units cost differently |

```python
from mrta_python import Executor, ExecutorTravelTimeProvider, Task, build_executor_plan

units = [Executor("A", ("d0", "d1"), frozenset({"AIR"}), 0.0, 1.0),
         Executor("B", ("d2", "d3"), frozenset({"AIR"}), 0.0, 2.0)]
tasks = [Task("survey", frozenset({"AIR"}), 2, 4.0, 1000.0, "far")]
travel = ExecutorTravelTimeProvider({"base": (0, 0, 0.5), "far": (10, 0, 0.5)},
                                    {"A": 1.0, "B": 2.0})
plan = build_executor_plan(units, tasks, travel, initial_target_ref="base")
assert plan.assignments == {"survey": "B"}   # the cheaper capable unit wins
```

Selection reuses the ported v9 reward order. The one generalization is that each
unit keeps its own queue finishing time, so a candidate's earliest start is *that
unit's* queue end — identical to the single global queue when only one unit
exists. Capability and size decide eligibility; cost decides between eligible
units; the same unit's queue serializes competing tasks. Ties break on
`executor_id`, so the result does not depend on input order.

The three scenarios plan.md requires are regression tests:

| Scenario | Test |
| --- | --- |
| Only unit A is eligible | `test_only_the_eligible_executor_is_selected`, `test_ineligible_executor_cannot_be_forced_into_the_plan` |
| A and B are both eligible at different cost | `test_lower_cost_wins_between_two_eligible_executors`, `test_selection_compares_the_same_task_between_units` |
| Two tasks compete for one unit | `test_two_tasks_compete_for_one_executor_and_serialize`, `test_competition_is_avoided_when_a_second_eligible_unit_exists` |

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=integration python -B -m pytest -q \
  -p no:cacheprovider integration/mrta_python/tests/test_executors.py
```

`validate_executor_plan` rejects an ineligible unit, a coalition that differs
from the unit's membership, overlapping tasks on one unit, and a task list that
is not allocated exactly once. Completion/repair for this layer is deliberately
out of scope at this stage: the ported `process_completion`/`plan_repair` remain
the fixed-coalition semantics and are not duplicated here.
