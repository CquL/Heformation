# FormationAction: fixed seven-member AIR execution

`Formation.action` is the ordinary ROS 1 actionlib boundary at `/formation_action`.
Each accepted, valid, executable goal publishes one `geometry_msgs/PoseStamped` to
`/move_base_simple/goal`. The seven native Swarm planners generate the individual
trajectories. This node never publishes `PositionCommand` or changes qn state.

The goal contains only `task_id`, `formation_center`, and `hold_duration`.
`formation_center` must have frame `world`, finite coordinates, and height 0.5 m;
`hold_duration` must be nonnegative. Invalid goals finish `ABORTED / INVALID_TARGET`
without publishing a Swarm goal. No target-frame transformation is performed.

## Odometry semantics

The qn node publishes two different odometry streams from one state snapshot.
They are deliberately not interchangeable:

| Topic | Frame | Twist | Consumer |
| --- | --- | --- | --- |
| `/drone_i_qn/odometry` | `world` → `drone_i/base_link` | body-frame linear and angular velocity | generic ROS consumers, this node |
| `/drone_i_visual_slam/odom` (remap of `~odometry_swarm_compat`) | `world` → `drone_i/swarm_compat` | world-frame linear velocity | the unchanged Swarm planners |

The compatibility topic keeps the numbers Swarm already consumed before the qn
bridge existed and is explicitly not a general ROS Odometry. `docs/QN_INTEGRATION.md`
records the qn 12ODE heave-row convention that makes the third component of the
world-frame velocity differ in sign from a strict `R(q) * v_body` rotation.

## Configuration and startup

The node reads private parameters:

| Parameter | Default |
| --- | --- |
| `agent_ids` | `[0, 1, 2, 3, 4, 5, 6]` |
| `relative_slots` | Original normal-hexagon offsets, string ID → three-element list |
| `swarm_scale` | `2.0` |
| `epsilon_p` | `0.5` m |
| `epsilon_v` | `0.25` m/s |
| `odom_timeout` | `0.25` s |
| `execution_timeout` | `180.0` s |
| `monitor_rate` | `20.0` Hz |
| `startup_timeout` | `120.0` wall-clock seconds |
| `sensor_backend` | `CPU_POINTCLOUD` |
| `known_empty_map` | `false` |
| `cloud_timeout` | `2.0` s |
| `qn_state_timeout` | `1.0` s |
| `model_time_window` | `0.06` s |
| `time_baseline_seconds` | `30.0` s |
| `time_rate_lower` / `time_rate_upper` | `0.95` / `1.05` |
| `max_model_ros_drift` | `0.05` s |
| `max_cross_agent_drift` | `0.05` s |
| `time_reference_displacement_limit` | `0.5 * epsilon_p` |
| `inter_agent_clearance` | `0.5` m |
| `obstacle_clearance` | `0.2` m |
| `obstacle_sample_period` | `1.0` s |
| `min_valid_sample_ratio` | `0.9` |
| `platform_radius_m` | `0.25` m |
| `surface_plane_m` | `0.0` m |
| `output_dir` | `/experiments/current` |

Exactly the seven IDs above are supported. The offsets have zero relative height.
The node checks every planner's `global_goal` offsets/scale, `manager/drone_id`, and
`fsm/flight_type=3` against its configuration.

## Readiness

The process starts immediately, but only `READY_IDLE` accepts a Goal. Readiness is
reported in three levels so "the topic exists" is never confused with "the input is
usable for this plan":

1. the topic is present on the ROS master,
2. a message has actually been received on it,
3. the received input is valid for the configured `sensor_backend`.

`CPU_POINTCLOUD` waits for the real global cloud and the planning interface;
`CUDA_DEPTH` additionally waits for the seven depth streams. An empty cloud alone
never proves "known empty world": it is accepted only with an explicit
`known_empty_map=true`. Runtime health continuously checks global-map
initialisation, local cloud freshness, the seven Odometry streams, planner health,
and the qn state update time.

Outputs: `~ready`, `~readiness_reason`, `sensor_backend`, `missing_topics`,
`stale_topics`, `planner_health`. Startup timeout shuts the node down. The mission
runner waits for `/formation_action_server/ready` and actionlib server availability.

## Goal callback and the resource state machine

`goal_callback` only validates the Goal, checks `READY_IDLE` and reserves the group
resource in one atomic operation, accepts or rejects, hands the work to a single
execution loop, and returns. The full task loop never runs inside the callback, and
the idle check and the reservation cannot be separated.

```text
BOOTING -> READY_IDLE -> ACTIVE -> HOLDING -> SUCCEEDED -> READY_IDLE
                            \-> UNKNOWN_LOCKED (fault, no automatic recovery)
```

* A goal while `ACTIVE`/`HOLDING` is rejected; the running task is never preempted.
* A cancel request never calls `set_preempted()` and never releases resources,
  overwrites the current reference, or dispatches the next task. actionlib may show
  `PREEMPTING`; that is not a physical cancellation.
* Odometry loss, execution timeout and result timeout enter `UNKNOWN_LOCKED`.
* Only a completed hold releases the resource and returns to `READY_IDLE`.
* Recovering from `UNKNOWN_LOCKED` requires restarting the whole simulated
  execution chain (trajectory servers, qn nodes, action server); clearing the lock
  alone is not recovery.

## Completion

The ROS-independent `GroupCompletionMonitor` computes each target as
`center + swarm_scale * relative_slots[agent_id]`. All seven actual qn Odometry
positions and speeds must satisfy the configured bounds over the **whole** window of
states received since the previous evaluation (about 250 ms), not merely at the
20 Hz sample instants, for the complete requested hold duration. Position or speed
excursions reset the hold window; a monitor-evaluation gap longer than
`odom_timeout` also restarts it. Missing, stale, future-dated, nonfinite, or
wrong-frame Odometry immediately aborts with `ODOMETRY_TIMEOUT`.

The hold is additionally gated on **model** time: every member's qn model clock must
advance by at least `hold_duration` between the start and end of the hold, computed
from the `/drone_i_qn/diagnostics` stream. ROS dwell time alone never completes a
task.

Feedback reports `MOVING` or `HOLDING` whenever the phase changes. The result keeps
actual start/finish ROS times and a reason code:

| Code | Meaning |
| --- | --- |
| `NONE=0` | No failure reason; inspect the actionlib terminal state |
| `INVALID_TARGET=1` | Invalid target or hold duration |
| `ODOMETRY_TIMEOUT=2` | A required actual state is unavailable or unusable |
| `EXECUTION_TIMEOUT=3` | Execution deadline or clock consistency failed |
| `REFERENCE_ADOPTION_UNCONFIRMED=4` | The new reference could not be attributed to every qn |
| `MODEL_TIME_MISMATCH=5` | Model hold, time alignment or a task metric failed |
| `UNKNOWN_LOCKED=6` | Fault; restart the execution chain |

`REFERENCE_ADOPTION_UNCONFIRMED` sets `experiment_validity=INCOMPLETE`;
`MODEL_TIME_MISMATCH` and the metric failures set `INVALID`.

Clients must require actionlib `SUCCEEDED`; `reason=0` alone is insufficient. The
first-stage runner aborts the mission on cancellation or failure, and the
experiment launcher tears down the isolated simulation.

## Reference adoption

`PositionCommand.stamp > dispatch_time` is explicitly *not* accepted as proof that
qn adopted a new task reference: the previous trajectory keeps publishing
end-of-trajectory hold commands with fresh stamps. Each task records an
`execution_id` ↔ Action GoalID ↔ dispatch time ↔ per-member pre/post
`trajectory_id` sets ↔ qn `source_trajectory_id` ↔ `used_outer_step` chain. A task
counts as adopting its new reference only when, for every member, a post-dispatch
trajectory exists and qn's `source_trajectory_id` belongs to that new trajectory;
otherwise it is reported as `REFERENCE_ADOPTION_UNCONFIRMED`. Local replanning
during a task keeps the same execution context. The group goal is published exactly
once per task by the authorised publisher.

## Verdicts and evidence

Every attempt writes `<sanitized_task_id>_<start_ns>.diagnostics.json`; dispatched
actions additionally write `.monitor.csv`. Files are flushed before the terminal
Action result is sent.

Diagnostics report three independent results:

```text
task_outcome        = PASS | FAIL
safety_outcome      = PASS | FAIL | NOT_VERIFIED
experiment_validity = VALID | INVALID | INCOMPLETE
```

* `task_outcome` needs physical completion, proven reference adoption, a satisfied
  model-time hold, compatible model/ROS time, `final_slot_error <= epsilon_p` and
  `hold_velocity <= epsilon_v`.
* `safety_outcome` is an independent discrete-sample check: minimum inter-agent
  distance and minimum member-to-cloud clearance. Reaching the slots after a
  collision is `task_outcome=PASS, safety_outcome=FAIL`. When no clearance sample
  could be taken the result is `NOT_VERIFIED`, never a silent pass, and the evidence
  kind is reported as `DISCRETE_SAMPLED`.
* `experiment_validity` covers sample accounting and time alignment. Missing or
  unalignable samples are reported through `valid_sample_count`,
  `valid_sample_ratio`, `max_continuous_gap_s` and `alignment_failure_count`
  instead of being silently dropped. An unproven reference adoption is
  `INCOMPLETE`, not `VALID`.

Reported metrics are kept separate: final slot error, trajectory tracking error
against the used reference, velocity tracking error, hold speed, and formation
shape error. Extremes reflect observed monitor samples; the experiment's rosbag
permits independent reconstruction from all recorded qn Odometry. These records are
evidence, not additional Action Result fields.

Run the pure Python checks without unrelated host ROS pytest plugins:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q integration/qn_aav_simulator/tests
```

These tests do not replace the seven-member ROS simulation acceptance runs.
