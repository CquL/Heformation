# FormationAction: fixed seven-member AIR execution

`Formation.action` is the ordinary ROS 1 actionlib boundary at `/formation_action`.
Each accepted, valid, executable goal publishes one `geometry_msgs/PoseStamped` to
`/move_base_simple/goal`. The seven native Swarm planners generate the individual
trajectories. This node never publishes `PositionCommand` or changes qn state.

The goal contains only `task_id`, `formation_center`, and `hold_duration`.
`formation_center` must have frame `world`, finite coordinates, and height 0.5 m;
`hold_duration` must be nonnegative. Invalid goals finish `ABORTED / INVALID_TARGET`
without publishing a Swarm goal. No target-frame transformation is performed.

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
| `startup_timeout` | `60.0` wall-clock seconds |
| `output_dir` | `/experiments/current` |

Exactly the seven IDs above are supported. The offsets have zero relative height.
The node checks every planner's `global_goal` offsets/scale, `manager/drone_id`, and
`fsm/flight_type=3` against its configuration. Startup additionally requires seven
fresh world-frame Odometry streams whose only publishers are the matching qn
nodes, the global map and seven local sensing depth streams, and established outgoing
TCPROS goal connections to the seven named planner nodes. ROS master subscription
names and public node XML-RPC `getBusInfo` are used; no private FSM state is inferred.
The sensing topics are `/drone_i_pcl_render_node/depth`, published periodically by
the upstream renderer at its configured sensing rate (30 Hz in the supplied launch).
The historical `cloud` remap in the upstream launch has no matching publisher.

`~ready` begins false and becomes true after these checks and action-server start.
`~readiness_reason` explains what is missing. Startup timeout shuts down the node.
The mission runner must wait for `/formation_action_server/ready` and actionlib
server availability. Every action rechecks Odometry before publishing its goal.

## Completion, failure, and cancellation

The ROS-independent `GroupCompletionMonitor` computes each target as
`center + swarm_scale * relative_slots[agent_id]`. All seven actual qn Odometry
positions and speeds must satisfy the configured bounds with fresh timestamps
for the complete requested hold duration. Position or speed excursions reset the
hold window; a monitor-evaluation gap longer than `odom_timeout` also restarts it.
Missing, stale, future-dated, nonfinite, or wrong-frame Odometry immediately
aborts with `ODOMETRY_TIMEOUT`; it cannot contribute to a successful hold.

Feedback reports `MOVING` or `HOLDING` whenever the phase changes. The result keeps
only actual start/finish ROS times and a reason code:

| Code | Meaning |
| --- | --- |
| `NONE=0` | No failure reason; inspect the actionlib terminal state |
| `INVALID_TARGET=1` | Invalid target or hold duration |
| `ODOMETRY_TIMEOUT=2` | A required actual state is unavailable or unusable |
| `EXECUTION_TIMEOUT=3` | Execution deadline or clock consistency failed |

Clients must require actionlib `SUCCEEDED`; `reason=0` alone is insufficient.
Cancellation returns `PREEMPTED` with actual timestamps, stops task monitoring,
and does not produce a successful completion. It does not stop the native Swarm
trajectory or qn plant. The first-stage runner aborts the mission on cancellation
or failure, and the experiment launcher tears down the isolated simulation.

`/drone_i_planning/finish` only supplies diagnostics. The native Bool has no
header or GoalID, so each timestamp is the first true message received during the
current action. It never changes the completion predicate. Swarm's trajectory
server continues commanding the final position while qn converges.

## Experiment records

Every attempt writes `<sanitized_task_id>_<start_ns>.diagnostics.json`; dispatched
actions additionally write `.monitor.csv`. Files are flushed before the terminal
Action result is sent. Diagnostics include the native `goal_id`, target and hold,
actual times, `terminal_state`, numeric `reason`, `goal_publish_count`, string-ID
`planner_nominal_finish_times`, nominal-to-actual differences, maximum position
error/speed, minimum member separation, per-member Odometry counts/rates, failure
IDs, and `successful_hold_window` (`start`, `end`, `duration`, `required_duration`).
Nonfinite invalid target coordinates are serialized as JSON null.

CSV columns are `ros_time`, `elapsed`, `phase`, `max_position_error`, `max_velocity`,
`min_inter_agent_distance`, `fresh_agent_count`, `stale_agent_ids` (semicolon
separated), and `hold_elapsed`. Extremes reflect observed monitor samples; the
experiment's rosbag permits independent reconstruction from all recorded qn
Odometry. These records are evidence, not additional Action Result fields.

Run the pure Python checks without unrelated host ROS pytest plugins:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q integration/qn_aav_simulator/tests
```

These tests do not replace the seven-member ROS simulation acceptance runs.
