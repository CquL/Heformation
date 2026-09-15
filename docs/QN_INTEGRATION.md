# qn integration

The upstream Swarm-Formation planner, optimizer, map, swarm graph, messages, and trajectory
server are unchanged. The platform boundary is replaced as follows:

```text
quadrotor_msgs/PositionCommand
  position, velocity, acceleration, yaw, yaw_dot
                    |
                    v
qn original RBF/PD controller
                    |
                    v
10 first-order actuators
                    |
                    v
qn 6DOF AIR/TRANSITION/WATER plant
                    |
                    v
nav_msgs/Odometry + medium_flag
                    |
                    v
Swarm-Formation planner feedback
```

The upstream source remains under `upstream/Swarm-Formation`. During the qn image build,
`integration/swarm_qn_bridge/simulator.xml` overlays the upstream simulator launch and replaces
`poscmd_2_odom` with `qn_aav_simulator/qn_aav_node.py`, preserving the command and odometry remaps.

The qn numerical equations are copied from the previously validated Python transcription:

- `integration/qn_aav_simulator/src/qn_aav_simulator/qn_dynamics.py`
- `integration/qn_aav_simulator/src/qn_aav_simulator/qn_python_backend.py`
- `integration/qn_aav_simulator/src/qn_aav_simulator/contracts.py`

Only the runtime-evaluated `tuple[...]` type aliases were changed to `typing.Tuple[...]` for ROS
Noetic's Python 3.8. No numerical equation, controller parameter, saturation, or integration rule was
changed.

The active backend contract is `ROUTE_POSITION`, `QN_ORIGINAL_POSITION`, and
`QN_ORIGINAL_RBF_PD`. The historical `LOS_SURGE_YAW` branch is not active. The original
`models/qn/qn.slx` is retained as the source model; MATLAB is not required at runtime.

This first reproduction keeps the official demo in its AIR operating region. Swarm-Formation's
`max_vel` and `max_acc` constraints do not model hydrodynamic drag, buoyancy, actuator envelopes,
or medium transition cost, so cross-medium formation flight is not yet claimed.

## Build and run

```bash
cd ~/Swarm-Formation
./scripts/docker_build_qn.sh
./scripts/docker_test_qn_single.sh
./scripts/docker_run_qn_demo.sh
```

After RViz opens and the seven qn nodes report ready, use `2D Nav Goal` to select the formation
destination, exactly as in the upstream demo.

The upstream `normal_hexagon.launch` actually starts IDs 0 through 6: one center member and six
hexagon members, so the current demo contains seven qn-controlled AAVs.

## Verified boundary

- The unmodified upstream image builds all 18 official catkin packages.
- The qn image builds 19 packages, adding only `qn_aav_simulator`.
- A single qn AAV given an x reference of 0.2 m reached x=0.177 m after four seconds. The state
  was dynamically integrated rather than copied from PositionCommand.
- The official normal-hexagon launch started all seven planner, trajectory, sensing, visualization,
  and qn nodes. After a goal was published, drone 0 moved from x=-26.0 m to x=5.06 m in 20 seconds.
- In a complete run to a clicked center target of `(20, 0, 0.5)` m, all seven qn Odometry states
  reached their assigned center-plus-hexagon slots. The maximum final position error was about
  0.00062 m.

This validates the software, nominal AIR execution chain, and final formation position. It does not
yet validate minimum obstacle clearance under qn tracking error, actuator saturation margins, or
cross-medium formation flight.
