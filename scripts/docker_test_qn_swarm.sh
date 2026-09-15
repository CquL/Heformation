#!/usr/bin/env bash
set -euo pipefail

docker run --rm swarm-formation-qn:noetic bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/devel/setup.bash
  roslaunch ego_planner normal_hexagon.launch >/tmp/swarm.log 2>&1 &
  LAUNCH_PID=$!
  trap "kill -INT ${LAUNCH_PID} 2>/dev/null || true; wait ${LAUNCH_PID} 2>/dev/null || true" EXIT
  sleep 12
  QN_COUNT=$(rosnode list | grep -c "_qn_aav$")
  test "$QN_COUNT" -eq 7
  START_X=$(timeout 10 rostopic echo -n 1 /drone_0_visual_slam/odom/pose/pose/position/x | head -n 1)
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: 20.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}" \
    >/tmp/goal.log 2>&1
  timeout 15 rostopic echo -n 1 /drone_0_planning/pos_cmd >/tmp/position_command.txt
  sleep 20
  END_X=$(timeout 10 rostopic echo -n 1 /drone_0_visual_slam/odom/pose/pose/position/x | head -n 1)
  python3 -c "import sys; a=float(sys.argv[1]); b=float(sys.argv[2]); assert abs(b-a)>0.05, (a,b); print(\"PASS qn swarm nodes=7 x_start=\",a,\"x_end=\",b)" "$START_X" "$END_X"
'
