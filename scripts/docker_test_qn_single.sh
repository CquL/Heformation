#!/usr/bin/env bash
set -euo pipefail

docker run --rm swarm-formation-qn:noetic bash -lc '
  source /opt/ros/noetic/setup.bash
  source /workspace/devel/setup.bash
  roscore >/tmp/roscore.log 2>&1 &
  ROSCORE_PID=$!
  trap "kill ${ROSCORE_PID} 2>/dev/null || true" EXIT
  sleep 2
  rosrun qn_aav_simulator qn_aav_node.py \
    _drone_id:=0 _init_x:=0.0 _init_y:=0.0 _init_z:=0.5 \
    __name:=qn_test \
    /qn_test/command:=/test/pos_cmd \
    /qn_test/odometry:=/test/odom >/tmp/qn.log 2>&1 &
  QN_PID=$!
  trap "kill ${QN_PID} ${ROSCORE_PID} 2>/dev/null || true" EXIT
  sleep 2
  rostopic pub -r 20 /test/pos_cmd quadrotor_msgs/PositionCommand \
    "{position: {x: 0.2, y: 0.0, z: 0.5}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}" \
    >/tmp/pub.log 2>&1 &
  PUB_PID=$!
  sleep 4
  X=$(timeout 10 rostopic echo -n 1 /test/odom/pose/pose/position/x | head -n 1)
  kill ${PUB_PID} 2>/dev/null || true
  python3 -c "import sys; x=float(sys.argv[1]); assert x > 0.05, x; print(\"PASS qn x=\", x)" "$X"
'
