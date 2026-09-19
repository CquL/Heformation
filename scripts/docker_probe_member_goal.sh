#!/usr/bin/env bash
# Stage-1 isolation probe: dispatch a goal to one member and check that it lands
# there without the formation slot offset, and that the other members stay put.
#
#   ./scripts/docker_probe_member_goal.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker run --rm --init \
  --volume "$ROOT/integration/qn_aav_simulator/launch/probe_member_goal.launch:/workspace/src/src/qn_aav_simulator/launch/probe_member_goal.launch:ro" \
  --volume "$ROOT/integration/qn_aav_simulator/scripts/probe_member_goal.py:/probe_member_goal.py:ro" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator probe_member_goal.launch > /tmp/probe_launch.log 2>&1 &
    launch_pid=$!
    cleanup() { kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; }
    trap cleanup EXIT
    sleep 50
    python3 /probe_member_goal.py
  '
