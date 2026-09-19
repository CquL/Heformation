#!/usr/bin/env bash
# Stage-1b probe: one formation centre must reach all three AAVs, each landing at
# centre + its own slot, with one qn per member.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm --init \
  --volume "$ROOT/integration/qn_aav_simulator/launch/formation_aav3.launch:/workspace/src/src/qn_aav_simulator/launch/formation_aav3.launch:ro" \
  --volume "$ROOT/integration/qn_aav_simulator/config/formation_aav3.yaml:/workspace/src/src/qn_aav_simulator/config/formation_aav3.yaml:ro" \
  --volume "$ROOT/integration/qn_aav_simulator/scripts/probe_group_goal.py:/probe_group_goal.py:ro" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_aav3.launch > /tmp/probe3_launch.log 2>&1 &
    launch_pid=$!
    cleanup() { kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; }
    trap cleanup EXIT
    sleep 55
    python3 /probe_group_goal.py
  '
