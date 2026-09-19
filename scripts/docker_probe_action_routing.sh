#!/usr/bin/env bash
# Does the selected executor control exactly its own members, through the real
# Action interface?
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm --init \
  --volume "$ROOT/integration/qn_aav_simulator/launch/formation_aav3.launch:/workspace/src/src/qn_aav_simulator/launch/formation_aav3.launch:ro" \
  --volume "$ROOT/integration/qn_aav_simulator/config/formation_aav3.yaml:/workspace/src/src/qn_aav_simulator/config/formation_aav3.yaml:ro" \
  --volume "$ROOT/integration/qn_aav_simulator/scripts/probe_action_routing.py:/probe.py:ro" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_aav3.launch > /tmp/probe_action.log 2>&1 &
    launch_pid=$!
    cleanup() { kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; }
    trap cleanup EXIT
    sleep 110
    python3 /probe.py
  '
