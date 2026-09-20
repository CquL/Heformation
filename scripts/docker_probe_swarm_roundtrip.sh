#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_swarm_roundtrip.sh output-directory [scenario]}"
SCENARIO="${2:-normal}"
HANDOVER_IMAGE="${HANDOVER_IMAGE:-swarm-formation-qn:five-finite-wire}"
TRANSITION_FAULT_BEHAVIOR="${TRANSITION_FAULT_BEHAVIOR:-FIXED_REFERENCE}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
if [[ -e "$OUTPUT/roundtrip.json" || -e "$OUTPUT/handover.bag" ]]; then
  echo 'Use a fresh experiment directory' >&2; exit 2
fi
STAGE="$(mktemp -d /dev/shm/formation-handover.XXXXXX)"
cleanup() { cp -a "$STAGE/." "$OUTPUT/"; rm -rf "$STAGE"; }
trap cleanup EXIT
docker image inspect "$HANDOVER_IMAGE" --format '{{.Id}}' > "$OUTPUT/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$OUTPUT/base-commit.txt"
sha256sum "$ROOT/integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py" \
  "$ROOT/integration/qn_aav_simulator/scripts/qn_aav_node.py" \
  "$ROOT/integration/qn_aav_simulator/scripts/formation_action_server.py" \
  "$ROOT/integration/qn_aav_simulator/scripts/probe_swarm_roundtrip.py" \
  "$ROOT/integration/qn_aav_simulator/src/qn_aav_simulator/qn_python_backend.py" \
  "$ROOT/integration/qn_aav_simulator/launch/swarm_roundtrip.launch" \
  "$ROOT/integration/swarm_qn_bridge/patches/plan_manage_reference_handover.patch" \
  "$ROOT/integration/swarm_qn_bridge/patches/root_finder_degenerate_bound.patch" \
  "$ROOT/integration/swarm_qn_bridge/patches/traj_opt_finite_commit.patch" > "$OUTPUT/source-hashes.txt"
docker run --rm --init --env ROS_HOME=/tmp/handover-ros \
  --env TRANSITION_FAULT_BEHAVIOR="$TRANSITION_FAULT_BEHAVIOR" \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  -v "$STAGE:/experiments/current" \
  "$HANDOVER_IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator swarm_roundtrip.launch transition_fault_behavior:="$TRANSITION_FAULT_BEHAVIOR" > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    trap "kill -INT $launch_pid 2>/dev/null || true; wait $launch_pid || true" EXIT
    for attempt in {1..120}; do
      if rosparam get /drone_0_qn_aav/reference_handover_enabled >/dev/null 2>&1; then break; fi
      sleep .25
    done
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_swarm_roundtrip.py \
      --output /experiments/current --scenario "$1"
  ' bash "$SCENARIO" | tee "$OUTPUT/probe.log"
