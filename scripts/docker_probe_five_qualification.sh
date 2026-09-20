#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_five_qualification.sh new-output-directory}"
FIVE_SCENARIO="${2:-normal}"
FIVE_PLATFORM_IMAGE="${FIVE_PLATFORM_IMAGE:-swarm-formation-qn:five-finite-wire}"
FIVE_SCENE_FILE="${FIVE_SCENE_FILE:-$ROOT/integration/qn_aav_simulator/config/five_scene.yaml}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
if [[ -e "$OUTPUT/five-integration.json" || -e "$OUTPUT/handover.bag" ]]; then
  echo 'Use a fresh experiment directory' >&2; exit 2
fi
STAGE="$(mktemp -d /dev/shm/formation-five.XXXXXX)"
cleanup() { cp -a "$STAGE/." "$OUTPUT/"; rm -rf "$STAGE"; }
trap cleanup EXIT
docker image inspect "$FIVE_PLATFORM_IMAGE" --format '{{.Id}}' > "$OUTPUT/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$OUTPUT/base-commit.txt"
cp "$FIVE_SCENE_FILE" "$OUTPUT/scene-declared.yaml"
git -C "$ROOT" diff -- integration docker scripts > "$OUTPUT/workspace.patch"
sha256sum "$ROOT/integration/qn_aav_simulator/scripts/"{probe_five_qualification,pvs_node,qn_aav_node}.py \
  "$ROOT/integration/qn_aav_simulator/scripts/probe_swarm_roundtrip.py" \
  "$ROOT/integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py" \
  "$ROOT/integration/swarm_qn_bridge/patches/traj_opt_finite_commit.patch" \
  "$ROOT/integration/qn_aav_simulator/launch/five_qualification.launch" > "$OUTPUT/source-hashes.txt"
docker run --rm --init --env ROS_HOME=/tmp/five-ros \
  --env FIVE_SCENARIO="$FIVE_SCENARIO" \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  -v "$STAGE:/experiments/current" \
  -v "$(realpath "$FIVE_SCENE_FILE"):/experiments/scene.yaml:ro" \
  "$FIVE_PLATFORM_IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator five_qualification.launch scene_file:=/experiments/scene.yaml > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    trap "kill -INT $launch_pid 2>/dev/null || true; wait $launch_pid || true" EXIT
    for attempt in {1..120}; do
      if rosparam get /uuv/agent_id >/dev/null 2>&1; then break; fi
      sleep .25
    done
    rosparam get /scene > /experiments/current/scene-resolved.yaml
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_five_qualification.py --output /experiments/current --scenario "$FIVE_SCENARIO"
  ' | tee "$OUTPUT/probe.log"
