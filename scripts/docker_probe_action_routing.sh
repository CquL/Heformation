#!/usr/bin/env bash
# Four independent Action checks; preserve failed runs as well as successes.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_DIR="${1:-$ROOT/experiments/$(date -u +%Y%m%dT%H%M%SZ)-action-routing}"
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"
if [[ -e "$ARTIFACT_DIR/probe.log" ]]; then
  echo "Use a new experiment directory" >&2
  exit 2
fi
docker image inspect swarm-formation-qn:noetic --format '{{.Id}}' > "$ARTIFACT_DIR/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$ARTIFACT_DIR/workspace-base-commit.txt"
git -C "$ROOT" diff -- integration docker scripts > "$ARTIFACT_DIR/workspace.patch"
STAGE_DIR="$(mktemp -d /dev/shm/formation-probe.XXXXXX)"
cleanup() { cp -a "$STAGE_DIR/." "$ARTIFACT_DIR/"; rm -rf "$STAGE_DIR"; }
trap cleanup EXIT
docker run --rm --init \
  --volume "$STAGE_DIR:/experiments/current" \
  --volume "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_aav3.launch > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    cleanup() { kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; }
    trap cleanup EXIT
    python3 - <<"PY"
import time, rospy
rospy.init_node("probe_wait_ready", anonymous=True)
nodes = ["aav_1", "aav_2", "aav_3", "aav_formation"]
end = time.monotonic() + 180
while time.monotonic() < end:
    if all(rospy.get_param("/" + n + "_action_server/ready", False) for n in nodes):
        break
    time.sleep(.5)
else:
    raise SystemExit("readiness timeout")
PY
    if rosrun qn_aav_simulator probe_action_routing.py; then probe_status=0; else probe_status=$?; fi
    grep -a "formation\] " /experiments/current/launch.log | head -8 || true
    exit "$probe_status"
  ' 2>&1 | tee "$ARTIFACT_DIR/probe.log"
