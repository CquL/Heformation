#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-single_cancel}"
HOLD_TIMEOUT=180
case "$MODE" in group_rpc_hang|state_missing) HOLD_TIMEOUT=15 ;; esac
OUT="${2:-$ROOT/experiments/$(date -u +%Y%m%dT%H%M%SZ)-safety-$MODE}"
mkdir -p "$OUT"
OUT="$(realpath "$OUT")"
[[ ! -e "$OUT/launch.log" ]] || { echo "Use a new experiment directory" >&2; exit 2; }
STAGE="$(mktemp -d /dev/shm/safety-hold.XXXXXX)"
GUI_ARGS=()
if [[ "${VISUALIZE:-false}" == true ]]; then
  GUI_ARGS=(--env DISPLAY --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1
    --env DISABLE_ROS1_EOL_WARNINGS=1 --env HOME=/tmp --user "$(id -u):$(id -g)"
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi
cleanup() { cp -a "$STAGE/." "$OUT/"; rm -rf "$STAGE"; }
trap cleanup EXIT
docker image inspect swarm-formation-qn:safety-hold --format '{{.Id}}' > "$OUT/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$OUT/base-commit.txt"
git -C "$ROOT" diff -- integration docker scripts > "$OUT/source.patch"
cp "$ROOT/integration/swarm_qn_bridge/patches/plan_manage_safety_hold.patch" "$OUT/"
cp "$ROOT/integration/qn_aav_simulator/scripts/probe_safety_hold.py" "$OUT/probe-source.py"
cp "${BASH_SOURCE[0]}" "$OUT/runner-source.sh"
docker run --rm --init "${GUI_ARGS[@]}" -e VISUALIZE="${VISUALIZE:-false}" \
  -e ROS_HOME=/tmp/safety-ros -e MPLCONFIGDIR=/tmp/safety-mpl -e SAFETY_PROBE_MODE="$MODE" -e HOLD_TIMEOUT="$HOLD_TIMEOUT" \
  -v "$STAGE:/experiments/current" \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  swarm-formation-qn:safety-hold bash -c '
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    mkdir -p "$ROS_HOME/log"
    roslaunch qn_aav_simulator formation_aav3.launch record:=true safety_hold_timeout_s:="$HOLD_TIMEOUT" > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    cleanup() { kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; }
    trap cleanup EXIT
    for attempt in $(seq 1 60); do
      if rosparam get /formation_mission_runner/executors >/dev/null 2>&1; then break; fi
      kill -0 "$launch_pid" || exit 2
      sleep 1
    done
    if [[ "$SAFETY_PROBE_MODE" == request_* ]]; then
      rosrun qn_aav_simulator mission_dashboard.py _planning_mode:=executor _window:="$VISUALIZE" _rate:=2 > /experiments/current/dashboard.log 2>&1 &
    fi
    if [[ "$VISUALIZE" == true ]]; then
      rviz -d "$(rospack find qn_aav_simulator)/config/monitoring_request.rviz" > /experiments/current/rviz.log 2>&1 &
    fi
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_safety_hold.py \
      _mode:="$SAFETY_PROBE_MODE" > /experiments/current/probe.log 2>&1
    status=$?
    cat /experiments/current/probe.log
    exit "$status"
  '
