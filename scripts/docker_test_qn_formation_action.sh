#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-single}"
PLANNER_SPEED="${2:-1.5}"
ARTIFACT_DIR="${3:-${ROOT}/experiments/$(date -u +%Y%m%dT%H%M%SZ)-${MODE}-${PLANNER_SPEED}}"
REPAIR_MODE="${4:-on}"
case "$MODE" in single|mission) ;; *) echo "mode must be single or mission" >&2; exit 2 ;; esac
case "$REPAIR_MODE" in on|off) ;; *) echo "repair_mode must be on or off" >&2; exit 2 ;; esac
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"
if [[ -e "$ARTIFACT_DIR/metrics.json" || -e "$ARTIFACT_DIR/execution.bag" ]]; then
  echo "Choose an empty experiment directory: $ARTIFACT_DIR" >&2
  exit 2
fi

docker image inspect swarm-formation-qn:noetic --format '{{.Id}}' > "$ARTIFACT_DIR/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$ARTIFACT_DIR/workspace-base-commit.txt"
git -C "$ROOT" status --short > "$ARTIFACT_DIR/workspace-status.txt"
printf '%q ' "$0" "$MODE" "$PLANNER_SPEED" "$ARTIFACT_DIR" "$REPAIR_MODE" > "$ARTIFACT_DIR/command.txt"
printf '\n' >> "$ARTIFACT_DIR/command.txt"

docker run --rm --init --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --env ROS_HOME=/experiments/current/ros \
  --env ROS_LOG_DIR=/experiments/current/ros/log \
  --volume "$ARTIFACT_DIR:/experiments/current" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_air.launch mode:="$1" planner_speed:="$2" \
      repair_mode:="$3" \
      > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    cleanup() {
      kill -INT "$launch_pid" 2>/dev/null || true
      wait "$launch_pid" || true
    }
    trap cleanup EXIT
    for attempt in $(seq 1 60); do
      if rosparam get /formation_mission_runner/centers >/dev/null 2>&1; then break; fi
      kill -0 "$launch_pid"
      sleep 1
    done
    set +e
    timeout --signal=INT 700 rosrun qn_aav_simulator formation_mission_runner.py \
      > /experiments/current/mission.log 2>&1
    result=$?
    set -e
    cleanup
    trap - EXIT
    cat /experiments/current/mission.log
    if [ "$result" -ne 0 ]; then
      tail -n 60 /experiments/current/launch.log
      exit "$result"
    fi
    rosrun qn_aav_simulator verify_formation_experiment.py /experiments/current
  ' bash "$MODE" "$PLANNER_SPEED" "$REPAIR_MODE"
printf 'Experiment saved: %s\n' "$ARTIFACT_DIR"
