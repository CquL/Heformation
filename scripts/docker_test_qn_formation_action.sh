#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-single}"
PLANNER_SPEED="${2:-1.5}"
ARTIFACT_DIR="${3:-${ROOT}/experiments/$(date -u +%Y%m%dT%H%M%SZ)-${MODE}-${PLANNER_SPEED}}"
REPAIR_MODE="${4:-on}"
OBSTACLE="${5:-off}"
case "$MODE" in single|mission) ;; *) echo "mode must be single or mission" >&2; exit 2 ;; esac
case "$REPAIR_MODE" in on|off) ;; *) echo "repair_mode must be on or off" >&2; exit 2 ;; esac
case "$OBSTACLE" in on|off) ;; *) echo "obstacle must be on or off" >&2; exit 2 ;; esac
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"
if [[ -e "$ARTIFACT_DIR/metrics.json" || -e "$ARTIFACT_DIR/execution.bag" ]]; then
  echo "Choose an empty experiment directory: $ARTIFACT_DIR" >&2
  exit 2
fi

docker image inspect swarm-formation-qn:noetic --format '{{.Id}}' > "$ARTIFACT_DIR/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$ARTIFACT_DIR/workspace-base-commit.txt"
git -C "$ROOT" status --short > "$ARTIFACT_DIR/workspace-status.txt"
printf '%q ' "$0" "$MODE" "$PLANNER_SPEED" "$ARTIFACT_DIR" "$REPAIR_MODE" "$OBSTACLE" > "$ARTIFACT_DIR/command.txt"
printf '\n' >> "$ARTIFACT_DIR/command.txt"

# Record onto a memory-backed staging directory and copy the result into the
# artefact directory afterwards.  Writing the bag straight to the home
# filesystem stalls the whole container for 16-32 ms every so often, which the
# fixed-step model clock turns into permanent drift: measured on the same
# mission, 57 ms of accumulated model/ROS drift when recording to disk versus
# 0.6 ms when recording to tmpfs, against a 50 ms gate.  The staging directory
# is not evidence, so it is emptied at the end of the run.
STAGE_DIR="$(mktemp -d /dev/shm/formation-record.XXXXXX 2>/dev/null || mktemp -d)"
echo "recording through $STAGE_DIR" >&2

docker run --rm --init --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --env ROS_HOME=/experiments/current/ros \
  --env ROS_LOG_DIR=/experiments/current/ros/log \
  --volume "$STAGE_DIR:/experiments/current" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_air.launch mode:="$1" planner_speed:="$2" \
      repair_mode:="$3" \
      obstacle:="$4" \
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
  ' bash "$MODE" "$PLANNER_SPEED" "$REPAIR_MODE" "$([ "$OBSTACLE" = on ] && echo true || echo false)"

# The staged files belong in the artefact directory; the staging directory only
# kept the recording off the disk while the simulation was running.
mv "$STAGE_DIR"/* "$ARTIFACT_DIR"/ 2>/dev/null || true
rmdir "$STAGE_DIR" 2>/dev/null || true
printf 'Experiment saved: %s\n' "$ARTIFACT_DIR"
