#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMATION_IMAGE="${FORMATION_IMAGE:-swarm-formation-qn:noetic}"
MODE="${1:-single}"
PLANNER_SPEED="${2:-1.5}"
ARTIFACT_DIR="${3:-${ROOT}/experiments/$(date -u +%Y%m%dT%H%M%SZ)-${MODE}-${PLANNER_SPEED}}"
REPAIR_MODE="${4:-on}"
OBSTACLE="${5:-off}"
FAULT="${6:-none}"
case "$FAULT" in none|local_scan) ;; *) echo "fault must be none or local_scan" >&2; exit 2 ;; esac
case "$MODE" in single|mission) ;; *) echo "mode must be single or mission" >&2; exit 2 ;; esac
case "$REPAIR_MODE" in on|off) ;; *) echo "repair_mode must be on or off" >&2; exit 2 ;; esac
case "$OBSTACLE" in on|off) ;; *) echo "obstacle must be on or off" >&2; exit 2 ;; esac
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"
if [[ -e "$ARTIFACT_DIR/metrics.json" || -e "$ARTIFACT_DIR/execution.bag" ]]; then
  echo "Choose an empty experiment directory: $ARTIFACT_DIR" >&2
  exit 2
fi

docker image inspect "$FORMATION_IMAGE" --format '{{.Id}}' > "$ARTIFACT_DIR/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$ARTIFACT_DIR/workspace-base-commit.txt"
git -C "$ROOT" status --short > "$ARTIFACT_DIR/workspace-status.txt"
printf '%q ' "$0" "$MODE" "$PLANNER_SPEED" "$ARTIFACT_DIR" "$REPAIR_MODE" "$OBSTACLE" "$FAULT" > "$ARTIFACT_DIR/command.txt"
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
preserve_evidence() {
  cp -a "$STAGE_DIR/." "$ARTIFACT_DIR/"
  rm -rf "$STAGE_DIR"
}
trap preserve_evidence EXIT
git -C "$ROOT" diff -- integration docker scripts > "$ARTIFACT_DIR/workspace.patch"

docker run --rm --init --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --env ROS_HOME=/experiments/current/ros \
  --env ROS_LOG_DIR=/experiments/current/ros/log \
  --volume "$STAGE_DIR:/experiments/current" \
  --volume "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  "$FORMATION_IMAGE" bash -c '
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
    if [ "$5" = local_scan ]; then
      python3 - <<"PYFAULT" > /experiments/current/injection.log 2>&1 &
import json, time, rospy, rosnode
from pathlib import Path
from actionlib_msgs.msg import GoalStatusArray
rospy.init_node("local_scan_failure_probe", anonymous=True)
end = time.monotonic() + 180
while time.monotonic() < end:
    status = rospy.wait_for_message("/formation_action/status", GoalStatusArray, timeout=10)
    if any(s.status == 1 for s in status.status_list):
        break
else:
    raise SystemExit("no active Action; fault was not injected")
rospy.sleep(.5)
stamp = rospy.Time.now().to_sec()
node = "/drone_0_pcl_render_node"
success, failure = rosnode.kill_nodes([node])
Path("/experiments/current/scan-injection.json").write_text(json.dumps({
    "node": node, "at_ros_s": stamp, "killed": success, "failed": failure}, indent=2))
if failure or node not in success:
    raise SystemExit("local scan injection failed")
PYFAULT
    fi
    set +e
    timeout --signal=INT 700 rosrun qn_aav_simulator formation_mission_runner.py \
      _planning_mode:=fixed_coalition \
      > /experiments/current/mission.log 2>&1
    result=$?
    set -e
    cleanup
    trap - EXIT
    cat /experiments/current/mission.log
    if [ "$result" -ne 0 ]; then
      tail -n 60 /experiments/current/launch.log
    fi
    set +e
    rosrun qn_aav_simulator verify_formation_experiment.py /experiments/current
    verification=$?
    set -e
    if [ "$result" -ne 0 ]; then exit "$result"; fi
    exit "$verification"
  ' bash "$MODE" "$PLANNER_SPEED" "$REPAIR_MODE" "$([ "$OBSTACLE" = on ] && echo true || echo false)" "$FAULT"

printf 'Experiment saved: %s\n' "$ARTIFACT_DIR"
