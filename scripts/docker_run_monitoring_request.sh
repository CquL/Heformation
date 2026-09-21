#!/usr/bin/env bash
# Start the existing three-AAV stack in standby, then load/preview/confirm once.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FORMATION_IMAGE="${FORMATION_IMAGE:-swarm-formation-qn:noetic}"
REQUEST="${1:-$ROOT/integration/qn_aav_simulator/config/monitoring_request_coastal.yaml}"
REQUEST="$(realpath "$REQUEST")"
ARTIFACT_DIR="${2:-$ROOT/experiments/$(date -u +%Y%m%dT%H%M%SZ)-monitoring-request}"
mkdir -p "$ARTIFACT_DIR"
ARTIFACT_DIR="$(cd "$ARTIFACT_DIR" && pwd)"
PLANNER_SPEED="${PLANNER_SPEED:-1.5}"
python3 - "$PLANNER_SPEED" <<'PY_SPEED'
import math,sys
value=float(sys.argv[1])
if not math.isfinite(value) or value<=0:raise SystemExit('PLANNER_SPEED must be finite and positive')
PY_SPEED
if [[ -e "$ARTIFACT_DIR/metrics.json" ]]; then
  echo "Use a new experiment directory" >&2
  exit 2
fi
cp "$REQUEST" "$ARTIFACT_DIR/request.yaml"
printf '%s\n' "$PLANNER_SPEED" > "$ARTIFACT_DIR/planner-speed.txt"
docker image inspect "$FORMATION_IMAGE" --format '{{.Id}}' > "$ARTIFACT_DIR/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$ARTIFACT_DIR/workspace-base-commit.txt"
git -C "$ROOT" diff -- integration docker scripts > "$ARTIFACT_DIR/workspace.patch"
STAGE_DIR="$(mktemp -d /dev/shm/formation-request.XXXXXX)"
cleanup() { cp -a "$STAGE_DIR/." "$ARTIFACT_DIR/"; rm -rf "$STAGE_DIR"; }
trap cleanup EXIT
VISUALIZE="${VISUALIZE:-${DISPLAY:+true}}"
VISUALIZE="${VISUALIZE:-false}"
case "$VISUALIZE" in true|false) ;; *) echo "VISUALIZE must be true or false" >&2; exit 2 ;; esac
TASK_UI="${TASK_UI:-false}"
case "$TASK_UI" in true|false) ;; *) echo "TASK_UI must be true or false" >&2; exit 2 ;; esac
if [[ "$TASK_UI" == true && "$VISUALIZE" != true ]]; then
  echo "TASK_UI=true requires VISUALIZE=true and DISPLAY" >&2; exit 2
fi
GUI_ARGS=()
if [[ "$VISUALIZE" == true ]]; then
  : "${DISPLAY:?VISUALIZE=true requires a desktop DISPLAY}"
  GUI_ARGS=(--env DISPLAY --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1
    --env DISABLE_ROS1_EOL_WARNINGS=1 --user "$(id -u):$(id -g)" --env HOME=/tmp
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  if [[ -d /usr/share/fonts/opentype/noto ]]; then
    GUI_ARGS+=(--volume /usr/share/fonts/opentype/noto:/usr/share/fonts/opentype/noto:ro)
  fi
fi
TTY_ARGS=(-i)
EXECUTOR_SERIAL="${EXECUTOR_SERIAL:-true}"
case "$EXECUTOR_SERIAL" in true|false) ;; *) echo "EXECUTOR_SERIAL must be true or false" >&2; exit 2 ;; esac
if [[ -t 0 ]]; then TTY_ARGS+=(-t); fi
docker run "${TTY_ARGS[@]}" "${GUI_ARGS[@]}" --env PLANNER_SPEED="$PLANNER_SPEED" --env REQUEST_TASK_UI="$TASK_UI" --env EXECUTOR_SERIAL="$EXECUTOR_SERIAL" --env REQUEST_VISUALIZE="$VISUALIZE" --env ROS_HOME=/tmp/request-ros --rm --init \
  --volume "$STAGE_DIR:/experiments/current" \
  --volume "$REQUEST:/request.yaml:ro" \
  --volume "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  "$FORMATION_IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    mkdir -p "$ROS_HOME/log"
    roslaunch qn_aav_simulator formation_aav3.launch record:=true planner_speed:="$PLANNER_SPEED" > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    cleanup() {
      kill -INT "$launch_pid" 2>/dev/null || true
      wait "$launch_pid" || true
    }
    trap cleanup EXIT
    # rosparam/rosrun cannot race the launch which declares the routing table.
    python3 - <<"PY"
import time, rospy, os, math
rospy.init_node("wait_executor_configuration", anonymous=True)
end = time.monotonic() + 120
while not rospy.has_param("/formation_mission_runner/executors"):
    if time.monotonic() > end:
        raise SystemExit("executor configuration unavailable")
    time.sleep(.2)
expected=float(os.environ["PLANNER_SPEED"])
for member in range(3):
    for component in ("manager", "optimization"):
        name="/drone_{}_ego_planner_node/{}/max_vel".format(member,component)
        actual=float(rospy.get_param(name))
        if not math.isclose(actual,expected,rel_tol=0,abs_tol=1e-9):
            raise SystemExit("planner speed not applied: {}={} expected {}".format(name,actual,expected))
PY
    view_pid=""
    rosparam set /formation_mission_runner/nominal_speed_mps "$PLANNER_SPEED"
    if [[ "$REQUEST_VISUALIZE" == true ]]; then
      rviz -d "$(rospack find qn_aav_simulator)/config/monitoring_request.rviz" \
        > /experiments/current/rviz.log 2>&1 &
      view_pid=$!
    fi
    rosrun qn_aav_simulator mission_dashboard.py _planning_mode:=executor \
      _window:="$REQUEST_VISUALIZE" _rate:=2 \
      > /experiments/current/dashboard.log 2>&1 &
    set +e
    if [[ "$REQUEST_TASK_UI" == true ]]; then
      python3 /workspace/src/src/qn_aav_simulator/scripts/mission_console.py \
        --request /request.yaml --output /experiments/current --serial "$EXECUTOR_SERIAL"
      echo "Task UI closed. Simulation and any confirmed runner continue; Ctrl+C ends this simulation."
      wait "$launch_pid"
      exit $?
    fi
    rosrun qn_aav_simulator formation_mission_runner.py \
      _planning_mode:=executor _request_file:=/request.yaml \
      _executor_serial:="$EXECUTOR_SERIAL" \
      _output_dir:=/experiments/current 2>&1 | tee /experiments/current/runner.log
    mission_rc=${PIPESTATUS[0]}
    rosnode kill /formation_recorder > /experiments/current/recorder-stop.log 2>&1 || true
    if [[ -n "$view_pid" ]]; then
      echo "Request finished (exit $mission_rc). Live final state remains visible; close RViz to stop the simulation."
      wait "$view_pid"
    fi
    exit "$mission_rc"
  '
