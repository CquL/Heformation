#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOINT_OUTPUT="${1:?usage: docker_run_joint_request.sh new-output-directory}"
JOINT_IMAGE="${JOINT_IMAGE:-swarm-formation-qn:joint-wip}"
JOINT_VISUALIZE="${JOINT_VISUALIZE:-true}"
JOINT_GPU_RENDER="${JOINT_GPU_RENDER:-false}"
JOINT_PLANNING_BUDGET_S="${JOINT_PLANNING_BUDGET_S:-10}"
JOINT_REPAIR_BUDGET_S="${JOINT_REPAIR_BUDGET_S:-10}"
JOINT_SIM_CPUSET="${JOINT_SIM_CPUSET:-}"
JOINT_PLANNER_CPUSET="${JOINT_PLANNER_CPUSET:-}"
JOINT_VIEW_CPUSET="${JOINT_VIEW_CPUSET:-}"
JOINT_VIEW_HOLD_S="${JOINT_VIEW_HOLD_S:-5}"
JOINT_REQUEST_FILE="${JOINT_REQUEST_FILE:-/workspace/src/src/qn_aav_simulator/config/monitoring_request_offshore.yaml}"
JOINT_EXECUTORS_FILE="${JOINT_EXECUTORS_FILE:-/workspace/src/src/qn_aav_simulator/config/joint_request_executors.yaml}"
JOINT_SCENE_FILE="${JOINT_SCENE_FILE:-$PROJECT_ROOT/integration/qn_aav_simulator/config/five_scene_offshore.yaml}"
JOINT_USV_INITIAL_POSITION="${JOINT_USV_INITIAL_POSITION:-[-25.0, -2.0, 0.0]}"
QN_SAME_SOURCE_ACCELERATION="${QN_SAME_SOURCE_ACCELERATION:-false}"
JOINT_QUALIFICATION_MISSING_AIR_MEMBER="${JOINT_QUALIFICATION_MISSING_AIR_MEMBER:-}"
JOINT_VISUAL_TIMING_RELAX="${JOINT_VISUAL_TIMING_RELAX:-false}"
case "$JOINT_VISUALIZE" in true|false) ;; *) echo 'JOINT_VISUALIZE must be true or false' >&2; exit 2 ;; esac
case "$JOINT_GPU_RENDER" in true|false) ;; *) echo 'JOINT_GPU_RENDER must be true or false' >&2; exit 2 ;; esac
mkdir -p "$JOINT_OUTPUT"
JOINT_OUTPUT="$(realpath "$JOINT_OUTPUT")"
if [[ -e "$JOINT_OUTPUT/metrics.json" ]]; then
  echo "Choose a new output directory: $JOINT_OUTPUT" >&2
  exit 2
fi
docker image inspect "$JOINT_IMAGE" --format '{{.Id}}' > "$JOINT_OUTPUT/image-id.txt"
git -C "$PROJECT_ROOT" rev-parse HEAD > "$JOINT_OUTPUT/base-commit.txt"
git -C "$PROJECT_ROOT" diff -- integration scripts > "$JOINT_OUTPUT/workspace.patch"

JOINT_GUI_ARGS=()
if [[ "$JOINT_VISUALIZE" == true ]]; then
  : "${DISPLAY:?Run the visual task from a desktop terminal with DISPLAY}"
  JOINT_FONT=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
  [[ -f "$JOINT_FONT" ]] || { echo 'Install fonts-noto-cjk before opening the Chinese RViz view' >&2; exit 2; }
  python3 "$PROJECT_ROOT/scripts/prepare_five_scene_assets.py" "$JOINT_OUTPUT/visual-assets"
  JOINT_GUI_ARGS+=(
    --env DISPLAY --env HOME=/tmp --env QT_X11_NO_MITSHM=1
    --env MPLCONFIGDIR=/tmp/mpl --env XDG_CONFIG_HOME=/tmp/config
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
    --volume /usr/share/fonts/opentype/noto:/usr/share/fonts/opentype/noto:ro
    --volume "$JOINT_FONT:/opt/ros/noetic/share/rviz/ogre_media/fonts/liberation-sans/HeformationCJK.ttc:ro"
    --volume "$PROJECT_ROOT/integration/qn_aav_simulator/config/five_view.fontdef:/opt/ros/noetic/share/rviz/ogre_media/fonts/ogre1.9.fontdef:ro"
    --volume "$JOINT_OUTPUT/visual-assets:/experiments/assets:ro")
  if [[ -n "${XAUTHORITY:-}" && -r "$XAUTHORITY" ]]; then
    JOINT_GUI_ARGS+=(--env XAUTHORITY=/tmp/heformation-xauthority
      --volume "$XAUTHORITY:/tmp/heformation-xauthority:ro")
  fi
  if [[ "$JOINT_GPU_RENDER" == true ]]; then
    JOINT_GUI_ARGS+=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display,compute)
  else
    JOINT_GUI_ARGS+=(--env LIBGL_ALWAYS_SOFTWARE=1)
  fi
fi

docker run --rm --init -i --user "$(id -u):$(id -g)" \
  --env ROS_HOME=/tmp/joint-request-ros \
  --env JOINT_VISUALIZE="$JOINT_VISUALIZE" \
  --env JOINT_PLANNING_BUDGET_S="$JOINT_PLANNING_BUDGET_S" \
  --env JOINT_REPAIR_BUDGET_S="$JOINT_REPAIR_BUDGET_S" \
  --env JOINT_SIM_CPUSET="$JOINT_SIM_CPUSET" \
  --env JOINT_PLANNER_CPUSET="$JOINT_PLANNER_CPUSET" \
  --env JOINT_VIEW_CPUSET="$JOINT_VIEW_CPUSET" \
  --env JOINT_VIEW_HOLD_S="$JOINT_VIEW_HOLD_S" \
  --env JOINT_REQUEST_FILE="$JOINT_REQUEST_FILE" \
  --env JOINT_EXECUTORS_FILE="$JOINT_EXECUTORS_FILE" \
  --env JOINT_USV_INITIAL_POSITION="$JOINT_USV_INITIAL_POSITION" \
  --env QN_SAME_SOURCE_ACCELERATION="$QN_SAME_SOURCE_ACCELERATION" \
  --env JOINT_QUALIFICATION_MISSING_AIR_MEMBER="$JOINT_QUALIFICATION_MISSING_AIR_MEMBER" \
  --env JOINT_VISUAL_TIMING_RELAX="$JOINT_VISUAL_TIMING_RELAX" \
  "${JOINT_GUI_ARGS[@]}" \
  --volume "$PROJECT_ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$PROJECT_ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  --volume "$JOINT_OUTPUT:/experiments/current" \
  --volume "$JOINT_SCENE_FILE:/experiments/scene.yaml:ro" \
  "$JOINT_IMAGE" bash -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    [[ -f "$JOINT_REQUEST_FILE" && -f "$JOINT_EXECUTORS_FILE" ]] || {
      echo "Declared request/executor file missing" >&2; exit 2;
    }
    sim_prefix=()
    if [[ -n "$JOINT_SIM_CPUSET" ]]; then sim_prefix=(taskset -c "$JOINT_SIM_CPUSET"); fi
    "${sim_prefix[@]}" roslaunch qn_aav_simulator five_qualification.launch record:=false \
      request_file:="$JOINT_REQUEST_FILE" \
      scene_file:=/experiments/scene.yaml visualize:="$JOINT_VISUALIZE" \
      usv_initial_position:="$JOINT_USV_INITIAL_POSITION" \
      > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    rviz_pid="";dashboard_pid="";recorder_pid=""
    cleanup() {
      if [[ -n "$recorder_pid" ]]; then kill -INT "$recorder_pid" 2>/dev/null || true; fi
      if [[ -n "$dashboard_pid" ]]; then kill -INT "$dashboard_pid" 2>/dev/null || true; fi
      if [[ -n "$rviz_pid" ]]; then kill -INT "$rviz_pid" 2>/dev/null || true; fi
      kill -INT "$launch_pid" 2>/dev/null || true
    }
    trap cleanup EXIT
    for attempt in {1..120}; do
      if rosparam get /uuv/agent_id >/dev/null 2>&1; then break; fi
      kill -0 "$launch_pid"
      sleep .25
    done
    if [[ -n "$JOINT_QUALIFICATION_MISSING_AIR_MEMBER" ]]; then
      rosparam set /mission/qualification_missing_air_member "$JOINT_QUALIFICATION_MISSING_AIR_MEMBER"
    fi
    if [[ "$JOINT_VISUAL_TIMING_RELAX" == true ]]; then
      rosparam set /mission/allow_visual_timing_relaxation true
    fi
    rosparam load "$JOINT_EXECUTORS_FILE" /formation_mission_runner
    rosparam set /formation_mission_runner/planning_mode joint_request
    rosparam set /formation_mission_runner/executor_serial false
    rosparam set /formation_mission_runner/request_file "$JOINT_REQUEST_FILE"
    rosparam set /formation_mission_runner/output_dir /experiments/current
    rosparam set /formation_mission_runner/planning_budget_s "$JOINT_PLANNING_BUDGET_S"
    rosparam set /formation_mission_runner/repair_budget_s "$JOINT_REPAIR_BUDGET_S"
    timeout 15 rosbag record --lz4 -l 1 -O /experiments/current/scene-once.bag \
      /scene/global_cloud > /experiments/current/scene-recorder.log 2>&1
    record_cmd=(rosbag record --lz4 --buffsize=256 -O /experiments/current/execution.bag
      /aav_1/formation_action/goal /aav_1/formation_action/result
      /aav_2/formation_action/goal /aav_2/formation_action/result
      /aav_3/formation_action/goal /aav_3/formation_action/result
      /drone_0_qn_aav/platform_task/goal /drone_0_qn_aav/platform_task/result
      /drone_1_qn_aav/platform_task/goal /drone_1_qn_aav/platform_task/result
      /drone_2_qn_aav/platform_task/goal /drone_2_qn_aav/platform_task/result
      /usv/platform_task/goal /usv/platform_task/result
      /uuv/platform_task/goal /uuv/platform_task/result
      /drone_0_qn/odometry /drone_0_qn/diagnostics
      /drone_1_qn/odometry /drone_1_qn/diagnostics
      /drone_2_qn/odometry /drone_2_qn/diagnostics
      /usv/odometry /usv/diagnostics /uuv/odometry /uuv/diagnostics
      /drone_0_planning/safety_status /drone_0_planning/trajectory
      /drone_1_planning/safety_status /drone_1_planning/trajectory
      /drone_2_planning/safety_status /drone_2_planning/trajectory
      /mother/received_products /mother/received_notifications
      /mother/command_requests /mother/command_deliveries /mother/state_claim_requests
      /drone_0_qn_aav/local_products /drone_1_qn_aav/local_products
      /drone_2_qn_aav/local_products
      /usv/local_products /uuv/local_products)
    if [[ -n "$JOINT_VIEW_CPUSET" ]]; then record_cmd=(taskset -c "$JOINT_VIEW_CPUSET" "${record_cmd[@]}"); fi
    "${record_cmd[@]}" > /experiments/current/recorder.log 2>&1 &
    recorder_pid=$!
    if [[ "$JOINT_VISUALIZE" == true ]]; then
      view_prefix=()
      if [[ -n "$JOINT_VIEW_CPUSET" ]]; then view_prefix=(taskset -c "$JOINT_VIEW_CPUSET"); fi
      "${view_prefix[@]}" rviz -d /workspace/src/src/qn_aav_simulator/config/five_qualification.rviz \
        > /experiments/current/rviz.log 2>&1 &
      rviz_pid=$!
      "${view_prefix[@]}" python3 /workspace/src/src/qn_aav_simulator/scripts/mission_dashboard.py \
        _planning_mode:=water_cooperation _window:=true _rate:=2 \
        > /experiments/current/dashboard.log 2>&1 &
      dashboard_pid=$!
      sleep 2
      if ! kill -0 "$rviz_pid" 2>/dev/null || ! kill -0 "$dashboard_pid" 2>/dev/null; then
        echo "RViz or the Chinese task view exited before planning; see its log" >&2
        tail -n 20 /experiments/current/rviz.log /experiments/current/dashboard.log >&2
        exit 1
      fi
    fi
    set +e
    runner_cmd=(python3 -u /workspace/src/src/qn_aav_simulator/scripts/formation_mission_runner.py)
    if [[ -n "$JOINT_PLANNER_CPUSET" ]]; then runner_cmd=(taskset -c "$JOINT_PLANNER_CPUSET" "${runner_cmd[@]}"); fi
    "${runner_cmd[@]}" 2>&1 | tee /experiments/current/runner.log
    runner_result=${PIPESTATUS[0]}
    set -e
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" || true
    recorder_pid=""
    if [[ "$JOINT_VISUALIZE" == true ]]; then sleep "$JOINT_VIEW_HOLD_S"; fi
    exit "$runner_result"
  '
