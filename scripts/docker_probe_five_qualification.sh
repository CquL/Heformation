#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_five_qualification.sh new-output-directory}"
FIVE_SCENARIO="${2:-normal}"
FIVE_PLATFORM_IMAGE="${FIVE_PLATFORM_IMAGE:-swarm-formation-qn:cooperation}"
VISUALIZE="${VISUALIZE:-false}"
case "$VISUALIZE" in true|false) ;; *) echo 'VISUALIZE must be true or false' >&2; exit 2 ;; esac
DEFAULT_SCENE=five_scene.yaml
if [[ "$VISUALIZE" == true || "$FIVE_SCENARIO" == cooperative ]]; then DEFAULT_SCENE=five_scene_harbor.yaml; fi
FIVE_SCENE_FILE="${FIVE_SCENE_FILE:-$ROOT/integration/qn_aav_simulator/config/$DEFAULT_SCENE}"
GUI_ARGS=()
# Keep stdin for concrete-plan confirmation, including headless integration runs.
if [[ "$FIVE_SCENARIO" == cooperative ]]; then GUI_ARGS=(-i); fi
if [[ "$VISUALIZE" == true ]]; then
  : "${DISPLAY:?Run the visual experiment from a desktop terminal with DISPLAY}"
  GUI_ARGS=(-i --env DISPLAY --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1
    --user "$(id -u):$(id -g)" --env HOME=/tmp
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  CJK_FONT=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
  if [[ ! -f "$CJK_FONT" ]]; then echo '缺少中文字体：请安装 fonts-noto-cjk' >&2; exit 2; fi
  GUI_ARGS+=(--volume /usr/share/fonts/opentype/noto:/usr/share/fonts/opentype/noto:ro
    --volume "$CJK_FONT:/opt/ros/noetic/share/rviz/ogre_media/fonts/liberation-sans/HeformationCJK.ttc:ro"
    --volume "$ROOT/integration/qn_aav_simulator/config/five_view.fontdef:/opt/ros/noetic/share/rviz/ogre_media/fonts/ogre1.9.fontdef:ro")
  if [[ -t 0 ]]; then GUI_ARGS+=(-t); fi
fi
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
if [[ -e "$OUTPUT/five-integration.json" || -e "$OUTPUT/handover.bag" ]]; then
  echo 'Use a fresh experiment directory' >&2; exit 2
fi
if [[ "$VISUALIZE" == true ]]; then
  python3 "$ROOT/scripts/prepare_five_scene_assets.py" "$OUTPUT/visual-assets"
  GUI_ARGS+=(--volume "$OUTPUT/visual-assets:/experiments/assets:ro")
fi
STAGE="$(mktemp -d /dev/shm/formation-five.XXXXXX)"
cleanup() { cp -a "$STAGE/." "$OUTPUT/"; rm -rf "$STAGE"; }
trap cleanup EXIT
docker image inspect "$FIVE_PLATFORM_IMAGE" --format '{{.Id}}' > "$OUTPUT/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$OUTPUT/base-commit.txt"
cp "$FIVE_SCENE_FILE" "$OUTPUT/scene-declared.yaml"
git -C "$ROOT" diff -- integration docker scripts > "$OUTPUT/workspace.patch"
sha256sum "$ROOT/integration/qn_aav_simulator/scripts/"{probe_five_qualification,pvs_node,qn_aav_node}.py \
  "$ROOT/integration/qn_aav_simulator/scripts/"{probe_swarm_roundtrip,run_water_cooperation,mission_dashboard,scene_publisher}.py \
  "$ROOT/integration/qn_aav_simulator/config/monitoring_request_water.yaml" \
  "$ROOT/integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py" \
  "$ROOT/integration/swarm_qn_bridge/patches/traj_opt_finite_commit.patch" \
  "$ROOT/integration/qn_aav_simulator/launch/five_qualification.launch" > "$OUTPUT/source-hashes.txt"
docker run "${GUI_ARGS[@]}" --rm --init --env ROS_HOME=/tmp/five-ros --env FIVE_VISUALIZE="$VISUALIZE" \
  --env FIVE_SCENARIO="$FIVE_SCENARIO" \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  -v "$STAGE:/experiments/current" \
  -v "$(realpath "$FIVE_SCENE_FILE"):/experiments/scene.yaml:ro" \
  "$FIVE_PLATFORM_IMAGE" bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    request_file=""
    record=true
    if [[ "$FIVE_SCENARIO" == cooperative ]]; then
      record=false
      request_file=/workspace/src/src/qn_aav_simulator/config/monitoring_request_water.yaml
    fi
    roslaunch qn_aav_simulator five_qualification.launch record:="$record" request_file:="$request_file" scene_file:=/experiments/scene.yaml visualize:="$FIVE_VISUALIZE" > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    trap "kill -INT $launch_pid 2>/dev/null || true; wait $launch_pid || true" EXIT
    for attempt in {1..120}; do
      if rosparam get /uuv/agent_id >/dev/null 2>&1; then break; fi
      sleep .25
    done
    rosparam get /scene > /experiments/current/scene-resolved.yaml
    if [[ "$FIVE_VISUALIZE" == true ]]; then
      rviz -d /workspace/src/src/qn_aav_simulator/config/five_qualification.rviz > /experiments/current/rviz.log 2>&1 &
      dashboard_mode=five_qualification
      if [[ "$FIVE_SCENARIO" == cooperative ]]; then dashboard_mode=water_cooperation; fi
      python3 /workspace/src/src/qn_aav_simulator/scripts/mission_dashboard.py _planning_mode:="$dashboard_mode" _window:=true _rate:=2 > /experiments/current/dashboard.log 2>&1 &
      if [[ "$FIVE_SCENARIO" != cooperative ]]; then
      echo "五平台实时资格实验：三 AAV + Otter + REMUS。"
      echo "场景包含实体码头、岩石、禁入区、水面和海底。"
      echo "将执行预置 AIR 动作、单 AAV 跨介质往返、USV/UUV 路径与终端行为。"
      echo "这是固定开发动作；尚不包含业务观测、有限交付和复查。"
      echo "确认前不发送实验 Goal；输入 yes 开始，其他输入退出。"
      read -r answer
      if [[ "$answer" != yes ]]; then exit 2; fi
      fi
    fi
    set +e
    if [[ "$FIVE_SCENARIO" == cooperative ]]; then
      python3 -u /workspace/src/src/qn_aav_simulator/scripts/run_water_cooperation.py
    elif [[ "$FIVE_SCENARIO" == prepared ]]; then
      python3 /workspace/src/src/qn_aav_simulator/scripts/probe_prepared_fragments.py --output /experiments/current
    else
      python3 /workspace/src/src/qn_aav_simulator/scripts/probe_five_qualification.py --output /experiments/current --scenario "$FIVE_SCENARIO"
    fi
    result=$?
    if [[ "$FIVE_VISUALIZE" == true ]]; then
      echo "实验观察结束，退出码 $result。画面继续实时更新；在此终端按 Ctrl+C 结束整链。"
      wait "$launch_pid"
    fi
    exit "$result"
  ' | tee "$OUTPUT/probe.log"
