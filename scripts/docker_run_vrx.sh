#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-native}"
case "$MODE" in native|five-view) ;; *) echo 'usage: docker_run_vrx.sh [native|five-view] [new-output-directory]' >&2; exit 2 ;; esac
OUTPUT="${2:-$ROOT/experiments/$(date -u +%Y%m%dT%H%M%SZ)-vrx-$MODE}"
: "${DISPLAY:?Run from a desktop terminal}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
if [[ -e "$OUTPUT/run.log" ]]; then echo 'Use a new experiment directory' >&2; exit 2; fi
TTY=(-i);if [[ -t 0 ]]; then TTY+=(-t);fi
GPU_ARGS=()
# Some installed NVIDIA container runtimes omit the shader compiler library
# required by newer host drivers. Mount that exact host version when present.
GPU_COMPILER="$(/sbin/ldconfig -p | awk '/libnvidia-gpucomp.so/{print $NF; exit}')"
if [[ -n "$GPU_COMPILER" && -f "$GPU_COMPILER" ]]; then
  GPU_ARGS+=(-v "$GPU_COMPILER:/usr/lib/x86_64-linux-gnu/$(basename "$GPU_COMPILER"):ro")
fi
docker image inspect heformation-vrx:noetic --format '{{.Id}}' > "$OUTPUT/image-id.txt"
git -C "$ROOT" rev-parse HEAD > "$OUTPUT/base-commit.txt"
git -C "$ROOT/upstream/VRX" rev-parse HEAD > "$OUTPUT/vrx-commit.txt"
docker run "${TTY[@]}" "${GPU_ARGS[@]}" --rm --init --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display --env DISPLAY \
  --env __NV_PRIME_RENDER_OFFLOAD=1 --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --env QT_X11_NO_MITSHM=1 --env VRX_MODE="$MODE" --env ROS_HOME=/tmp/vrx-ros \
  --user "$(id -u):$(id -g)" --env HOME=/tmp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$OUTPUT:/experiments/current" \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  heformation-vrx:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    source /vrx_ws/devel/setup.bash --extend
    mkdir -p "$ROS_HOME"
    VRX_WORLD=/vrx_ws/devel/share/vrx_gazebo/worlds/example_course.world
    test -s "$VRX_WORLD" || { echo "VRX编译产物不存在：$VRX_WORLD" >&2; exit 2; }
    if [[ "$VRX_MODE" == native ]]; then
      echo "VRX 官方海面示例：原生 WAM-V，独立于 qn/PVS。Ctrl+C 结束。"
      exec roslaunch vrx_gazebo sydneyregatta.launch verbose:=true
    fi
    echo "VRX 实际状态视图：Gazebo只显示 qn/PVS，不接管其动力学或判定任务。"
    roscore > /experiments/current/roscore.log 2>&1 &
    master_pid=$!
    trap "kill -INT $master_pid 2>/dev/null || true; wait $master_pid || true" EXIT
    for attempt in {1..120}; do
      if rosparam list >/dev/null 2>&1; then break; fi
      sleep .25
    done
    rosparam set /use_sim_time false
    roslaunch qn_aav_simulator five_qualification.launch scene_file:=/workspace/src/src/qn_aav_simulator/config/five_scene.yaml > /experiments/current/platforms.log 2>&1 &
    platform_pid=$!
    trap "kill -INT $platform_pid $master_pid 2>/dev/null || true; wait $platform_pid || true; wait $master_pid || true" EXIT
    # Preserve the existing wall-time execution chain. Gazebo has its own
    # environment clock; /clock is deliberately not consumed by qn/PVS.
    roslaunch gazebo_ros empty_world.launch world_name:="$VRX_WORLD" use_sim_time:=false gui:=true verbose:=true > /experiments/current/gazebo.log 2>&1 &
    python3 /workspace/src/src/qn_aav_simulator/scripts/vrx_state_view.py _output_dir:=/experiments/current > /experiments/current/state-view.log 2>&1 &
    python3 - <<"PY"
import json,time
from pathlib import Path
import rospy
from gazebo_msgs.srv import GetWorldProperties
rospy.init_node("wait_vrx_scene",anonymous=True)
rospy.wait_for_service("/gazebo/get_world_properties",timeout=90)
query=rospy.ServiceProxy("/gazebo/get_world_properties",GetWorldProperties)
deadline=time.monotonic()+90
while time.monotonic()<deadline:
    world=query()
    expected={"sydneyregatta","ocean_waves"}|{"heformation_"+m for m in ("drone_0","drone_1","drone_2","usv","uuv")}
    if expected.issubset(set(world.model_names)):
        Path("/experiments/current/world-ready.json").write_text(json.dumps({"models":world.model_names,"success":True},indent=2))
        break
    time.sleep(.25)
else:raise SystemExit("VRX场景/五平台显示对象未就绪，不派发动作；检查gazebo.log")
PY
    echo "这是固定动作与显示试接；VRX地形尚未接入任务安全地图。输入 yes 开始五平台动作。"
    read -r answer
    if [[ "$answer" != yes ]]; then exit 2; fi
    set +e
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_five_qualification.py --output /experiments/current
    result=$?
    echo "动作探针退出码 $result。场景继续显示；Ctrl+C 结束整链。"
    wait "$platform_pid"
  ' 2>&1 | tee "$OUTPUT/run.log"
