#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-manual}"
case "$MODE" in manual|mission) ;; *) echo "mode must be manual or mission" >&2; exit 2 ;; esac
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is empty; run this script from an Ubuntu desktop terminal." >&2
  exit 1
fi
xhost +local:docker >/dev/null
trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT

if [[ "$MODE" == mission ]]; then
  ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  ARTIFACT_DIR="${ROOT}/experiments/$(date -u +%Y%m%dT%H%M%SZ)-rviz-mission"
  mkdir -p "$ARTIFACT_DIR"
  MOUNT_ARGS=(--volume "$ARTIFACT_DIR:/experiments/current")
else
  MOUNT_ARGS=()
fi

docker run --rm --init -it \
  --env DISPLAY="${DISPLAY}" --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw "${MOUNT_ARGS[@]}" \
  swarm-formation-qn:noetic bash -c '
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    # The master belongs to this shell, not to a launch file.  Starting
    # rviz.launch first made that launch own the master, and its rviz node is
    # required="true": closing the RViz window ended the launch, took the
    # master with it, and left the simulation nodes spamming
    # "XmlRpcClient::writeRequest: Connection refused" until they died.
    roscore > /tmp/roscore.log 2>&1 &
    core_pid=$!
    finish() {
      kill -INT "$core_pid" 2>/dev/null || true
      kill -INT "$rviz_pid" 2>/dev/null || true
    }
    trap finish EXIT
    sleep 3
    # RViz runs beside the simulation: closing its window no longer ends the run.
    roslaunch ego_planner rviz.launch &
    rviz_pid=$!
    if [ "$1" = mission ]; then
      roslaunch qn_aav_simulator formation_air.launch run_mission:=true
    else
      roslaunch ego_planner normal_hexagon.launch
    fi
  ' bash "$MODE"
