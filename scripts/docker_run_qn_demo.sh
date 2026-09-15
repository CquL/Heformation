#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is empty; run this script from an Ubuntu desktop terminal." >&2
  exit 1
fi
xhost +local:docker >/dev/null
trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT

docker run --rm -it \
  --env DISPLAY="${DISPLAY}" \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  swarm-formation-qn:noetic \
  bash -lc '
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch ego_planner rviz.launch &
    sleep 3
    roslaunch ego_planner normal_hexagon.launch
  '
