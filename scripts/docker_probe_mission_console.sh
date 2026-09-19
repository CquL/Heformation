#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_mission_console.sh output-directory}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
docker run --rm --init -e QT_QPA_PLATFORM=offscreen \
  -v /usr/share/fonts/opentype/noto:/usr/share/fonts/opentype/noto:ro \
  -v "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  -v "$ROOT/integration/mrta_python:/workspace/integration/mrta_python:ro" \
  -v "$OUTPUT:/experiments/current" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roslaunch qn_aav_simulator formation_aav3.launch record:=false > /experiments/current/launch.log 2>&1 &
    launch_pid=$!
    trap "kill -INT $launch_pid 2>/dev/null || true; wait $launch_pid || true" EXIT
    for attempt in {1..120}; do
      if rosparam get /formation_mission_runner/executors > /dev/null 2>&1; then break; fi
      sleep .25
    done
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_mission_console.py \
      --request /workspace/src/src/qn_aav_simulator/config/monitoring_request_coastal.yaml \
      --output /experiments/current
  ' | tee "$OUTPUT/ui-probe.log"
