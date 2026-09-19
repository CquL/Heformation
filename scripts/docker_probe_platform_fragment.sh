#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_platform_fragment.sh output-directory [--cancel]}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
docker run --rm --init \
  --volume "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$OUTPUT:/evidence" \
  swarm-formation-qn:five-platform-dev bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    roscore > /evidence/roscore.log 2>&1 &
    master=$!
    trap "kill $master 2>/dev/null || true" EXIT
    for attempt in {1..60}; do
      if rosparam list > /dev/null 2>&1; then break; fi
      sleep .25
    done
    python3 /workspace/src/src/qn_aav_simulator/scripts/qn_aav_node.py \
      __name:=qualification_qn _init_z:=0.5 _enable_platform_action:=true \
      _qualification_only:=true _water_guidance_mode:=LOS_VELOCITY_REFERENCE \
      _water_horizontal_controller_mode:=LOS_SURGE_YAW > /evidence/qn.log 2>&1 &
    qn=$!
    trap "kill $qn $master 2>/dev/null || true" EXIT
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_platform_fragment.py --output /evidence "$@"
  ' bash "${@:2}" | tee "$OUTPUT/probe.log"
