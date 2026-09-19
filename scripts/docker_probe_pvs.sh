#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:?usage: docker_probe_pvs.sh output-directory}"
mkdir -p "$OUTPUT"
OUTPUT="$(realpath "$OUTPUT")"
docker run --rm --init \
  --volume "$ROOT/integration/qn_aav_simulator:/workspace/src/src/qn_aav_simulator:ro" \
  --volume "$ROOT/upstream/Fossen/src:/pvs:ro" --volume "$OUTPUT:/evidence" \
  --env OTTER_INITIALIZATION="${OTTER_INITIALIZATION:-NATIVE_ZERO}" \
  swarm-formation-qn:five-platform-dev bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    export PYTHONPATH="/pvs:$PYTHONPATH"
    export MPLBACKEND=Agg
    roscore > /evidence/roscore.log 2>&1 &
    master=$!
    trap "kill $master 2>/dev/null || true" EXIT
    for attempt in {1..60}; do
      if rosparam list > /dev/null 2>&1; then break; fi
      sleep .25
    done
    python3 /workspace/src/src/qn_aav_simulator/scripts/pvs_node.py \
      __name:=usv _agent_id:=usv _model:=otter _initial_position:="[-5.0,0.0,0.0]" \
      _initialization_mode:="$OTTER_INITIALIZATION" \
      _propulsion_effort:=20.0 > /evidence/usv.log 2>&1 &
    usv=$!
    python3 /workspace/src/src/qn_aav_simulator/scripts/pvs_node.py \
      __name:=uuv _agent_id:=uuv _model:=remus100 _initial_position:="[-5.0,0.0,-2.0]" \
      _propulsion_effort:=500.0 > /evidence/uuv.log 2>&1 &
    uuv=$!
    trap "kill $uuv $usv $master 2>/dev/null || true" EXIT
    probe_args=()
    if [[ "$OTTER_INITIALIZATION" == STATIC_TRIM ]]; then probe_args+=(--otter-trim); fi
    python3 /workspace/src/src/qn_aav_simulator/scripts/probe_pvs_actions.py --output /evidence "${probe_args[@]}"
  ' | tee "$OUTPUT/probe.log"
