#!/usr/bin/env bash
# Render the whole-mission figure and the animated replay for one experiment.
#
#   ./scripts/docker_render_figures.sh experiments/20260918-m2-baseline-shm [fps]
#
# Writes mission_overview.png and mission_replay.gif next to the recording.
# Read-only with respect to the simulation: only the bag, metrics.json and
# config.json are read, so this can be re-run at any time.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXPERIMENT_DIR="${1:?usage: docker_render_figures.sh <experiment-dir> [fps]}"
FPS="${2:-8}"
if [[ ! -f "$EXPERIMENT_DIR/execution.bag" ]]; then
  echo "no execution.bag under $EXPERIMENT_DIR" >&2
  exit 2
fi
EXPERIMENT_DIR="$(cd "$EXPERIMENT_DIR" && pwd)"
TITLE="$(basename "$EXPERIMENT_DIR")"

docker run --rm --user "$(id -u):$(id -g)" --env HOME=/tmp \
  --volume "$EXPERIMENT_DIR:/experiments/run" \
  swarm-formation-qn:noetic bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /workspace/devel/setup.bash
    cd /tmp
    # Render beside the target and rename into place: the rename only needs write
    # permission on the directory, so this also replaces a file left behind by an
    # earlier run under a different user.
    rosrun qn_aav_simulator plot_mission_overview.py /experiments/run \
      --title "$1" --output /experiments/run/.mission_overview.tmp.png
    mv /experiments/run/.mission_overview.tmp.png /experiments/run/mission_overview.png
    rosrun qn_aav_simulator animate_mission_replay.py /experiments/run \
      --title "$1" --fps "$2" --output /experiments/run/.mission_replay.tmp.gif
    mv /experiments/run/.mission_replay.tmp.gif /experiments/run/mission_replay.gif
  ' bash "$TITLE" "$FPS"

printf 'Figures written under %s\n' "$EXPERIMENT_DIR"
