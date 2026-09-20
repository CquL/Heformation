#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="$ROOT/upstream/VRX"
REVISION=c9b9388308f8976c724da4af6685f69c9c378983
if [[ ! -d "$SOURCE" ]]; then
  git clone --depth 1 --branch gazebo_classic https://github.com/osrf/vrx.git "$SOURCE"
fi
if [[ "$(git -C "$SOURCE" rev-parse HEAD)" != "$REVISION" ]]; then
  echo "VRX checkout differs from tested revision $REVISION; preserving it. Inspect before building." >&2
  exit 2
fi
docker build -f "$ROOT/docker/Dockerfile.vrx" -t heformation-vrx:noetic "$SOURCE"
