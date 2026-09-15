#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM="${ROOT}/upstream/Swarm-Formation"

docker build -t swarm-formation-upstream:noetic "${UPSTREAM}"
docker build -f "${ROOT}/docker/Dockerfile.qn" -t swarm-formation-qn:noetic "${ROOT}"
