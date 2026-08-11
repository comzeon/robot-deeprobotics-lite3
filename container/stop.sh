#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop and remove the robonix_lite3_ros container (and the compose project).
# Safe to run when already down.
set -euo pipefail
export ROBONIX_LITE3_CONTAINER="${ROBONIX_LITE3_CONTAINER:-robonix_lite3_ros}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-robonix_lite3_ros}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
docker compose -f compose.yaml down
echo "[container/stop] $ROBONIX_LITE3_CONTAINER down."