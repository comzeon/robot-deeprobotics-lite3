#!/usr/bin/env bash
# Lite3 chassis primitive — robonix-api managed lifecycle
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

# robonix-api PYTHONPATH
ROBONIX_SRC="${ROBONIX_SOURCE_PATH:-$HOME/Desktop/robonix}"
export PYTHONPATH="${ROBONIX_SRC}/pylib/robonix-api:${PYTHONPATH:-}"

# Also add codegen proto path if it exists
CODGEN="$SCRIPT_DIR/rbnx-build/codegen/proto_gen"
[ -d "$CODGEN" ] && export PYTHONPATH="$CODGEN:$PYTHONPATH"

exec python3 -m lite3_driver.driver \
  --robot-ip "${ROBOT_IP:-192.168.2.1}" \
  --cmd-port "${CMD_PORT:-43893}" \
  --state-port "${STATE_PORT:-43897}"
