#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

ROBONIX_SRC="${ROBONIX_SOURCE_PATH:-$HOME/Desktop/robonix}"
if [ ! -d "${ROBONIX_SRC}/pylib/robonix-api" ]; then
  echo "[lite3_camera] ROBONIX_SOURCE_PATH='${ROBONIX_SRC}' has no pylib/robonix-api." >&2
  echo "[lite3_camera] Run 'rbnx setup' or export ROBONIX_SOURCE_PATH=/path/to/robonix" >&2
  exit 1
fi
export PYTHONPATH="$SCRIPT_DIR:$SCRIPT_DIR/rbnx-build/codegen/proto_gen:${ROBONIX_SRC}/pylib/robonix-api:${PYTHONPATH:-}"
exec python3 -m camera_driver.driver