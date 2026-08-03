#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"
ROBONIX_SRC="${ROBONIX_SOURCE_PATH:-$HOME/Desktop/robonix}"
export PYTHONPATH="$SCRIPT_DIR:$SCRIPT_DIR/rbnx-build/codegen/proto_gen:$ROBONIX_SRC/pylib/robonix-api:${PYTHONPATH:-}"
exec python3 -m camera_driver.driver
