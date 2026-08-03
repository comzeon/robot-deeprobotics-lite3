#!/usr/bin/env bash
# Lite3 chassis primitive — robonix-api managed lifecycle.
# The driver reads its network config from the manifest `config:` block (passed
# as cfg to on_init); the CLI args below are a fall-through for standalone runs.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCRIPT_DIR"

# robonix-api + codegen Python path. `rbnx setup` exports ROBONIX_SOURCE_PATH;
# fall back to ~/Desktop/robonix only on the canonical dev host and fail clearly
# otherwise so a stale env does not silently break imports.
ROBONIX_SRC="${ROBONIX_SOURCE_PATH:-$HOME/Desktop/robonix}"
if [ ! -d "${ROBONIX_SRC}/pylib/robonix-api" ]; then
  echo "[lite3_chassis] ROBONIX_SOURCE_PATH='${ROBONIX_SRC}' has no pylib/robonix-api." >&2
  echo "[lite3_chassis] Run 'rbnx setup' or export ROBONIX_SOURCE_PATH=/path/to/robonix" >&2
  exit 1
fi
export PYTHONPATH="${ROBONIX_SRC}/pylib/robonix-api:${PYTHONPATH:-}"

# add codegen proto path if the package has been built
CODGEN="$SCRIPT_DIR/rbnx-build/codegen/proto_gen"
[ -d "$CODGEN" ] && export PYTHONPATH="$CODGEN:$PYTHONPATH"

exec python3 -m lite3_driver.driver