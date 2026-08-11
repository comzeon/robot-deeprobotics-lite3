#!/bin/bash
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
echo "[lite3_camera/build] rbnx codegen -p $PKG --mcp"
rbnx codegen -p "$PKG" --mcp
echo "[lite3_camera/build] done."
