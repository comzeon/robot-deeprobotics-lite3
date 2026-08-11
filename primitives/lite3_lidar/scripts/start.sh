#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# lite3_lidar runtime — docker-exec into the pre-running robonix_lite3_ros
# container. The vendor livox_ros_driver2 node (MID-360S) runs in the same
# container/ROS domain and publishes /livox/lidar (PointCloud2); this primitive
# slices it into /scan (LaserScan). See ADR-0004.
set -euo pipefail

LITE3_CT="${ROBONIX_LITE3_CONTAINER:-robonix_lite3_ros}"

if ! docker ps --format '{{.Names}}' | grep -qx "$LITE3_CT"; then
  echo "[lite3_lidar] error: ROS container '$LITE3_CT' is not running." >&2
  echo "               Bring it up first:  bash container/start.sh" >&2
  exit 1
fi

resolve_advertise_host() {
  if [ -n "${ROBONIX_ADVERTISE_HOST:-}" ]; then
    printf '%s\n' "$ROBONIX_ADVERTISE_HOST"
    return
  fi
  local network_mode inspected
  network_mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$LITE3_CT" 2>/dev/null || true)"
  if [ "$network_mode" = "host" ]; then
    printf '%s\n' "127.0.0.1"
    return
  fi
  inspected="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$LITE3_CT" 2>/dev/null || true)"
  if [[ "$inspected" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    printf '%s\n' "$inspected"
    return
  fi
  printf '%s\n' "127.0.0.1"
}
ADVERTISE_HOST="$(resolve_advertise_host)"

exec docker exec \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e ROBONIX_ADVERTISE_HOST="$ADVERTISE_HOST" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-lite3_lidar}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e LITE3_LIDAR_CLOUD_TOPIC="${LITE3_LIDAR_CLOUD_TOPIC:-/livox/lidar}" \
  -e LITE3_LIDAR_SCAN_TOPIC="${LITE3_LIDAR_SCAN_TOPIC:-/scan}" \
  -e LITE3_LIDAR_FRAME="${LITE3_LIDAR_FRAME:-lidar_link}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/lite3_lidar/rbnx-build/codegen/proto_gen:/robonix_pkgs/primitives/lite3_lidar/rbnx-build/codegen/robonix_mcp_types" \
  "$LITE3_CT" \
  bash -lc 'set -eo pipefail
            set +u
            source /opt/ros/humble/setup.bash >/dev/null
            [ -f /livox_ws/install/setup.bash ] && source /livox_ws/install/setup.bash >/dev/null || true
            OVL=/robonix_pkgs/primitives/lite3_lidar/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" >/dev/null || true
            cd /robonix_pkgs/primitives/lite3_lidar
            LOG=/tmp/lite3_lidar_driver.log
            : > "$LOG"
            python3 -m lidar_driver.driver >>"$LOG" 2>&1 &
            DRIVER_PID=$!
            tail --pid="$DRIVER_PID" -n +1 -F "$LOG" &
            TAIL_PID=$!
            set +e
            wait "$DRIVER_PID"
            STATUS=$?
            set -e
            kill "$TAIL_PID" 2>/dev/null || true
            wait "$TAIL_PID" 2>/dev/null || true
            exit "$STATUS"
            '