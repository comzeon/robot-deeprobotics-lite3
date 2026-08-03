#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# lite3_chassis runtime — docker-exec into the pre-running robonix_lite3_ros
# container. The container is brought up first by `bash container/start.sh`; it
# holds ROS 2 (rclpy) + the zenoh router + robot_state_publisher. See ADR-0004.
#
# Lifecycle is owned by rbnx/Soma (Driver CMD_INIT/SHUTDOWN + manifest stop).
# Keep start.sh a foreground docker-exec wrapper so provider stdout/stderr stay
# connected and failures surface in `rbnx logs`.
set -euo pipefail

# ROS container name — overridable for an isolated/parallel deploy.
LITE3_CT="${ROBONIX_LITE3_CONTAINER:-robonix_lite3_ros}"

if ! docker ps --format '{{.Names}}' | grep -qx "$LITE3_CT"; then
  echo "[lite3_chassis] error: ROS container '$LITE3_CT' is not running." >&2
  echo "                  Bring it up first:  bash container/start.sh" >&2
  exit 1
fi

# Cross-host wiring: the driver runs INSIDE the container but registers with
# the host's atlas; when the executor dials the driver's gRPC/MCP endpoint back,
# it must use an address the host can reach. Host-network containers have no
# bridge IP, so fall back to localhost unless a valid inspected IPv4 is present.
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
  -e RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-lite3_chassis}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e LITE3_ROBOT_IP="${LITE3_ROBOT_IP:-192.168.1.1}" \
  -e LITE3_BASE_FRAME="${LITE3_BASE_FRAME:-base_link}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/lite3_chassis/rbnx-build/codegen/proto_gen" \
  "$LITE3_CT" \
  bash -lc 'set -eo pipefail
            set +u
            source /opt/ros/humble/setup.bash >/dev/null
            OVL=/robonix_pkgs/primitives/lite3_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash
            [ -f "$OVL" ] && source "$OVL" >/dev/null || true
            cd /robonix_pkgs/primitives/lite3_chassis
            LOG=/tmp/lite3_chassis_driver.log
            : > "$LOG"
            python3 -m lite3_driver.driver >>"$LOG" 2>&1 &
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