#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Bring up the robonix_lite3_ros container. Run this BEFORE `rbnx boot` from the
# robot-deeprobotics-lite3 package dir — robonix primitives docker-exec into the
# container started here, so it has to exist first. Mirrors the Webots sim
# bring-up (examples/webots/sim/start.sh) but launches robot_state_publisher
# directly with our URDF instead of a Webots controller. See ADR-0004.
#
# Re-running is safe: `docker compose up` reuses the running container.
# Stop from another terminal: `bash container/stop.sh` or `docker compose
# -f container/compose.yaml down`.
set -euo pipefail

export ROBONIX_LITE3_CONTAINER="${ROBONIX_LITE3_CONTAINER:-robonix_lite3_ros}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-robonix_lite3_ros}"
export ROBONIX_LITE3_ROS_BASE_IMAGE="${ROBONIX_LITE3_ROS_BASE_IMAGE:-robonix-ros:humble-ros-base}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "[container/start] base image:  $ROBONIX_LITE3_ROS_BASE_IMAGE"
echo "[container/start] container:    $ROBONIX_LITE3_CONTAINER"
echo "[container/start] RMW:          $RMW_IMPLEMENTATION  domain: $ROS_DOMAIN_ID"

# Pre-flight: the robonix-ros base image must exist locally (or be pullable).
# If a mirror is needed, point ROBONIX_LITE3_ROS_BASE_IMAGE at a mirror alias
# and `docker pull` it before running this.
if ! docker image inspect "$ROBONIX_LITE3_ROS_BASE_IMAGE" >/dev/null 2>&1; then
  echo "[container/start] base image '$ROBONIX_LITE3_ROS_BASE_IMAGE' not found locally." >&2
  echo "[container/start] docker pull it first, or set ROBONIX_LITE3_ROS_BASE_IMAGE" >&2
  echo "[container/start] to a locally-available alias." >&2
  exit 1
fi

# Build (first run installs rmw_zenoh_cpp; subsequent runs are cached) and start.
docker compose -f compose.yaml up -d --build

# Wait for robot_state_publisher to be alive inside the container so TF is
# publishing before `rbnx boot` execs the primitives in.
echo "[container/start] waiting for robot_state_publisher…"
ok=0
for _ in $(seq 1 30); do
  if docker exec "$ROBONIX_LITE3_CONTAINER" pgrep -x robot_state_publ >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 1
done
if [ "$ok" -ne 1 ]; then
  echo "[container/start] WARN: robot_state_publisher not detected after 30s." >&2
  echo "[container/start] check:  docker exec $ROBONIX_LITE3_CONTAINER tail -80 /tmp/robot_state_publisher.log" >&2
  exit 0
fi
echo "[container/start] $ROBONIX_LITE3_CONTAINER up; robot_state_publisher running."

# Vendor drivers are launched by the container entrypoint. Confirm their topics
# are up so `rbnx boot`'s primitives (camera/lidar sentinels) don't stall — but
# only WARN if absent: the operator may have disabled them (LITE3_ENABLE_*=0)
# or the hardware may be off, and the primitives themselves gate on first frame.
wait_topic() {
  local topic="$1" what="$2"
  local found=0
  for _ in $(seq 1 20); do
    if docker exec "$ROBONIX_LITE3_CONTAINER" \
         bash -lc "source /opt/ros/humble/setup.bash; timeout 1 ros2 topic info '$topic' >/dev/null 2>&1" \
         >/dev/null 2>&1; then
      found=1
      break
    fi
    sleep 1
  done
  if [ "$found" -eq 1 ]; then
    echo "[container/start] $what topic $topic up."
  else
    echo "[container/start] WARN: $what topic $topic not seen after 20s." >&2
    echo "[container/start]   check: docker exec $ROBONIX_LITE3_CONTAINER tail -80 /tmp/orbbec_camera.log" >&2
    echo "[container/start]   check: docker exec $ROBONIX_LITE3_CONTAINER tail -80 /tmp/livox_ros_driver2.log" >&2
    echo "[container/start]   (disable: LITE3_ENABLE_ORBBEC=0 / LITE3_ENABLE_LIVOX=0 before start)" >&2
  fi
}
wait_topic /camera/color/image_raw "Orbbec RGB"
wait_topic /livox/lidar "Livox MID-360S"

echo "[container/start] now run:  rbnx boot -f robonix_manifest.yaml"