#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# robonix_lite3_ros container entrypoint.
#
# Runs ONLY the environment layer: source ROS, bring up the zenoh router (for
# rmw_zenoh discovery across the host + sibling containers), and launch
# robot_state_publisher with the Lite3 URDF. The robonix primitives
# (lite3_chassis / lite3_camera) are NOT started here — `rbnx boot` docker
# exec's their drivers into THIS container after it is up (ADR-0004).
set -eo pipefail
source /opt/ros/humble/setup.bash
# Livox ROS driver 2 (MID-360S) colcon overlay — build.sh humble installs here.
if [ -f /livox_ws/install/setup.bash ]; then
  source /livox_ws/install/setup.bash
fi
set -u

# Allow `docker run -- <cmd>` to override the entrypoint (used by compose
# healthchecks or one-off probes).
if [ "$#" -gt 0 ]; then
  exec "$@"
fi

ZENOH_PID=""
RSP_PID=""
cleanup() {
  [ -n "${RSP_PID:-}" ]   && kill -TERM "$RSP_PID" 2>/dev/null || true
  [ -n "${ZENOH_PID:-}" ] && kill -TERM "$ZENOH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── zenoh router (needed so rmw_zenoh_cpp discovery crosses container/host) ──
start_zenoh_router() {
  if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ]; then
    return 0
  fi
  local router_bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
  if [ ! -x "$router_bin" ]; then
    echo "[lite3_ros] rmw_zenohd not found at $router_bin" >&2
    return 1
  fi
  "$router_bin" >/tmp/rmw_zenohd.log 2>&1 &
  ZENOH_PID=$!
  local i
  for i in $(seq 1 20); do
    if ! kill -0 "$ZENOH_PID" 2>/dev/null; then
      echo "[lite3_ros] rmw_zenohd exited early; last 80 lines:" >&2
      tail -80 /tmp/rmw_zenohd.log 2>&1 || true
      return 1
    fi
    if python3 - <<'PY' >/dev/null 2>&1
import socket
with socket.create_connection(("127.0.0.1", 7447), timeout=0.2):
    pass
PY
    then
      echo "[lite3_ros] rmw_zenohd ready on tcp/127.0.0.1:7447"
      return 0
    fi
    sleep 0.25
  done
  echo "[lite3_ros] rmw_zenohd did not listen on :7447" >&2
  return 1
}

start_zenoh_router

# ── robot_state_publisher: the single TF publisher (onboarding §3.1) ──────────
# Publishes /robot_description + the URDF's STATIC joint chain
# (base_link→TORSO→head_camera_* / ultrasonic_*). The DYNAMIC odom→base_link
# transform is NOT published here — it comes from the chassis primitive's
# /odom (a nav_msgs/Odometry topic); nav2/scene consume odom via the /odom
# topic, not a TF from state_publisher, so we keep one publisher for the
# fixed tree and let odom flow as a topic.
RSP_URDF="${LITE3_URDF_PATH:-/robonix_pkgs/urdf/Lite3.urdf}"
if [ ! -f "$RSP_URDF" ]; then
  echo "[lite3_ros] WARN: URDF not found at $RSP_URDF — TF tree will be empty" >&2
fi

# robot_state_publisher reads the URDF from the robot_description parameter and
# publishes /tf_static for the fixed joints; the joint_states subscriber
# (subscribed by RSP) drives the moving leg joints once lite3_chassis publishes
# /joint_states.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY=0

# Read the file with xacro-less substitution: pass the raw URDF content.
URDF_CONTENT="$(cat "$RSP_URDF" 2>/dev/null || echo '<robot name=\"empty\"><link name=\"base_link\"/></robot>')"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$URDF_CONTENT" \
  -r __ns:=/ >/tmp/robot_state_publisher.log 2>&1 &
RSP_PID=$!
echo "[lite3_ros] robot_state_publisher pid=${RSP_PID} (urdf=$RSP_URDF)"

# Stay alive so `rbnx boot` driver packages can docker exec in. SIGTERM from
# `compose down` reaches us via the trap, killing RSP + the zenoh router.
echo "[lite3_ros] environment ready — primitives may docker exec now"
wait "$RSP_PID"