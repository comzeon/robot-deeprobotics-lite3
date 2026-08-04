#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# robonix_lite3_ros container entrypoint.
#
# Brings up the ENTIRE ROS 2 environment layer:
#   1. zenoh router     (rmw_zenoh discovery across host + sibling containers)
#   2. robot_state_publisher  (the single TF publisher for the URDF fixed chain)
#   3. orbbec_camera driver  (Gemini 335 RGB-D)   — vendor, out-of-band
#   4. livox_ros_driver2     (MID-360S lidar)     — vendor, out-of-band
# then stays alive so `rbnx boot` docker exec's the robonix primitives
# (lite3_chassis / lite3_camera / lite3_lidar) into THIS container (ADR-0004).
#
# The vendor drivers are optional: if the hardware is absent (bring-up / bench),
# set LITE3_ENABLE_ORBBEC=0 / LITE3_ENABLE_LIVOX=0 to skip them — the entrypoint
# logs a WARN and continues so the chassis-only environment still comes up.
set -eo pipefail
source /opt/ros/humble/setup.bash
# Livox ROS driver 2 (MID-360S) colcon overlay — build.sh humble installs here.
# Guarantee it is on AMENT_PREFIX_PATH so `ros2 pkg prefix livox_ros_driver2`
# resolves. Prefer sourcing setup.bash; fall back to exporting the vars directly
# if the file is missing/unreadable (the Docker build VM runs colcon as root and
# the runtime user may differ).
ensure_livox_overlay() {
  if [ -f /livox_ws/install/setup.bash ]; then
    if [ -r /livox_ws/install/setup.bash ]; then
      # shellcheck disable=SC1091
      source /livox_ws/install/setup.bash
    else
      echo "[lite3_ros] WARN: /livox_ws/install/setup.bash unreadable — chmod + retry" >&2
      chmod a+r /livox_ws/install/setup.bash /livox_ws/install/local_setup.* 2>/dev/null || true
      # shellcheck disable=SC1091
      source /livox_ws/install/setup.bash 2>&1 || true
    fi
  fi
  # idempotent manual fallback — ensures discovery even if setup.bash is absent
  case ":$AMENT_PREFIX_PATH:" in
    *":/livox_ws/install:"*) : ;;
    *) export AMENT_PREFIX_PATH="/livox_ws/install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
  if [ -d /livox_ws/install/lib ]; then
    case ":$LD_LIBRARY_PATH:" in
      *":/livox_ws/install/lib:"*) : ;;
      *) export LD_LIBRARY_PATH="/livox_ws/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
    esac
  fi
}
ensure_livox_overlay
set -u

# Allow `docker run -- <cmd>` to override the entrypoint (used by compose
# healthchecks or one-off probes).
if [ "$#" -gt 0 ]; then
  exec "$@"
fi

RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export RMW_IMPLEMENTATION
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY=0

ZENOH_PID=""
RSP_PID=""
ORBBEC_PID=""
LIVOX_PID=""
_children=()
cleanup() {
  for pid in "${_children[@]:-}"; do
    [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# ── zenoh router (needed so rmw_zenoh_cpp discovery crosses container/host) ──
start_zenoh_router() {
  local router_bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
  if [ ! -x "$router_bin" ]; then
    echo "[lite3_ros] rmw_zenohd not found at $router_bin" >&2
    return 1
  fi
  "$router_bin" >/tmp/rmw_zenohd.log 2>&1 &
  ZENOH_PID=$!
  _children+=("$ZENOH_PID")
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
# (base_link→TORSO→head_camera_* / ultrasonic_* / lidar_link). The DYNAMIC
# odom→base_link transform is NOT published here — it comes from the chassis
# primitive's /odom topic; we keep one publisher for the fixed tree.
RSP_URDF="${LITE3_URDF_PATH:-/robonix_pkgs/urdf/Lite3.urdf}"
if [ ! -f "$RSP_URDF" ]; then
  echo "[lite3_ros] WARN: URDF not found at $RSP_URDF — TF tree will be empty" >&2
fi

# Pass the URDF through a YAML params file, NOT `-p robot_description:="..."`.
# A multiline URDF as a CLI --ros-args -p value trips ROS2's parameter parser
# (this crashed robot_state_publisher at startup, ExitCode 250). Writing it into
# a YAML block scalar and using --params-file is robust.
RSP_PARAMS=/tmp/rsp_params.yaml
{
  printf 'robot_state_publisher:\n  ros__parameters:\n    robot_description: |\n'
  # indent every URDF line so YAML block-scalar preserves the content verbatim
  sed 's/^/      /' "$RSP_URDF" 2>/dev/null || printf '      <robot/>\n'
} > "$RSP_PARAMS"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "$RSP_PARAMS" \
  -r __ns:=/ >/tmp/robot_state_publisher.log 2>&1 &
RSP_PID=$!
_children+=("$RSP_PID")
echo "[lite3_ros] robot_state_publisher pid=${RSP_PID} (urdf=$RSP_URDF)"

# ── vendor driver: Orbbec Gemini 335 (RGB-D) ─────────────────────────────────
start_orbbec() {
  local log=/tmp/orbbec_camera.log
  if ! ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
    echo "[lite3_ros] orbbec_camera not installed — skipping RGB-D (set LITE3_ENABLE_ORBBEC=0 to silence)" >&2
    return 0
  fi
  ros2 launch orbbec_camera gemini_330_series.launch.py \
    depth_registration:=true enable_point_cloud:=false \
    color_width:=640 color_height:=480 color_fps:=30 \
    depth_width:=640 depth_height:=480 depth_fps:=30 \
    >"$log" 2>&1 &
  ORBBEC_PID=$!
  _children+=("$ORBBEC_PID")
  echo "[lite3_ros] orbbec_camera pid=${ORBBEC_PID} (log=$log)"
}

# ── vendor driver: Livox MID-360S (3D lidar) ────────────────────────────────
start_livox() {
  local log=/tmp/livox_ros_driver2.log
  if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
    echo "[lite3_ros] livox_ros_driver2 not installed — skipping lidar (set LITE3_ENABLE_LIVOX=0 to silence)" >&2
    return 0
  fi
  # Point the driver at the HOST-editable config (bind-mounted at /robonix_pkgs).
  # The in-image default under /livox_ws/install is baked in and would be lost
  # on container recreation; this path survives `compose down/up`. Edit
  # container/vendor/livox_ros_driver2/config/MID360s_config.json on the host.
  local user_config="${LITE3_LIVOX_CONFIG:-/robonix_pkgs/container/vendor/livox_ros_driver2/config/MID360s_config.json}"
  if [ ! -f "$user_config" ]; then
    echo "[lite3_ros] WARN: livox config not found at $user_config — using in-image default" >&2
    user_config=""
  fi
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py \
    ${user_config:+user_config_path:="$user_config"} \
    >"$log" 2>&1 &
  LIVOX_PID=$!
  _children+=("$LIVOX_PID")
  echo "[lite3_ros] livox_ros_driver2 pid=${LIVOX_PID} (config=$user_config, log=$log)"
}

if [ "${LITE3_ENABLE_ORBBEC:-1}" = "1" ]; then start_orbbec; else
  echo "[lite3_ros] orbbec_camera disabled (LITE3_ENABLE_ORBBEC=0)" >&2
fi
if [ "${LITE3_ENABLE_LIVOX:-1}" = "1" ]; then start_livox; else
  echo "[lite3_ros] livox_ros_driver2 disabled (LITE3_ENABLE_LIVOX=0)" >&2
fi

# Stay alive so `rbnx boot` driver packages can docker exec in. SIGTERM from
# `compose down` reaches us via the trap, killing all children. If robot_state_
# publisher dies (the primary TF source), exit so a broken TF tree is obvious.
echo "[lite3_ros] environment ready — primitives may docker exec now"
wait "$RSP_PID"