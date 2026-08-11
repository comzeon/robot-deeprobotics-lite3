#!/usr/bin/env bash
# sensor_watchdog.sh - camera/lidar health watchdog (wrapper, no upstream edits)
# started by entrypoint; restarts the vendor driver if its topic goes silent
set -u
LOG=/tmp/sensor_watchdog.log
log() { echo "[$(date +%H:%M:%S)] $*" >> "$LOG"; }

alive() { timeout "${2:-8}" ros2 topic echo "$1" --once >/dev/null 2>&1; }

restart_orbbec() {
  log "orbbec DEAD -> restarting"
  pkill -f "gemini_330_series.launch" 2>/dev/null; pkill -f "component_container" 2>/dev/null
  sleep 2
  ros2 launch orbbec_camera gemini_330_series.launch.py \
    depth_registration:=true enable_point_cloud:=false \
    color_width:=640 color_height:=480 color_fps:=30 \
    depth_width:=640 depth_height:=480 depth_fps:=30 > /tmp/orbbec_camera.log 2>&1 &
}

restart_livox() {
  log "livox DEAD -> restarting"
  pkill -f "livox_ros_driver2" 2>/dev/null
  sleep 2
  local cfg=/robonix_pkgs/container/vendor/livox_ros_driver2/config/MID360s_config.json
  local img=/livox_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json
  [ -f "$cfg" ] && cp "$cfg" "$img"
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py \
    user_config_path:="$cfg" > /tmp/livox_ros_driver2.log 2>&1 &
}

sleep 30
declare -A fail
while true; do
  for pair in "orbbec /camera/color/image_raw" "livox /livox/lidar"; do
    set -- $pair; name=$1; topic=$2
    if alive "$topic"; then fail[$name]=0
    else
      fail[$name]=$(( ${fail[$name]:-0} + 1 ))
      log "$name no data on $topic (fail=${fail[$name]})"
      if [ "${fail[$name]}" -ge 2 ]; then
        case "$name" in orbbec) restart_orbbec;; livox) restart_livox;; esac
        fail[$name]=0; sleep 25
      fi
    fi
  done
  sleep 10
done
