#!/usr/bin/env bash
# Lite3 rviz 可视化一键启动 (Thor 上运行)
# 用法: bash tools/start_rviz.sh
# 前置: 主容器 robonix_lite3_ros 已起 (docker compose up -d) + 全栈 boot 或至少传感器在跑
# 说明: 所有配置固化在 tools/ 目录, Thor 重启不会丢 (/tmp 会被清)
set -euo pipefail

# ── 配置 (固化在部署目录, 不用 /tmp) ───────────────────
TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"
CT_NAME="robonix_rviz"
IMG="robonix-rviz:latest"
DISPLAY_N=":2"                        # thor-vnc TigerVNC :2 (5900 端口)
XAUTH="/home/nvidia/.Xauthority"
CFG="$TOOLS_DIR/rviz2_lite3.nocamera.rviz"   # 真机配置 (Fixed Frame=base_link + 相机)
ZENOH_CFG="$TOOLS_DIR/zenoh_rviz_config.json5"

# 若无 zenoh client 配置则生成 (显式连主容器 router, 拿 latched /tf_static)
if [ ! -f "$ZENOH_CFG" ]; then
  cat > "$ZENOH_CFG" << 'EOF'
{
  mode: "client",
  connect: { timeout_ms: 10000, endpoints: ["tcp/localhost:7447"] },
  listen: { endpoints: [] }
}
EOF
fi

if [ ! -f "$CFG" ]; then
  echo "[start_rviz] ERROR: 配置文件不存在: $CFG" >&2
  exit 1
fi

# 清理旧容器 (可重复运行)
docker rm -f "$CT_NAME" 2>/dev/null || true

echo "[start_rviz] 启动 $CT_NAME (Fixed Frame=base_link, zenoh client 模式)..."
docker run -d --name "$CT_NAME" --network host \
  -e DISPLAY="$DISPLAY_N" \
  -e XAUTHORITY="$XAUTH" \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=0 \
  -e ZENOH_CONFIG_FILE="$ZENOH_CFG" \
  -v "$XAUTH:$XAUTH:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$CFG:/tmp/rviz2_lite3.rviz:ro" \
  -v "$ZENOH_CFG:$ZENOH_CFG:ro" \
  "$IMG" bash -lc 'source /opt/ros/humble/setup.bash && sleep 3 && exec ros2 run rviz2 rviz2 -d /tmp/rviz2_lite3.rviz'

sleep 6
echo "[start_rviz] 容器已起。VNC (:5900) 查看; 日志: docker logs $CT_NAME"
echo "[start_rviz] 验证: docker exec $CT_NAME bash -lc 'grep \"Fixed Frame\" /tmp/rviz2_lite3.rviz | grep -v Reference'"
echo "[start_rviz] 验证: docker logs $CT_NAME 2>&1 | grep -c dropping   (应为 0)"
