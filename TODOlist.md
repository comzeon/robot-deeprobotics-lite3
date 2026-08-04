# robot-deeprobotics-lite3 接入完整性检查与修复 TODO List

审查依据：[Robonix 本体接入指南](https://robonix-book.syswonder.org/integration-guide/vendor-onboarding)（§1 catalog / §3.1 URDF·TF / §3.2 provider_id / §4.2 RBNX_INSTANCE_NAME·config.spec / §4.4 能力须由代码声明 / §5 服务接线 / §6.1 odom·TF / §7 验收 / §8 社区包元数据）
协议参照：云深处开源 [`DeepRoboticsLab/Lite3_ROS`](https://github.com/DeepRoboticsLab/Lite3_ROS) 的 `protocol.hpp`/`Jetson2Motion.cpp`，并对照独立实现 [`automatika-robotics/emos-plugin-lite3`](https://github.com/automatika-robotics/emos-plugin-lite3)（`ctypes _pack_=4`，与 C 结构逐字段一致）。
架构决策：见 `docs/adr/0001-0003`；术语见 `CONTEXT.md`。

> 本轮已完成 P0–P4 全部条目。**协议层正确性经字节布局核对**（ctypes 结构尺寸与官方一致：RobotStateReceived=212B / JointStateReceived=108B / HandleStateReceived=60B / SimpleCMD=12B / ComplexCMD=20B），**未做实机抓包**——实机 UDP 协议验证请在 `100.72.167.58` 上 `rbnx boot` 后用 `ros2 topic echo /odom` 确认（决策时该主机仅密码登录，我无法非交互登录）。

---

## P0 — 关键正确性（底盘驱动按官方协议重写）✅

### 1. 底盘状态帧过滤码 / 帧分派 — 已修复 ✅
- [x] 改用 ctypes `_pack_=4` 结构逐字节镜像 `protocol.hpp`（`RobotStateReceived`/`JointStateReceived`/`HandleStateReceived`/`ImuDataReceived`）。
- [x] `udp_loop` 按包长 + `code`（2305/2306/2309/0x010901）分派，等价 C++ 的 `switch(recv_num_)`；同时兼容较新固件 4 字节更大的 `RobotStateReceivedWithPolicy` 布局。
- [x] 移除错误常量 `CMD_ROBOT_DATA = 0x00000906`。→ ADR-0001

### 2. 状态负载布局 — 已修复 ✅
- [x] `RobotState` 按官方字段解析：`rpy/rpy_vel/xyz_acc/pos_world[3]`/`vel_world/vel_body/battery_level/ultrasound[2]`。
- [x] `odom` 取 `pos_world[0]`(x)、`pos_world[1]`(y)、`pos_world[2]`（yaw, rad）+ `vel_body` 速度项，`/odom`(nav_msgs/Odometry) 已发布。
- [x] 前后超声波 `ultrasound[0]/[1]` 发布为 `/ultrasound/front`、`/ultrasound/rear`（sensor_msgs/Range）。

### 3. 命令格式 — 已修复 ✅
- [x] 改发官方 `MotionComplexCMD`（`SimpleCMD`+double）到 `:43893`；速度为三包，`cmd_code` 320(vx)/325(vy)/321(wz)，yaw 按桥约定取负。
- [x] `move` rpc 与 `/cmd_vel` 转为 vx/vy/wz；删除发送 240B 全零关节位姿的危险路径。→ ADR-0003
- [x] 安全边界：无速度输入 0.5s 即发零速；不发包时运动主机自停（与官方 `transfer` 一致），**不发 keep-alive**。

### 4. 网络默认值 — 已修复 ✅
- [x] 采用官方默认：运动主机 `192.168.1.1`，感知主机 `192.168.1.120`，`cmd_port 43893`，`state_port 43897`；写入 README “硬件 & 网络”表。RTSP 路径已移除（改用奥比中光 RGB-D）。

---

## P1 — 能力声明须由代码声明 (§4.4) ✅

### 5. `robonix/primitive/chassis/move` gRPC 处理 ✅
- [x] 用 `@lite3.grpc("robonix/primitive/chassis/move")` 绑定 `move()`，import `chassis_pb2`/`std_msgs_pb2`，按 `MoveCommand` 速度字段执行。

### 6. `odom` / `twist_in` 数据面 ✅
- [x] `init` 中 `create_publisher` 发布 `/odom`，`create_subscription` 订阅 `/cmd_vel`（`_on_cmd_vel`）。
- [x] manifest 新增 `env:` 块（`ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION`），运行时 RMW 一致化。

### 7. `robonix/primitive/camera/snapshot` MCP 工具 ✅
- [x] `lite3_camera` 用 `@lite3_cam.mcp("...snapshot")`/`...depth_snapshot` 绑定，返回 `sensor_msgs_mcp.Image`。
- [x] `rgb`/`depth` 改为真订阅 `create_subscription`（从奥比中光 `orbbec_camera` 驱动桥接）；package_manifest 能力清单与实现对齐（含 intrinsics/extrinsics/driver）。

### 8. 提供方 ID 使用 `RBNX_INSTANCE_NAME` ✅
- [x] 两原语均 `Primitive(id=os.environ.get("RBNX_INSTANCE_NAME", ...))`。

---

## P2 — soma 声明与清单接线 (§3.2 / §5) ✅

### 9. `exports[].provider_id` 与清单一致 ✅
- [x] soma 导出改为 `nav2` / `explore`，逐字等于 manifest 实例名；删除原 `lite3_nav2`/`skill_explore`/`lite3_audio` 引用。

### 10. 建图/导航服务接线 ✅
- [x] manifest 新增 `service.mapping`（`rtabmap_inputs:[lidar,rgbd,odom]`、`occupancy_sources:[lidar,depth]`、`sensor_providers: {lidar2d: lite3_lidar, rgb: lite3_camera, depth: lite3_camera, odom: lite3_chassis}`）。
- [x] manifest 新增 `service.nav2`（`provider_ids: {map: mapping, odom: lite3_chassis, scan: lite3_lidar}`）。
- [x] 新增 `lite3_lidar` 原语：订阅 MID-360S 的 `/livox/lidar`(PointCloud2) → 水平带切片成 `/scan`(LaserScan)，导出 `robonix/primitive/lidar/{lidar,snapshot,driver}`。
- [x] `nav2_params.yaml` 的 `/scan` 障碍层改为真实 lidar；`soma` 增 `lidar_2d` 组件(`urdf_link: lidar_link`)。→ ADR-0002 / ADR-0004

### 11. `robot_description` / TF 发布方 ✅ → 已改为容器化
- [x] **不使用** 外部 `primitive-robot-description-rbnx`（裸 L4T 主机无 `/opt/ros/humble`，其 build.sh 必失败）。改为在 `robonix_lite3_ros` 容器内由 `robot_state_publisher` 发布 URDF 固定 TF 链（`base_link`→`TORSO`→`head_camera_*`/`ultrasonic_*`）。→ ADR-0004
- [x] URDF 新增 `base_link`→`TORSO` 固定关节，TF 树连通（已用 ElementTree 校验 23 link）。
- [x] 新增 `container/`（compose.yaml/Dockerfile/entrypoint.sh/start.sh/stop.sh）：`robonix-ros:humble-ros-base` + `apt ros-humble-rmw-zenoh-cpp`，`--network host`+`--ipc host`，entrypoint 启动 zenoh 路由器 + `robot_state_publisher` 后常驻供原语 `docker exec`。
- [x] 两个原语 `scripts/start.sh` 改为 `docker exec` 进 `robonix_lite3_ros`（镜像 `tiago_chassis`/`tiago_camera` 的 `docker exec` 模板，含 advertise-host 解析、容器内 PYTHONPATH、日志 tail）。
- [x] manifest `env.RMW_IMPLEMENTATION` 改为 `rmw_zenoh_cpp`（与 webots 及 mapping/nav2/scene 容器同域）。

### 12. 音频能力 ✅
- [x] 从 soma / manifest 删除 `audio` 组件与 `lite3_audio` 导出；`cannot_do` 写明“未部署音频”。
- [x] `head_camera` 的 `urdf_link` 改为 `head_camera_rgb_optical_frame`（URDF 真实光学坐标系）。

---

## P3 — URDF / 传感器完整性 (§3.1) ✅

### 13. URDF 传感器坐标树 ✅
- [x] 新增 `head_camera_link`/`head_camera_rgb_optical_frame`/`head_camera_depth_optical_frame`、`ultrasonic_front_link`/`ultrasonic_rear_link`（均固定到 `TORSO`，含 6DOF 安装位姿估计；README 提示实机核对）。

### 14. soma 部件树 / `cannot_do` ✅
- [x] `head_camera`(type `rgbd_camera`) + 前后 `range_sensor` + `lidar_2d`(MID-360S) 部件；`cannot_do` 列速度-only、音频未部署、leg-odom 漂移（已**无**"无 lidar/无深度"残留——两者均已部署）。

---

## P4 — 打包卫生与发布 (§1/§2/§8) ✅

### 15. `catalog:` 元数据块 ✅
- [x] manifest 顶部补 `catalog:`（name/version/description/license/tags/maintainers）。

### 16. `config.spec` / 元数据 ✅
- [x] 两 package_manifest 补 `tags`/`maintainers`/`vendor`/`version`；`config.spec` 列全 `robot_ip/cmd_port/state_port/...` 与 `rgb_topic/depth_topic/camera_info_topic/fx...`。

### 17. README 与启动脚本 ✅
- [x] 新增 `README.md`：平台/硬件/网络/坐标系/安全边界/构建启动流程。
- [x] `start.sh` 移除裸 `$HOME/Desktop/robonix` 兜底：不存在则明确报错并指引 `rbnx setup`/导出 `ROBONIX_SOURCE_PATH`。

### 18. 运行时/临时产物 ✅
- [x] `git rm` 已跟踪的 `driver.py.bak`、`primitives/*/logs/*.log`（4 项）；`.gitignore` 增补 `logs/`/`*.log`/`*.bak`/`*.swp`。

### 19. 启动脚本硬编码 — 见 #17 ✅

### 20. MID-360S 激光雷达接入 ✅
- [x] `container/Dockerfile` 源码编译 `Livox-SDK2(v1.3.1)` + `livox_ros_driver2`（`build.sh humble` → colcon install 到 `/livox_ws/install`，打进镜像持久化）。
- [x] `container/entrypoint.sh` source `/livox_ws/install/setup.bash`，容器内 `ros2 launch livox_ros_driver2` 可用。
- [x] 新增 `primitives/lite3_lidar`（PointCloud2→LaserScan 切片原语，含 config.spec/start.sh docker exec/CAPABILITY.md）。
- [x] manifest 接线 `lite3_lidar`（mapping `lidar2d` + nav2 `scan`）；soma `lidar_2d` 组件；URDF `lidar_link`（固定 TORSO，估计安装位姿）；README 接入步骤。

### 21. GitHub clone 网络容错（容器构建）✅
- [x] `container/Dockerfile` 的 Livox clone 加 `git config http.version HTTP/1.1` + `http.postBuffer 524288000`，缓解 `curl 16 HTTP2 framing layer`（国内直连 GitHub 典型失败）。
- [x] clone URL 参数化为 `ARG LIVOX_SDK2_URL` / `ARG LIVOX_ROS2_DRIVER_URL`，可指向 Gitee 镜像/fork 绕过 GitHub。
- [ ] 若 HTTP/1.1 仍不稳定：`container/start.sh` 可前置 `git clone` 到本地缓存目录再 `--build` 复用（构建期多阶段缓存）。

### 22. vendor 驱动收进容器 entrypoint ✅
- [x] `container/entrypoint.sh` 在 zenohd + robot_state_publisher 之后启动 `orbbec_camera`（`gemini_330_series.launch.py`，`depth_registration:=true`）与 `livox_ros_driver2`（`msg_MID360s_launch.py`），`container/start.sh` 一键起全套；两驱动可用 `LITE3_ENABLE_ORBBEC=0`/`LITE3_ENABLE_LIVOX=0` 禁用（调试期/无硬件）。
- [x] `container/start.sh` 起容器后确认 `/camera/color/image_raw` 与 `/livox/lidar` 两个 vendor topic 就绪（缺席只 WARN 不失败，原语有 sentinel）。
- [x] README 构建步骤改为"udev 一次性 + container/start.sh 一键起 → rbnx boot"；ADR-0004 补充 vendor 驱动归属。

---

## 待你在实机上验证（我无法非交互登录）

```bash
# 前提：host 100.72.167.58 是裸 L4T（无 ROS），原语与 robot_state_publisher 都在
# robonix_lite3_ros 容器里跑（ADR-0004）。
rbnx validate ./primitives/lite3_chassis
rbnx validate ./primitives/lite3_camera
rbnx validate ./primitives/lite3_lidar
rbnx build -f robonix_manifest.yaml          # 期望 Failed:0 / Skipped:0 / Built==Total
sudo cp /opt/ros/humble/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger   # 一次性
bash container/start.sh                       # 一键起全套：zenohd + rsp + orbbec + livox
rbnx boot -f robonix_manifest.yaml
rbnx caps -v                                   # 期望 lite3_chassis/lite3_camera/lite3_lidar 为 ACTIVE
docker exec robonix_lite3_ros ros2 topic echo /odom --once          # 协议层唯一真值验证
docker exec robonix_lite3_ros ros2 run tf2_ros tf2_echo odom base_link
docker exec robonix_lite3_ros ros2 topic echo /ultrasound/front --once
docker exec robonix_lite3_ros ros2 topic echo /scan --once          # 验证 MID-360S 切片
# 协议若不通：核对运动主机 IP/网段、确认感知主机 43897 未被官方 transfer 占用、
#            确认 rbnx 启动了容器（`docker ps | grep robonix_lite3`）。
# /scan 为空：核对 MID360s_config.json 的 lidar IP（默认 192.168.1.100）+ host；
#             vendor 日志：`docker exec robonix_lite3_ros tail -80 /tmp/livox_ros_driver2.log`。
```

## 遗留未决（需你决定后我可继续）

- URDF 传感器安装位姿为**估计值**（camera +x0.18/+z0.18，超声波 ±0.25，lidar z=0.30）；实机量测后回填。
- `rtabmap_params.yaml` 暂未针对 lidar+depth 融合做完整调参，建议实机建图后迭代。
- 若 Git clone 仍不稳定：`container/start.sh` 可改为先在宿主预 clone 到本地缓存再 `--build`（多阶段复用），或 `docker build --build-arg LIVOX_SDK2_URL=<gitee镜像>`。