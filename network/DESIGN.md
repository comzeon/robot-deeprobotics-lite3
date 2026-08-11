# 连接统一管理设计（network/）

> 状态：**设计稿**（2026-08-07）。本目录目前只含设计文档，暂不落可执行脚本。
> 适用范围：`robot-deeprobotics-lite3` 在 Thor（Jetson AGX Thor, JetPack 7.2, Ubuntu 24.04）上的实机部署。
> 相关设备：机器狗 Lite3（RK3588 感知主机）、Thor 主机、Livox MID-360S 雷达、Orbbec Gemini 335 相机。

---

## 1. 背景与目标

实机设备间的物理连线（网线 / USB 转网口）目前是临时拉线，换场地或换设备（如更换 USB 转网口适配器）后：

- 接口名 `enx<mac>` 由 MAC 派生，换了适配器接口名即变，NetworkManager 连接失配；
- IP 硬编码散布在 5+ 个文件中，漏改一处即断链；
- 策略路由（雷达 .168）未持久化，重启丢失。

**目标**：单一权威连接清单 + 一个 wrapper 脚本，统一管理有线 + 无线链路。
迁移流程收敛为：**改清单 → dry-run 看 diff → apply → verify**。

**约束**（用户偏好）：极简 wrapper、不改上游文件、写操作前先备份、原子化小步执行、高风险操作（改机器人侧/抓包）先征求同意。

---

## 2. 现状盘点（2026-08-07 实测）

### 2.1 物理拓扑

```
机器狗 Lite3 (RK3588)            Thor (Jetson AGX Thor)              外设
┌─────────────────┐   网线    ┌──────────────────────────┐
│ eth1  = .120     │◄────────►│ enP2p1s0 = .130  (有线)    │
│ jy_exe: 收 43893 │           │ wlP1p1s0 = .29   (WiFi)    │
│ 推 43897 → 目标? │           │ enx*…fd63 = .131 (USB网卡) │◄─网线─► Livox MID-360S .168
└─────────────────┘           │ USB: Orbbec Gemini 335     │  (USB 直连)
                               └──────────────────────────┘
  tailscale0 = 100.72.167.58（远程管理 / 代理出口）
```

### 2.2 配置散落点

| 链路 | Thor 主机侧（NetworkManager） | 项目文件（IP 硬编码） | 对端侧 |
|---|---|---|---|
| 狗↔Thor 有线 | `lite3-link`：enP2p1s0 静态 .130/24 | `Lite3_ROS/src/transfer/launch/transfer_launch.py` target_ip **.120**:43893 / local 43897；`Jetson2Motion.cpp` 默认值同；`primitives/lite3_chassis/lite3_driver/driver.py` DEFAULT_ROBOT_IP **.120**（动作帧直发） | RK3588 eth1=.120；jy_exe `conf/network.toml` local 43893 / target 43897 / ip=上位机（当前靠 **iptables DNAT** 临时把推流转到 Thor .130，默认目标是 ubuntu-box 192.168.2.195） |
| 雷达↔Thor 以太 | `Wired connection 6`（**泛名**）：enxec9a0c10fd63 静态 .131/24 | `container/vendor/livox_ros_driver2/config/MID360s_config.json`：lidar ip **.168**、host_ip **.131**、端口 56100–56500 | Livox 出厂 .168 |
| 相机↔Thor USB | 无网络配置（USB 枚举 2bc5:0800） | `container/compose.yaml` devices 透传 /dev/bus/usb；`robonix_manifest.yaml` 相机 topics `/camera/color\|depth/image_raw` | 无 IP |
| Thor→外网（无线） | `caiwu-5g` WiFi DHCP .29（默认路由走它）；tailscale0 100.72.167.58 | — | 公司网关 .1 |
| 策略路由 | `/etc/iproute2/rt_tables` 表 100 `lidar`；`/etc/systemd/system/lite3-routing.service`（**仅固化 .120→enP2p1s0**） | — | — |

### 2.3 迁移隐患

1. **⚠️ .168 雷达路由无持久化**：`lite3-routing.service` 只固化 .120 一条规则；`.168 → enx*（src .131）`是运行时手工加的，**重启即丢**。当前靠 USB 网卡 metric 100 最低碰巧能通，但网卡枚举顺序变化或换适配器后流量会改走 WiFi 出口，雷达失联。
2. **IP 硬编码散布 5+ 处**：.168/.131/.120 写死在 JSON、launch.py、C++ 默认值、driver.py、RK3588 network.toml/DNAT，迁移时逐处手改。
3. **`Wired connection 6` 是 NM 自动泛名**，且接口名 `enxec9a0c10fd63` 由 MAC 派生——换 USB 转网口即失配。
4. **狗侧推流目标是临时 DNAT**：`iptables -t nat -A OUTPUT -d 192.168.2.195 … --to 192.168.1.130:43897` 重启丢失，未落盘。
5. WiFi 是 DHCP，换场地改网段需同步改。

---

## 3. 设计

### 3.1 目录结构

```
network/
├── topology.yaml      # 权威连接清单（唯一需要人工编辑的文件）
├── net.sh             # wrapper：status / dry-run / apply / verify
└── DESIGN.md          # 本文档
```

### 3.2 `topology.yaml` schema（草案）

```yaml
# 每台主机一个 inventory；先覆盖 Thor，RK3588 侧见 §3.6
host: thor

interfaces:
  dog:                              # 机器狗有线链路
    type: wired
    nm_connection: lite3-dog-link   # 期望的 NM 连接名（有含义，非泛名）
    iface: enP2p1s0                 # PCIe 固定名，稳定
    ip: 192.168.1.130/24
    gateway: null
    policy_routes:                  # 写入策略路由表 lidar
      - {peer: 192.168.1.120, table: lidar, src: 192.168.1.130}
  lidar:                            # 雷达 USB 转以太链路
    type: usb-ethernet
    nm_connection: lite3-lidar-link
    iface: enlidar0                 # udev 稳定名（§3.4），替代 enx<mac>
    mac_prefix: "ec:9a:0c"          # 实测 enxec9a0c10fd63 的 MAC 前缀
    ip: 192.168.1.131/24
    gateway: null
    policy_routes:
      - {peer: 192.168.1.168, table: lidar, src: 192.168.1.131}
  wifi:
    type: wireless
    nm_connection: caiwu-5g
    ssid: caiwu-5g
    dhcp: true
    gateway: 192.168.1.1
  remote:
    type: tailscale
    enabled: true

peers:
  dog:   192.168.1.120   # RK3588 eth1 / jy_exe 命令口
  lidar: 192.168.1.168   # Livox MID-360S

project_files:           # apply 时要同步 IP 的项目文件（写前先备份）
  - path: container/vendor/livox_ros_driver2/config/MID360s_config.json
    bindings:
      - {key: "lidar_configs[0].ip",        value: "$(peers.lidar)"}
      - {key: "host_net_info[0].host_ip",   value: "$(interfaces.lidar.ip)"}
  - path: Lite3_ROS/src/transfer/launch/transfer_launch.py
    bindings:
      - {key: "jetson2motion.target_ip",    value: "$(peers.dog)"}
  - path: primitives/lite3_chassis/lite3_driver/driver.py
    bindings:
      - {key: "DEFAULT_ROBOT_IP",           value: "$(peers.dog)"}
```

### 3.3 `net.sh` 行为（草案，未实现）

- **`net.sh status`**：只读。打印当前 `ip addr` / `ip rule` / `ip route table lidar` / NM 连接名，与清单逐项对照。
- **`net.sh dry-run`**：只读。计算期望 vs 当前 diff（NM 连接、IP、路由、规则、项目文件 IP），不写任何东西。
- **`net.sh apply`**：写操作，顺序执行：
  1. 备份将修改的项目文件到 `network/backup/<date>/`（原文件不动，先备份再改）；
  2. 写 udev 规则（§3.4）→ `udevadm control --reload`（接口改名需重启或手动 rename，apply 时提示）；
  3. `nmcli` 建/改连接（`lite3-dog-link` / `lite3-lidar-link`，静态 IP，含 `ipv4.routing-rules` 可选）；
  4. 确保 `/etc/iproute2/rt_tables` 有 `100 lidar`；重写 `lite3-routing.service`（.120 **和 .168** 两条规则都固化）→ `systemctl daemon-reload && systemctl enable --now lite3-routing.service`；
  5. 按 `project_files` 同步 IP（先备份，sed 或生成式写回）；
  6. **只打印** RK3588 侧建议命令（§3.6），不自动 ssh 执行。
- **`net.sh verify`**：只读。ping 对端（.120/.168）；可选 `--no-ros` 跳过 ROS 检查；默认检查 `/livox/lidar`、`/odom`、`/camera/color/image_raw` topic 可达性（进容器 `ros2 topic info`）。

### 3.4 udev 稳定名（草案）

```udev
# /etc/udev/rules.d/90-lite3-net.rules
# 按 MAC 前缀把 USB 转网口固定为 enlidar0（替代 enx<mac> 派生名）
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="ec:9a:0c:*", NAME="enlidar0"
```

- 前缀 `ec:9a:0c` 来自实测接口 `enxec9a0c10fd63`（enx + MAC 十六进制）。
- 改名后 NM 连接 `lite3-lidar-link` 绑定 `interface-name=enlidar0`。
- **换适配器时只需改规则的 MAC 前缀**，连接名、路由、清单其余部分都不用动。

### 3.5 路由持久化（修复 .168 隐患）

现状：`/etc/systemd/system/lite3-routing.service` 只固化 .120→enP2p1s0；.168→enx* 是运行时手工加。
目标：把 .168 规则并入同一单元（**改动最小，优先**；备选：改用 NM `ipv4.routing-rules`，二选一，实施时定）。
合并后 ExecStart 草案：

```sh
ip rule del to 192.168.1.120 pref 100 lookup lidar 2>/dev/null; ip rule add to 192.168.1.120 pref 100 lookup lidar
ip rule del to 192.168.1.168 pref 100 lookup lidar 2>/dev/null; ip rule add to 192.168.1.168 pref 100 lookup lidar
ip route del 192.168.1.120 dev enP2p1s0 table lidar 2>/dev/null; ip route add 192.168.1.120 dev enP2p1s0 table lidar
ip route del 192.168.1.168 dev enlidar0 table lidar src 192.168.1.131 2>/dev/null; ip route add 192.168.1.168 dev enlidar0 table lidar src 192.168.1.131
```

> .168 的 table lidar 路由带 `src .131`：显式固定源地址，防止多网口同网段下源地址漂移（这正是 .120 规则存在的同样原因）。

### 3.6 RK3588 侧（机器狗）

狗侧连接定义不在 Thor 项目内，但属于同一连接域，清单应记录：

- `conf/network.toml`：jy_exe `local_port=43893`（收命令）、`target_port=43897`（推状态）、`ip`=上位机 IP。
- 现状：推流目标默认 ubuntu-box 192.168.2.195，改推 Thor 靠**临时 DNAT**（重启丢失）：
  `iptables -t nat -A OUTPUT -d 192.168.2.195 -p udp --dport 43897 -j DNAT --to-destination 192.168.1.130:43897`
- 建议（迁移后二选一，**需用户确认访问路径与偏好**）：
  - a) 直接改 `network.toml` 的 `ip`=Thor 地址（持久，但要编辑机器人文件，先备份）；
  - b) DNAT 规则落盘（nftables / iptables-persistent，机器人文件零改动）。
- `net.sh apply` 第 6 步只打印建议命令，不自动执行（机器人操作前须征求同意）。

### 3.7 迁移流程 SOP

1. 编辑 `network/topology.yaml`（新 IP / 新接口 / 新 WiFi）；
2. `net.sh status` 看当前实际状态；
3. `net.sh dry-run` 核对 diff；
4. `net.sh apply`（自动备份）；
5. `net.sh verify`（ping 对端 + ROS topic 检查）；
6. 按输出执行 RK3588 侧命令（如需要，先征求同意）。

---

## 4. 验收标准

- Thor 重启后：`ip rule` / `ip route table lidar` 与清单一致（.120 与 .168 都在）；
- 换 USB 转网口后：接口仍是 `enlidar0`（udev 生效），NM 连接自动匹配；
- `net.sh verify` 全绿：ping .120/.168 通；`/livox/lidar`、`/odom`、`/camera/color/image_raw` topic 可达；
- 项目内 IP 与清单一致（grep 无漂移值）。

---

## 5. 开放问题 / 待确认

1. 清单放部署副本（Thor `~/Desktop/…`）还是开发源（WSL `/mnt/d/robonix/…`）？是否需要双写？
2. USB 网卡 MAC 前缀实测值 = `ec:9a:0c`（从 enxec9a0c10fd63 推断），apply 前复核 `ip link`；
3. 狗侧 `network.toml` 直改 vs DNAT 落盘，用户选哪个？
4. 是否支持"狗↔Thor 走 WiFi"（狗热点 p2p0 / wlan0）作为有线备援？Livox 雷达只能有线，该链路不动；
5. RMW / ROS_DOMAIN_ID 是否进清单？（目前固定 rmw_zenoh_cpp / 0，与容器配置一致即可，暂不进）。

---

## 附：实测数据快照（2026-08-07）

- Thor：`enP2p1s0=.130/24`（NM `lite3-link`）、`enxec9a0c10fd63=.131/24`（NM `Wired connection 6`）、`wlP1p1s0=.29/24`（`caiwu-5g` DHCP）、`tailscale0=100.72.167.58`
- 策略路由：`ip rule` pref 100 → `.120/.168 lookup lidar`；`/etc/iproute2/rt_tables` 有 `100 lidar`
- 雷达：`MID360s_config.json` ip=.168、host_ip=.131、端口 56100–56500
- 狗：`transfer_launch.py` target .120:43893 / local 43897；`driver.py` DEFAULT_ROBOT_IP=.120；RK3588 `network.toml` local 43893 / target 43897 / ip=上位机（当前 DNAT→.130）
- 相机：Orbbec Gemini 335（2bc5:0800）USB 直连 Thor，`compose.yaml` devices 透传 /dev/bus/usb
