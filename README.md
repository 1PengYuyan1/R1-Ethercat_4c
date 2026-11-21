# R1 EtherCAT-4C 上位机控制工程

**简体中文** | [English](README_EN.md)

> 基于 **ROS 2 Humble + SOEM + LinkX / EtherCAT-4C** 的 R1 机器人上位机底盘与机构控制工程。
>
> **整合说明**：本仓库已作为 R1 EtherCAT 系列唯一主仓库，整合了早期分散的 `R1-Ethercat-4c`（F710 + 舵轮）与 `R1-Ethercat-FSreomte-4c`（FS-i6S + 舵轮），并重构为五层架构（`1_Middleware` ~ `5_Task`），同时包含全向轮与历史舵轮代码。

## 项目概览

- **主用底盘方案**：X 型四驱全向轮底盘（`omni_wheel`），经 LinkX-4C CAN 控制 4 个 DJI M3508 / C620 驱动轮，并接入 OPS-9 平面定位里程计。
- **主用遥控方案**：FlySky FS-i6S iBus 串口遥控器（`/dev/rc_receiver`，115200 8N1），同时保留 Logitech F710 驱动代码（`LogF710/`）以供参考。
- **保留的历史舵轮方案**：四舵轮代码（`steer_wheel`）、BRT 编码器虚拟无限位移与掉电恢复、DM 舵向电机 / ODrive 标定及测试工具仍完整保留，便于调试或回归对比。

## 运行拓扑

```text
FS-i6S Receiver (/dev/rc_receiver, iBus 115200 8N1)
        ↓
remote_node_cpp
        ├─ /cmd_vel
        ├─ /robot_buttons
        └─ /remote_vra
        ↓
vehicle_control (linkx_soem_demo)
        ├─ SOEM EtherCAT Master
        ├─ LinkX / EtherCAT-4C Gateway
        ├─ CAN ch0: 4 × DJI 3508 / C620
        └─ CAN ch3: OPS-9 Odometry
```

`full_system.launch.py` 默认启动两个核心节点：

| 节点 | 可执行文件 | 功能 |
| --- | --- | --- |
| `remote_node` | `linkx_soem_demo/remote_node_cpp` | 解析 FS-i6S iBus 串口，发布 `/cmd_vel`、`/robot_buttons`、`/remote_vra` |
| `vehicle_control` | `linkx_soem_demo/linkx_soem_demo` | EtherCAT 主站、LinkX-4C CAN 通信、底盘控制与硬件反馈处理 |

## 工程目录结构

```text
R1-Ethercat_4c/
├── README.md                    # 中文主文档（本文件）
├── README_EN.md                 # English README
├── start_full_system.sh         # 一键编译、授权并启动脚本
├── src/
│   ├── linkx_soem_demo/         # 主控包：EtherCAT、LinkX/CAN、底盘、遥控与诊断工具
│   │   ├── include/linkx_soem_demo/
│   │   ├── src/remote/          # FSI6S（当前启用）与 LogF710 遥控驱动
│   │   ├── src/vehicle_control/ # 1_Middleware / 2_Device / 3_Chariot / 4_Interaction / 5_Task
│   │   └── src/test_mains/      # 独立硬件诊断、标定与测试可执行程序
│   └── linkx_bringup/           # ROS 2 launch 启动编排（full_system / teleop）
├── tools/                       # 调试脚本、轨迹曲线绘制与分析工具
├── instructions/                # 历史实施记录
└── readme/                      # 详细子文档（01 ~ 08）
```

## 环境要求

- Ubuntu 22.04 + **ROS 2 Humble**（`/opt/ros/humble/setup.bash`）
- `colcon`、`cmake`、`libpcap-dev` 及 ROS 2 C++ 编译依赖
- 一张**独立专用有线网卡**直连 LinkX-4C（默认示例：`enp86s0`）
- FS-i6S 接收机串口可用（默认 udev 软链：`/dev/rc_receiver`）

## 快速开始

### 一键编译与启动（推荐）

```bash
./start_full_system.sh --ifname enp86s0
```

脚本会自动执行：
1. 创建并清空 `var_data/live_variables.log`。
2. 加载 `/opt/ros/humble/setup.bash` 并拉起指定的 EtherCAT 网卡。
3. 执行 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`。
4. 为主控二进制自动设置 `cap_net_raw,cap_net_admin=eip` 权限。
5. 启动 `ros2 launch linkx_bringup full_system.launch.py ifname:=<ifname>`。

### 常用分步命令

```bash
# 手动编译
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 仅测试 FS-i6S 遥控链路
ros2 launch linkx_bringup teleop.launch.py serial_port:=/dev/rc_receiver

# 启动完整 launch 但不拉起车体 EtherCAT 主控
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 \
  start_vehicle_control:=false \
  serial_port:=/dev/rc_receiver
```

## 详细文档导航

| 文档 | 内容 |
| --- | --- |
| [readme/README.md](readme/README.md) | 详细中文总览与运行前检查 |
| [readme/01_structure.md](readme/01_structure.md) | 工作区、包结构与模块职责 |
| [readme/02_quickstart.md](readme/02_quickstart.md) | 最小可执行步骤：环境检查、编译、启动 |
| [readme/03_run_guide.md](readme/03_run_guide.md) | 完整运行指南、launch 参数、权限模型 |
| [readme/04_steer_encoder.md](readme/04_steer_encoder.md) | 舵轮方案编码器虚拟无限位移与掉电恢复 |
| [readme/05_calibration.md](readme/05_calibration.md) | 舵轮 / 电机 / 底盘标定流程 |
| [readme/06_test_executables.md](readme/06_test_executables.md) | 独立测试与诊断可执行程序说明 |
| [readme/07_tools.md](readme/07_tools.md) | 调试脚本、曲线绘制与分析工具 |
| [readme/08_troubleshooting.md](readme/08_troubleshooting.md) | 常见故障排查 |

## 注意事项

- 现场启动主控前请确认车体已离地架起或具备有效急停条件。
- EtherCAT 原始以太网收发需要 `cap_net_raw` / `cap_net_admin` 权限，每次重新编译后需重新授权（`start_full_system.sh` 会自动处理）。
- `readme/04_steer_encoder.md`、`readme/05_calibration.md` 主要面向历史舵轮方案；使用当前全向轮主路径时请优先查阅 `02_quickstart.md` 与 `03_run_guide.md`。
