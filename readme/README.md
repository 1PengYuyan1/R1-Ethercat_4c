# R1 EtherCAT-4C

> 基于 ROS 2 Humble、SOEM 与 LinkX / EtherCAT-4C 的 R1 底盘控制工程。

当前主程序配置为 **X 型四驱全向轮底盘**：FS-i6S iBus 遥控器经串口发布 ROS 2 指令，`vehicle_control` 通过 EtherCAT 主站驱动 LinkX / EtherCAT-4C 网关，再经 CAN 控制 4 个 DJI 3508 / C620 驱动轮，并接入 OPS-9 里程计。

旧的四舵轮代码、舵向编码器持久化、DM 舵向电机标定与相关测试程序仍保留在仓库中，主要用于调试、回归或切回舵轮方案；当前一键启动路径以 `omni_wheel` 实现为准。

## 文档导航

| 文件 | 内容 |
| --- | --- |
| [01_structure.md](01_structure.md) | 工作区、包结构与模块职责 |
| [02_quickstart.md](02_quickstart.md) | 最小可执行步骤：环境检查、编译、启动 |
| [03_run_guide.md](03_run_guide.md) | 完整运行指南、launch 参数、权限模型 |
| [04_steer_encoder.md](04_steer_encoder.md) | 舵轮方案的编码器虚拟无限位移与掉电恢复 |
| [05_calibration.md](05_calibration.md) | 舵轮 / 电机 / 底盘标定流程 |
| [06_test_executables.md](06_test_executables.md) | 测试与诊断可执行程序 |
| [07_tools.md](07_tools.md) | 调试脚本、曲线绘制与分析工具 |
| [08_troubleshooting.md](08_troubleshooting.md) | 常见故障排查 |

## 快速开始

```bash
cd /home/<你>/code/R1-Ethercat_4c
./start_full_system.sh --ifname enp86s0
```

脚本会完成：

1. 创建并清空 `var_data/live_variables.log`，导出 `VAR_DATA_DIR` / `VAR_DATA_FILE`。
2. 检查 `/opt/ros/humble/setup.bash` 并 source ROS 2 Humble。
3. 检查 EtherCAT 网卡，必要时通过 `sudo ip link set <ifname> up` 拉起。
4. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`。
5. 给主控真实二进制自动补 `cap_net_raw,cap_net_admin=eip`。
6. 启动 `ros2 launch linkx_bringup full_system.launch.py ifname:=<ifname>`。

## 运行拓扑

```text
FS-i6S receiver (/dev/rc_receiver, iBus 115200 8N1)
        ↓
remote_node_cpp
        ├─ /cmd_vel
        ├─ /robot_buttons
        └─ /remote_vra
        ↓
vehicle_control (linkx_soem_demo)
        ├─ SOEM EtherCAT master
        ├─ LinkX / EtherCAT-4C
        ├─ CAN ch0: 4 × DJI 3508 / C620
        └─ CAN ch3: OPS-9
```

`full_system.launch.py` 默认启动两个节点：

| 节点 | 可执行 | 作用 |
| --- | --- | --- |
| `remote_node` | `linkx_soem_demo/remote_node_cpp` | 读取 FS-i6S iBus 串口，发布 `/cmd_vel`、`/robot_buttons`、`/remote_vra` |
| `vehicle_control` | `linkx_soem_demo/linkx_soem_demo` | EtherCAT 主站、底盘控制、硬件反馈处理 |

## 工程一览

```text
R1-Ethercat_4c/
├── src/
│   ├── linkx_soem_demo/           主控包：EtherCAT、LinkX/CAN、底盘控制、遥控节点
│   │   ├── src/remote/FSI6S/      FS-i6S iBus 串口遥控链路
│   │   ├── src/vehicle_control/   控制主循环、设备层、底盘与任务调度
│   │   └── src/test_mains/        独立硬件诊断 / 标定程序
│   └── linkx_bringup/             launch 与启动编排
├── tools/                         调试脚本与数据分析工具
├── instructions/                  历史实施记录
├── readme/                        项目文档
├── start_full_system.sh           当前推荐的一键启动入口
└── build/ install/ log/           colcon 自动生成目录
```

## 关键文件

| 路径 | 说明 |
| --- | --- |
| `start_full_system.sh` | 推荐启动脚本，仅接受 `--ifname <name>` |
| `src/linkx_bringup/launch/full_system.launch.py` | 全系统 launch，参数包含 `ifname`、`serial_port`、`start_vehicle_control` |
| `src/linkx_bringup/launch/teleop.launch.py` | 只启动 FS-i6S 遥控链路 |
| `src/linkx_soem_demo/src/remote/FSI6S/remote_node.cpp` | iBus 串口读取、遥控参数默认值、失联安全态 |
| `src/linkx_soem_demo/include/linkx_soem_demo/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis.h` | 当前主控使用的全向轮底盘接口 |
| `src/linkx_soem_demo/src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis.cpp` | 当前主控使用的全向轮运动学与 DJI 电流控制 |

## 运行前检查

- ROS 2 Humble 已安装：`ls /opt/ros/humble/setup.bash`。
- EtherCAT 网卡独立专用，常用默认名为 `enp86s0`。
- FS-i6S 接收机串口可用，默认 udev 软链为 `/dev/rc_receiver`。
- `libpcap`、`colcon` 与 ROS 2 C++ 依赖已安装。
- 现场确认车体离地或具备安全急停条件后再启动车体主控。

## 常用命令

```bash
# 编译
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 启动全系统
./start_full_system.sh --ifname enp86s0

# 只测遥控链路
source install/setup.bash
ros2 launch linkx_bringup teleop.launch.py serial_port:=/dev/rc_receiver

# 全系统 launch，但不启动车体主控
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 \
  start_vehicle_control:=false \
  serial_port:=/dev/rc_receiver
```

## 注意事项

1. 根目录 `start_full_system.sh` 当前不再接受 `--sudo`、`--auto-ifname` 或 `--max-speed`。
2. EtherCAT 原始以太网需要 `cap_net_raw` / `cap_net_admin`；启动脚本会在构建后自动对真实二进制执行 `setcap`，可能触发 sudo 密码输入。
3. 遥控最大速度、角速度、死区默认值位于 `src/linkx_soem_demo/src/remote/FSI6S/remote_node.cpp` 的 `declare_parameter`；运行时可用 `ros2 param set /remote_node <param> <value>` 临时调整。
4. `src/linkx_soem_demo/launch/run_link.sh` 是旧兼容入口，当前文档以根目录 `start_full_system.sh` 和 `linkx_bringup` launch 文件为准。
5. `readme/04_steer_encoder.md`、`readme/05_calibration.md` 中大量内容针对舵轮方案；调试当前全向轮主路径时，优先看本文件、快速开始和运行指南。
