# 03 · 完整运行指南

## 1. 推荐入口：`start_full_system.sh`

```bash
cd /home/<你>/code/R1-Ethercat_4c
./start_full_system.sh --ifname enp86s0
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--ifname <name>` | `enp86s0` | EtherCAT 网卡名 |
| `-h` / `--help` | - | 打印帮助 |

根目录脚本当前不接受 `--sudo`、`--auto-ifname`、`--max-speed`。如果需要只启动遥控链路或关闭车体主控，直接使用 `linkx_bringup` 的 launch 参数。

脚本流程：

1. 创建 `var_data/`，清空 `var_data/live_variables.log`，导出运行时数据路径。
2. 检查 `/opt/ros/humble/setup.bash`。
3. 检查 EtherCAT 网卡；状态不是 `UP` 时执行 `sudo ip link set <ifname> up`。
4. `source /opt/ros/humble/setup.bash`。
5. `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`。
6. `source install/setup.bash`。
7. 对真实主控二进制执行 `sudo setcap cap_net_raw,cap_net_admin=eip`。
8. `exec ros2 launch linkx_bringup full_system.launch.py ifname:=<ifname>`。

## 2. 直接 launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 \
  serial_port:=/dev/rc_receiver \
  start_vehicle_control:=true
```

`full_system.launch.py` 参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `ifname` | `enp86s0` | EtherCAT 网卡名，作为 `vehicle_control` 第一个命令行参数 |
| `serial_port` | `/dev/rc_receiver` | FS-i6S iBus 串口 |
| `start_vehicle_control` | `true` | 是否启动车体主控；`false` 时只跑遥控链路 |
| `vehicle_prefix` | `""` | 给车体主控加命令前缀 |
| `ros_nodes_prefix` | `""` | 给 ROS 节点加命令前缀 |

## 3. 只启动遥控链路

```bash
ros2 launch linkx_bringup teleop.launch.py serial_port:=/dev/rc_receiver
```

或使用全系统 launch 关闭车体：

```bash
ros2 launch linkx_bringup full_system.launch.py \
  start_vehicle_control:=false \
  serial_port:=/dev/rc_receiver
```

可用以下命令检查：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_buttons
ros2 topic echo /remote_vra
```

## 4. 权限模型

EtherCAT 原始以太网需要 `cap_net_raw` 和 `cap_net_admin`。推荐使用根目录脚本，它会在每次构建后自动对真实二进制补 capability：

```bash
./start_full_system.sh --ifname enp86s0
```

手动授权：

```bash
sudo setcap 'cap_net_raw,cap_net_admin+ep' \
  install/linkx_soem_demo/lib/linkx_soem_demo/linkx_soem_demo
```

注意：`colcon build` 重新安装可执行文件后 capability 可能丢失。

## 5. 启动后的节点

| 节点 | 包 / 可执行 | 角色 |
| --- | --- | --- |
| `remote_node` | `linkx_soem_demo/remote_node_cpp` | FS-i6S iBus 串口 → `/cmd_vel`、`/robot_buttons`、`/remote_vra` |
| `vehicle_control` | `linkx_soem_demo/linkx_soem_demo` | EtherCAT 主站、LinkX/CAN、全向轮底盘控制 |

当前不使用 `joy_node`，iBus 遥控链路不经过 `/joy`。

## 6. 遥控参数

`remote_node_cpp` 默认参数在 `src/linkx_soem_demo/src/remote/FSI6S/remote_node.cpp` 中声明：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `max_speed` | `0.5` | 最大平移速度 |
| `max_omega` | `0.5` | 最大角速度 |
| `deadzone` | `0.03` | 摇杆死区 |
| `serial_port` | `/dev/rc_receiver` | iBus 串口路径 |
| `rx_timeout_ms` | `200` | 超时后发布零速和零按键 |

运行时临时调整示例：

```bash
ros2 param set /remote_node max_speed 0.3
ros2 param set /remote_node max_omega 0.4
```

## 7. 实时观察反馈

仓库自带 `tools/plot_feedback_wave.py`，可从控制台日志中提取 `key=value` 波形：

```bash
./start_full_system.sh --ifname enp86s0 2>&1 | \
python3 tools/plot_feedback_wave.py \
  --series now_vx --series now_vy --series now_omega \
  --height 18 --history 600 --refresh-hz 10
```

也可分两个终端：

```bash
# 终端 1
./start_full_system.sh --ifname enp86s0 2>&1 | tee /tmp/r1_live.log

# 终端 2
tail -f /tmp/r1_live.log | python3 tools/plot_feedback_wave.py --series now_vx
```

## 8. 当前主控底盘配置

主可执行 `linkx_soem_demo` 当前在 CMake 中链接：

- `src/vehicle_control/3_Chariot/chassis/omni_wheel/crt_chassis.cpp`
- `src/vehicle_control/2_Device/Motor/dvc_motor_dji.cpp`
- `src/remote/FSI6S/dvc_fsi6s.cpp`
- `src/remote/FSI6S/ibus_parser.cpp`

CAN 通道约定：

| 通道 | 用途 |
| --- | --- |
| ch0 | 4 × DJI 3508 / C620，反馈 ID `0x201..0x204`，控制 ID `0x200` |
| ch1 / ch2 | 当前全向轮主路径闲置；舵轮测试程序可能使用 |
| ch3 | OPS-9 |

## 9. 安全注意事项

- EtherCAT 网卡建议独立专用，不要混跑普通网络业务。
- 启动车体主控前确认车体架空或现场具备急停条件。
- `/dev/rc_receiver` 丢帧超过 `rx_timeout_ms` 后，遥控节点会发布一次零速和零按键。
- `build/`、`install/`、`log/` 是自动产物，构建异常时可清理后重编。
- `src/linkx_soem_demo/launch/run_link.sh` 是旧兼容脚本，可能与当前 launch 参数不同步；日常运行以根目录脚本为准。
