# 02 · 快速开始

## 1. 环境检查

```bash
ls /opt/ros/humble/setup.bash
ip -br link
ls -l /dev/rc_receiver
```

依赖：

- ROS 2 Humble
- `colcon`、`rclcpp`、`geometry_msgs`、`std_msgs`
- 系统库：`libpcap`
- FS-i6S 接收机 iBus 串口，默认路径 `/dev/rc_receiver`
- 独立 EtherCAT 网卡，默认 `enp86s0`

## 2. 编译

```bash
cd /home/<你>/code/R1-Ethercat_4c
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 3. 启动全系统

推荐使用根目录脚本：

```bash
cd /home/<你>/code/R1-Ethercat_4c
./start_full_system.sh --ifname enp86s0
```

脚本会自动编译、source 工作空间、拉起网卡，并给主控二进制补 `cap_net_raw,cap_net_admin`。涉及 `ip link set` 或 `setcap` 时会调用 `sudo`。

启动后默认节点：

| 节点 | 作用 |
| --- | --- |
| `remote_node` | 读取 FS-i6S iBus 串口，发布 `/cmd_vel`、`/robot_buttons`、`/remote_vra` |
| `vehicle_control` | EtherCAT 主站 + LinkX/CAN + 当前全向轮底盘控制 |

## 4. 启动参数

根目录 `start_full_system.sh` 当前只接受：

| 参数 | 用途 |
| --- | --- |
| `--ifname <name>` | 指定 EtherCAT 网卡，默认 `enp86s0` |
| `-h` / `--help` | 打印帮助 |

`--sudo`、`--auto-ifname`、`--max-speed` 不是当前根脚本参数。

## 5. 只测遥控链路

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch linkx_bringup teleop.launch.py serial_port:=/dev/rc_receiver
```

检查输出：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_buttons
ros2 topic echo /remote_vra
```

## 6. 全系统 launch 但不启动车体

```bash
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 \
  serial_port:=/dev/rc_receiver \
  start_vehicle_control:=false
```

## 7. 优雅退出

`Ctrl+C` 会触发主控信号处理，`vehicle_control` 将退出 EtherCAT 主循环并关闭主站。
