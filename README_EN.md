# R1 EtherCAT-4C Upper-Computer Control Stack

[简体中文](README.md) | **English**

> ROS 2 Humble + SOEM + LinkX / EtherCAT-4C upper-computer chassis and mechanism control stack for the R1 robot.
>
> **Consolidation Note**: This repository is the unified primary repo for the R1 EtherCAT series, superseding the legacy archived repositories `R1-Ethercat-4c` (F710 + steering chassis) and `R1-Ethercat-FSreomte-4c` (FS-i6S + steering chassis). It uses a clean 5-layer architecture (`1_Middleware` through `5_Task`) and retains both the active omni-wheel chassis and legacy steering-wheel implementations.

## Overview

- **Active Chassis**: X-configuration 4WD omni-wheel chassis (`omni_wheel`), driving 4 × DJI M3508 / C620 motors via LinkX-4C CAN and reading OPS-9 planar odometry.
- **Active Teleop**: FlySky FS-i6S serial iBus receiver (`/dev/rc_receiver`, 115200 8N1). Logitech F710 driver sources (`LogF710/`) are also kept for reference.
- **Legacy Steering Chassis**: The 4-wheel steering implementation (`steer_wheel`), BRT encoder virtual multi-turn persistence, DM steer motor / ODrive calibration, and diagnostic utilities remain available for debugging and regression testing.

## Runtime Topology

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

Default nodes launched by `full_system.launch.py`:

| Node | Executable | Role |
| --- | --- | --- |
| `remote_node` | `linkx_soem_demo/remote_node_cpp` | Parses FS-i6S iBus serial stream and publishes `/cmd_vel`, `/robot_buttons`, `/remote_vra` |
| `vehicle_control` | `linkx_soem_demo/linkx_soem_demo` | SOEM EtherCAT master, LinkX-4C CAN bridge, chassis control, and hardware feedback |

## Directory Structure

```text
R1-Ethercat_4c/
├── README.md                    # Chinese primary README
├── README_EN.md                 # English README (this file)
├── start_full_system.sh         # One-click build, setcap, and launch script
├── src/
│   ├── linkx_soem_demo/         # Main package: EtherCAT, LinkX/CAN, chassis, remote, diagnostics
│   │   ├── include/linkx_soem_demo/
│   │   ├── src/remote/          # FSI6S (active) and LogF710 drivers
│   │   ├── src/vehicle_control/ # 1_Middleware / 2_Device / 3_Chariot / 4_Interaction / 5_Task
│   │   └── src/test_mains/      # Standalone hardware diagnostic & calibration executables
│   └── linkx_bringup/           # ROS 2 launch files (full_system / teleop)
├── tools/                       # Analysis scripts and trace plotting utilities
├── instructions/                # Historical engineering notes
└── readme/                      # Detailed Chinese sub-documents (01 ~ 08)
```

## Prerequisites

- Ubuntu 22.04 + **ROS 2 Humble** (`/opt/ros/humble/setup.bash`)
- `colcon`, `cmake`, `libpcap-dev`, and standard ROS 2 C++ dependencies
- A **dedicated wired Ethernet interface** for EtherCAT (example default: `enp86s0`)
- FS-i6S receiver serial port (default udev symlink: `/dev/rc_receiver`)

## Quick Start

### One-Click Build & Launch (Recommended)

```bash
./start_full_system.sh --ifname enp86s0
```

This script automatically:
1. Prepares `var_data/live_variables.log`.
2. Sources `/opt/ros/humble/setup.bash` and brings up the specified EtherCAT NIC.
3. Builds the workspace with `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`.
4. Grants `cap_net_raw,cap_net_admin=eip` to the compiled main executable.
5. Launches `ros2 launch linkx_bringup full_system.launch.py ifname:=<ifname>`.

### Manual Commands

```bash
# Build
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Test FS-i6S teleop only
ros2 launch linkx_bringup teleop.launch.py serial_port:=/dev/rc_receiver

# Launch full stack without EtherCAT vehicle control
ros2 launch linkx_bringup full_system.launch.py \
  ifname:=enp86s0 \
  start_vehicle_control:=false \
  serial_port:=/dev/rc_receiver
```

## Detailed Documentation (Chinese)

| Document | Contents |
| --- | --- |
| [readme/01_structure.md](readme/01_structure.md) | Workspace layout, packages, and module responsibilities |
| [readme/02_quickstart.md](readme/02_quickstart.md) | Minimal environment check, build, and launch steps |
| [readme/03_run_guide.md](readme/03_run_guide.md) | Full run guide, launch arguments, and permissions |
| [readme/04_steer_encoder.md](readme/04_steer_encoder.md) | Steering encoder virtual multi-turn & persistence |
| [readme/05_calibration.md](readme/05_calibration.md) | Steering / motor / chassis calibration procedures |
| [readme/06_test_executables.md](readme/06_test_executables.md) | Standalone diagnostic and test executables |
| [readme/07_tools.md](readme/07_tools.md) | Debugging scripts and plotting tools |
| [readme/08_troubleshooting.md](readme/08_troubleshooting.md) | Common troubleshooting tips |

## Safety Notes

- Ensure the robot is lifted off the ground or protected by a working emergency stop before enabling `vehicle_control`.
- Raw Ethernet access requires `cap_net_raw` / `cap_net_admin` capabilities, which must be re-applied after rebuilding (`start_full_system.sh` handles this automatically).
