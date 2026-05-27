from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ifname = LaunchConfiguration('ifname')
    start_vehicle_control = LaunchConfiguration('start_vehicle_control')
    vehicle_prefix = LaunchConfiguration('vehicle_prefix')
    ros_nodes_prefix = LaunchConfiguration('ros_nodes_prefix')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('ifname', default_value='enp86s0'),
        DeclareLaunchArgument('start_vehicle_control', default_value='true'),
        DeclareLaunchArgument('vehicle_prefix', default_value=''),
        DeclareLaunchArgument('ros_nodes_prefix', default_value=''),
        # FS-i6S 接收机 iBus -> USB-UART, udev 软链常驻路径
        DeclareLaunchArgument('serial_port', default_value='/dev/rc_receiver'),

        # 1) FS-i6S iBus 解算节点（直接读串口, 发布 /cmd_vel + /robot_buttons）
        #    max_speed/max_omega/deadzone 默认值在 remote_node.cpp 的 declare_parameter 里;
        #    serial_port 用 launch 参数覆盖。joy_node 已废除——iBus 不走 /joy。
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            prefix=ros_nodes_prefix,
            parameters=[{'serial_port': serial_port}],
            output='screen',
        ),

        # 2) 整车主控程序（可选）
        Node(
            package='linkx_soem_demo',
            executable='linkx_soem_demo',
            name='vehicle_control',
            output='screen',
            condition=IfCondition(start_vehicle_control),
            prefix=vehicle_prefix,
            arguments=[ifname],
        ),
    ])
