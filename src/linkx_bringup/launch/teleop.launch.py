from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/rc_receiver'),

        # FS-i6S iBus 解算节点 (串口 -> /cmd_vel + /robot_buttons)
        # 仅起遥控通路, 不带整车主控, 便于单独调试遥控器。
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            parameters=[{'serial_port': serial_port}],
            output='screen',
        ),
    ])
