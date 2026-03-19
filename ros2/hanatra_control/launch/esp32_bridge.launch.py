#!/usr/bin/env python3
"""esp32_bridge.launch.py — Start the ESP32 UART bridge node"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_port',
            default_value='/dev/ttyUSB0',
            description='UART port for ESP32 connection'
        ),
        DeclareLaunchArgument(
            'uart_baud',
            default_value='115200',
            description='UART baud rate'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='hanatra',
            description='Robot namespace'
        ),

        Node(
            package='hanatra_control',
            executable='esp32_bridge_node',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'uart_baud': LaunchConfiguration('uart_baud'),
                'namespace': LaunchConfiguration('namespace'),
            }],
            remappings=[
                ('~/cmd_vel', '/hanatra/cmd_vel'),
                ('~/odom', '/hanatra/odom'),
                ('~/imu/data', '/hanatra/imu/data'),
                ('~/battery', '/hanatra/battery'),
                ('~/estop', '/hanatra/estop'),
                ('~/motor_status', '/hanatra/motor_status'),
            ]
        ),
    ])
