"""Web UI + safety watchdog only (no agent, no navigation).

Handy on a desk: you can open the UI, look at the pages and edit settings
without any hardware connected.

    ros2 launch gps_localize web_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    runtime_config_dir = LaunchConfiguration('runtime_config_dir')
    params = [{'runtime_config_dir': runtime_config_dir}]

    return LaunchDescription([
        DeclareLaunchArgument('runtime_config_dir', default_value='~/.gps_localize'),
        Node(package='gps_localize', executable='web_server', name='web_server',
             parameters=params, output='screen'),
        Node(package='gps_localize', executable='safety_watchdog', name='safety_watchdog',
             parameters=params, output='screen'),
    ])
