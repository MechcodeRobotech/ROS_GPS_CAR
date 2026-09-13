# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Web UI + safety watchdog only (no agent, no navigation).

Handy on a desk: you can open the UI, look at the pages and edit settings
without any hardware connected.

    ros2 launch gps_localize web_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    runtime_config_dir = LaunchConfiguration('runtime_config_dir')
    params = [{'runtime_config_dir': runtime_config_dir}]

    open_browser = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '-c', 'import webbrowser; webbrowser.open("http://localhost:8080")'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        open_browser,
        DeclareLaunchArgument('runtime_config_dir', default_value='~/.gps_localize'),
        Node(package='gps_localize', executable='web_server', name='web_server',
             parameters=params, output='screen'),
        # Turns the board's one compact frame into six typed topics. Runs
        # here rather than on the ESP32 so the Wi-Fi link carries a single
        # message per cycle instead of six.
        Node(package='gps_localize', executable='telemetry_split', name='telemetry_split',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
        Node(package='gps_localize', executable='safety_watchdog', name='safety_watchdog',
             parameters=params, output='screen'),
    ])
