"""Start everything: micro-ROS agent, navigation, safety watchdog and the web UI.

    ros2 launch gps_localize bringup.launch.py

Useful arguments:
    transport:=udp4|serial     how the ESP32 is connected (default udp4, Wi-Fi)
    agent_port:=8888           UDP port of the micro-ROS agent
    serial_device:=/dev/ttyUSB0
    domain_id:=10              must match ROS_DOMAIN_ID in firmware/config/conf_network.h
    start_agent:=false         if you already run the agent somewhere else
    runtime_config_dir:=~/.gps_localize
"""

import os
import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def find_standalone_agent():
    """Path to the eProsima MicroXRCEAgent binary, or None.

    Ubuntu 22.04 / Humble has no ros-humble-micro-ros-agent package, so the agent is
    normally built from source. That produces a plain MicroXRCEAgent executable rather
    than a ROS package, which cannot be started with Node().
    """
    found = shutil.which('MicroXRCEAgent')
    if found:
        return found
    local = os.path.expanduser('~/.local/bin/MicroXRCEAgent')
    return local if os.path.isfile(local) and os.access(local, os.X_OK) else None


def have_ros_agent_package():
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return False
    try:
        get_package_share_directory('micro_ros_agent')
        return True
    except (PackageNotFoundError, Exception):
        return False


def generate_launch_description():
    transport = LaunchConfiguration('transport')
    agent_port = LaunchConfiguration('agent_port')
    serial_device = LaunchConfiguration('serial_device')
    domain_id = LaunchConfiguration('domain_id')
    start_agent = LaunchConfiguration('start_agent')
    runtime_config_dir = LaunchConfiguration('runtime_config_dir')

    args = [
        DeclareLaunchArgument('transport', default_value='udp4'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('domain_id', default_value='10'),
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('runtime_config_dir', default_value='~/.gps_localize'),
    ]

    params = [{'runtime_config_dir': runtime_config_dir}]

    udp_when = IfCondition(PythonExpression(
        ["'", start_agent, "' == 'true' and '", transport, "' == 'udp4'"]))
    serial_when = IfCondition(PythonExpression(
        ["'", start_agent, "' == 'true' and '", transport, "' == 'serial'"]))

    standalone_agent = None if have_ros_agent_package() else find_standalone_agent()

    if standalone_agent:
        # Source-built agent: a plain executable, so ExecuteProcess rather than Node.
        # It takes -p / -D where the ROS wrapper takes --port / --dev.
        agent_udp = ExecuteProcess(
            cmd=[standalone_agent, 'udp4', '-p', agent_port],
            name='micro_ros_agent', output='screen', respawn=True, respawn_delay=2.0,
            condition=udp_when,
        )
        agent_serial = ExecuteProcess(
            cmd=[standalone_agent, 'serial', '-D', serial_device, '-b', '115200'],
            name='micro_ros_agent', output='screen', respawn=True, respawn_delay=2.0,
            condition=serial_when,
        )
    else:
        agent_udp = Node(
            package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
            arguments=['udp4', '--port', agent_port],
            output='screen', condition=udp_when,
        )
        agent_serial = Node(
            package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
            arguments=['serial', '--dev', serial_device, '-b', '115200'],
            output='screen', condition=serial_when,
        )

    nodes = [
        # Turns the board's one compact frame into six typed topics. Runs
        # here rather than on the ESP32 so the Wi-Fi link carries a single
        # message per cycle instead of six.
        Node(package='gps_localize', executable='telemetry_split', name='telemetry_split',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
        Node(package='gps_localize', executable='safety_watchdog', name='safety_watchdog',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
        Node(package='gps_localize', executable='gps_odom', name='gps_odom',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
        Node(package='gps_localize', executable='waypoint_nav', name='waypoint_nav',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
        Node(package='gps_localize', executable='web_server', name='web_server',
             parameters=params, output='screen', respawn=True, respawn_delay=2.0),
    ]

    return LaunchDescription(
        args
        + [SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id), agent_udp, agent_serial]
        + nodes
    )
