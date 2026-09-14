#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
# the micro-ROS agent lives in its own workspace, built by micro_ros_setup
source /uros_ws/install/local_setup.bash
source /ws/install/setup.bash
exec "$@"
