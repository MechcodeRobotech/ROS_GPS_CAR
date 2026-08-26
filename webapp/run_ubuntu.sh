#!/usr/bin/env bash
# GPS_Localize Viewer - Ubuntu/Linux launcher (VIEW ONLY, cannot control the robot).
#
#   ./run_ubuntu.sh                  view on this machine only
#   ./run_ubuntu.sh --host 0.0.0.0   let other devices open the demo
#
# On the real robot PC do NOT use this script - run the ROS stack instead:
#   ros2 launch gps_localize bringup.launch.py
set -e
cd "$(dirname "$0")"

if [ -x "bin/GPS_Localize_Viewer" ]; then
    exec ./bin/GPS_Localize_Viewer "$@"
fi

if command -v python3 >/dev/null 2>&1; then
    exec python3 viewer_server.py "$@"
fi

echo "python3 not found:  sudo apt install python3 python3-yaml"
exit 1
