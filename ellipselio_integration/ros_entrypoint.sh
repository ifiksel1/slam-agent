#!/bin/bash
set -e

# ROS 2 base
source "/opt/ros/humble/setup.bash"

# Overlay the workspace if it has been built (mounted from host at /root/ros2_ws)
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source "/root/ros2_ws/install/setup.bash"
fi

echo "============== EllipseLIO ROS2 (Humble) Docker Env Ready =============="

cd /root/ros2_ws

exec "$@"
