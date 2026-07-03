#!/usr/bin/env bash
# Adaptive-LIO container entrypoint. Sources ROS + the catkin workspace, then execs the command.
set -e
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
if [ -f /root/catkin_ws/devel/setup.bash ]; then
  source /root/catkin_ws/devel/setup.bash
fi
exec "$@"
