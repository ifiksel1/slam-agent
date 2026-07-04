#!/usr/bin/env bash
# Convenience wrapper: sources ROS + the shared master, then runs the live SLAM metrics reader.
# Usage: ./slam_metrics.sh [--topic /odom] [--csv out.csv] [--raw]
set +u
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_IP=${ROS_IP:-192.168.2.50}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$HERE/slam_metrics.py" "$@"
