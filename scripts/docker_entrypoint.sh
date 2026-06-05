#!/bin/bash
# Docker entrypoint script for SLAM system
# Sets up ROS environment and network configuration

set -e

echo "================================================"
echo "SLAM System Container Starting"
echo "================================================"

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash

# Set ROS network configuration
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

echo "ROS Configuration:"
echo "  ROS_HOSTNAME: $ROS_HOSTNAME"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  HESAI_IP: ${HESAI_IP:-192.168.1.201}"
echo ""

# Check network connectivity to the Hesai JT128 sensor
if [ ! -z "${HESAI_IP}" ]; then
    echo "Checking Hesai JT128 sensor connectivity..."
    if ping -c 1 -W 2 ${HESAI_IP} &>/dev/null; then
        echo "✓ Hesai sensor reachable at ${HESAI_IP}"
    else
        echo "⚠ Warning: Cannot reach Hesai sensor at ${HESAI_IP}"
        echo "  Make sure sensor is powered and connected to same network"
    fi
    echo ""
fi

# Display available ROS packages
echo "Available SLAM packages:"
if [ -d /root/slam_ws/devel/lib/fast_lio ]; then
    echo "  ✓ FAST-LIO (odometry)"
else
    echo "  ✗ FAST-LIO not found"
fi

if [ -d /root/slam_ws/devel/lib/aloam_velodyne ]; then
    echo "  ✓ SC-PGO (loop closure)"
else
    echo "  ✗ SC-PGO not found"
fi

if [ -d /root/slam_ws/devel/lib/hesai_ros_driver ]; then
    echo "  ✓ Hesai ROS driver"
else
    echo "  ✗ Hesai ROS driver not found"
fi
echo ""

echo "================================================"
echo "Container ready. Starting requested command..."
echo "================================================"
echo ""

# Execute the main command
exec "$@"
