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
echo "  OUSTER_IP: ${OUSTER_IP:-169.254.56.220}"
echo ""

# Check network connectivity to Ouster
if [ ! -z "${OUSTER_IP}" ]; then
    echo "Checking Ouster sensor connectivity..."
    if ping -c 1 -W 2 ${OUSTER_IP} &>/dev/null; then
        echo "✓ Ouster sensor reachable at ${OUSTER_IP}"
    else
        echo "⚠ Warning: Cannot reach Ouster sensor at ${OUSTER_IP}"
        echo "  Make sure sensor is powered and connected to same network"
    fi
    echo ""
fi

# Display available ROS packages
echo "Available SLAM packages:"
if [ -d /root/slam_ws/devel/lib/fast_lio ]; then
    echo "  ✓ FAST-LIO (SLAM)"
else
    echo "  ✗ FAST-LIO not found"
fi

if [ -d /root/slam_ws/devel/lib/ouster_ros ]; then
    echo "  ✓ Ouster ROS driver"
else
    echo "  ✗ Ouster ROS driver not found"
fi
echo ""

echo "================================================"
echo "Container ready. Starting requested command..."
echo "================================================"
echo ""

# Execute the main command
exec "$@"
