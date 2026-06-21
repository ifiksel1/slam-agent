#!/bin/bash
# Docker entrypoint for FAST-LIVO2 SLAM system (Ouster OS1-64 + ArduPilot)

set -e

echo "========================================================"
echo "FAST-LIVO2 SLAM Container Starting"
echo "========================================================"

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash

# -----------------------------------------------------------------------
# OpenCV 4.2 / 4.5 ABI fix (Jetson JetPack host has OpenCV 4.5 while
# ROS Noetic's cv_bridge was built against 4.2 → setSize crash in
# cv::filter2D).  On the Jetson HOST, the system OpenCV 4.5 libs take
# precedence; LD_PRELOAD forces the container-local 4.2 libs first.
# Inside a fully-isolated Docker container this is a harmless no-op.
# -----------------------------------------------------------------------
OPENCV_SO=$(ls /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2* 2>/dev/null | head -1)
if [ -n "$OPENCV_SO" ]; then
    ALL_OPENCV_LIBS=$(ls /usr/lib/aarch64-linux-gnu/libopencv_{core,highgui,imgproc,features2d,calib3d,imgcodecs}.so.4.2* 2>/dev/null | tr '\n' ':' | sed 's/:$//')
    if [ -n "$ALL_OPENCV_LIBS" ]; then
        export LD_PRELOAD="${ALL_OPENCV_LIBS}${LD_PRELOAD:+:$LD_PRELOAD}"
        echo "OpenCV ABI fix: LD_PRELOAD set for OpenCV 4.2 libs"
    fi
else
    echo "Note: OpenCV 4.2 so not found at expected path — skipping LD_PRELOAD"
fi

# ROS network
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

echo "ROS Configuration:"
echo "  ROS_HOSTNAME: $ROS_HOSTNAME"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
OUSTER_IP="${OUSTER_IP:-192.168.2.60}"
echo "  OUSTER_IP: $OUSTER_IP"
echo ""

# Ouster connectivity
echo "Checking Ouster sensor connectivity..."
if ping -c 1 -W 2 "$OUSTER_IP" &>/dev/null; then
    echo "  Ouster reachable at $OUSTER_IP"
else
    echo "  WARNING: Cannot reach Ouster at $OUSTER_IP — check eth0 / sensor power"
fi
echo ""

# Package verification
echo "SLAM package status:"
for pkg in fast_livo ouster_ros vision_to_mavros fast_livo2_integration; do
    if [ -d /root/slam_ws/devel/lib/$pkg ] || [ -d /root/slam_ws/devel/share/$pkg ]; then
        echo "  [OK]  $pkg"
    else
        echo "  [!!]  $pkg — NOT FOUND in devel (build may have failed)"
    fi
done
echo ""

echo "========================================================"
echo "Container ready. Starting requested command..."
echo "========================================================"
echo ""

exec "$@"
