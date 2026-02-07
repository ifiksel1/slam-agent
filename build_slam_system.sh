#!/bin/bash
################################################################################
# Build SLAM System - Resolves CMake/googletest compatibility issues
################################################################################

set -e

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     SLAM System Build                                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Fix googletest CMakeLists.txt CMake version requirement
echo "[1/3] Fixing googletest CMake compatibility..."
sudo sed -i 's/cmake_minimum_required(VERSION 2.8.3)/cmake_minimum_required(VERSION 3.5)/' /usr/src/googletest/CMakeLists.txt 2>/dev/null || echo "⚠ Skipping googletest fix (requires sudo)"

# Clean workspace
echo "[2/3] Cleaning workspace..."
cd ~/slam_ws
rm -rf build devel .catkin_tools logs 2>/dev/null || true

# Build SLAM packages
echo "[3/3] Building FAST-LIO, Ouster driver, and Vision-to-MAVROS bridge..."
source /opt/ros/noetic/setup.bash

echo ""
echo "Building with catkin_build (this may take 2-5 minutes)..."
echo ""

catkin build fast_lio ouster_ros vision_to_mavros -j4 2>&1

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║     Build Summary                                          ║"
echo "╚════════════════════════════════════════════════════════════╝"

if [ -d ~/slam_ws/devel/lib/fast_lio ]; then
    echo "✅ Build SUCCESSFUL!"
    echo ""
    echo "Next steps:"
    echo "1. Source environment: source ~/slam_ws/devel/setup.bash"
    echo "2. Run tests: bash ~/slam-agent/RUN_FULL_SYSTEM_TEST.sh"
else
    echo "❌ Build FAILED - check errors above"
    exit 1
fi
