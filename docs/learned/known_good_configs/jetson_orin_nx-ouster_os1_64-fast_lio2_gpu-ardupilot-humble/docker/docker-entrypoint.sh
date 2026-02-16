#!/bin/bash
# Docker entrypoint for FAST-LIO-GPU SLAM system
# Sources ROS 2 workspace and provides diagnostic output

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}FAST-LIO-GPU SLAM System${NC}"
echo -e "${BLUE}Jetson Orin NX | CUDA 11.4 | ROS 2 Humble${NC}"
echo -e "${BLUE}========================================${NC}"

# Source ROS 2 environment
# Note: This image has ROS 2 built from source at a non-standard path
echo -e "${GREEN}[1/5] Sourcing ROS 2 Humble...${NC}"
if [ -f "/opt/ros/humble/install/setup.bash" ]; then
    source /opt/ros/humble/install/setup.bash
    echo -e "${GREEN}      OK - ROS 2 environment loaded${NC}"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}      OK - ROS 2 environment loaded (apt install)${NC}"
else
    echo -e "${RED}      FAIL - ROS 2 installation not found${NC}"
    exit 1
fi

# Source SLAM workspace
echo -e "${GREEN}[2/5] Sourcing SLAM workspace...${NC}"
if [ -f "/opt/slam_ws/install/setup.bash" ]; then
    source /opt/slam_ws/install/setup.bash
    echo -e "${GREEN}      OK - Workspace loaded${NC}"
else
    echo -e "${YELLOW}      WARN - Workspace not built yet${NC}"
fi

# Check CUDA availability
echo -e "${GREEN}[3/5] Checking CUDA/GPU...${NC}"
if command -v nvcc &> /dev/null; then
    CUDA_VER=$(nvcc --version 2>/dev/null | grep "release" | awk '{print $5}' | tr -d ',')
    echo -e "${GREEN}      OK - CUDA ${CUDA_VER}${NC}"
else
    echo -e "${YELLOW}      WARN - nvcc not in PATH${NC}"
fi
# Tegra GPU check (nvidia-smi not available on Jetson, use tegrastats or /dev/nvhost-gpu)
if [ -e "/dev/nvhost-gpu" ]; then
    echo -e "${GREEN}      OK - Tegra GPU device present${NC}"
elif command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null || echo "query failed")
    echo -e "${GREEN}      OK - GPU: ${GPU_INFO}${NC}"
else
    echo -e "${YELLOW}      WARN - No GPU device detected${NC}"
fi

# Check network access to Ouster
echo -e "${GREEN}[4/5] Checking Ouster LiDAR...${NC}"
if [ -n "$OUSTER_IP" ]; then
    if ping -c 1 -W 2 "$OUSTER_IP" &> /dev/null; then
        echo -e "${GREEN}      OK - Ouster reachable at $OUSTER_IP${NC}"
    else
        echo -e "${YELLOW}      WARN - Cannot ping Ouster at $OUSTER_IP (check network/PoE)${NC}"
    fi
else
    echo -e "${YELLOW}      WARN - OUSTER_IP not set${NC}"
fi

# Check serial device for ArduPilot
echo -e "${GREEN}[5/5] Checking ArduPilot serial...${NC}"
if [ -n "$MAVROS_PORT" ] && [ -e "$MAVROS_PORT" ]; then
    echo -e "${GREEN}      OK - Serial device: $MAVROS_PORT${NC}"
elif [ -e "/dev/ttyUSB0" ]; then
    echo -e "${GREEN}      OK - Serial device: /dev/ttyUSB0${NC}"
elif [ -e "/dev/ttyACM0" ]; then
    echo -e "${GREEN}      OK - Serial device: /dev/ttyACM0${NC}"
else
    echo -e "${YELLOW}      WARN - No serial device found (ArduPilot may not be connected)${NC}"
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Environment ready${NC}"
echo -e "${BLUE}========================================${NC}"

# Print configuration summary
echo ""
echo -e "${BLUE}Configuration:${NC}"
echo "  ROS_DOMAIN_ID:      ${ROS_DOMAIN_ID:-not set}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
echo "  OUSTER_IP:          ${OUSTER_IP:-not set}"
echo "  OUSTER_LIDAR_MODE:  ${OUSTER_LIDAR_MODE:-not set}"
echo "  MAVROS_PORT:        ${MAVROS_PORT:-not set}"
echo "  MAVROS_BAUD:        ${MAVROS_BAUD:-not set}"
echo ""

echo -e "${BLUE}Quick commands:${NC}"
echo "  ros2 topic list                       - Show ROS topics"
echo "  ros2 topic hz /ouster/points          - Check LiDAR data rate"
echo "  ros2 topic echo /Odometry             - View SLAM output"
echo "  ros2 run tf2_tools view_frames        - Visualize TF tree"
echo "  ros2 launch /opt/slam_ws/launch/slam_launch.py  - Launch SLAM"
echo ""

# Execute command passed to docker run
exec "$@"
