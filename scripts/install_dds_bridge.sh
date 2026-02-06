#!/bin/bash
# Install DDS Bridge (micro-ROS Agent)
# Part of SLAM Integration Installation

set -e

ROS_VERSION=$1
ROS_DISTRO=$2
FLIGHT_CONTROLLER=$3
WORKSPACE_PATH=$4

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }
log_error() { echo -e "${RED}[✗]${NC} $1"; }

if [[ "$ROS_VERSION" != "ROS2" ]]; then
    log_error "DDS is only available for ROS2"
    exit 1
fi

log_info "Installing micro-ROS Agent (DDS) for $FLIGHT_CONTROLLER..."

# Check if already installed
if ros2 pkg list | grep -q "^micro_ros_agent$"; then
    log_success "micro-ROS Agent already installed"
    exit 0
fi

# Try apt install first (easier)
log_info "Attempting to install from apt..."
if sudo apt install -y ros-${ROS_DISTRO}-micro-ros-agent 2>/dev/null; then
    log_success "micro-ROS Agent installed from apt"
    ros2 run micro_ros_agent micro_ros_agent --help &> /dev/null && log_success "micro-ROS Agent verified"
    exit 0
fi

log_warn "Apt package not available. Building from source..."

# Install dependencies
log_info "Installing build dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions

pip3 install --user empy pyros-genmsg setuptools

# Clone and build micro-ROS setup
cd "$WORKSPACE_PATH/src"
if [[ ! -d "micro_ros_setup" ]]; then
    log_info "Cloning micro-ROS setup..."
    git clone https://github.com/micro-ROS/micro_ros_setup.git
fi

# Build micro-ROS setup
cd "$WORKSPACE_PATH"
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select micro_ros_setup
source install/setup.bash

# Create agent workspace
log_info "Creating micro-ROS agent workspace..."
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
log_info "Building micro-ROS agent..."
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# Verify installation
if ros2 pkg list | grep -q "^micro_ros_agent$"; then
    log_success "micro-ROS Agent installed and verified"
else
    log_error "micro-ROS Agent installation failed"
    exit 1
fi

# For ArduPilot: Install ardupilot_msgs
if [[ "$FLIGHT_CONTROLLER" == "ArduPilot" ]]; then
    log_info "Installing ardupilot_msgs for ArduPilot DDS support..."
    cd "$WORKSPACE_PATH/src"
    
    if [[ ! -d "ardupilot_msgs" ]]; then
        log_info "Cloning ArduPilot to extract ardupilot_msgs..."
        git clone https://github.com/ArduPilot/ardupilot.git --depth=1
        cp -r ardupilot/Tools/ros2/ardupilot_msgs .
        rm -rf ardupilot
    fi
    
    cd "$WORKSPACE_PATH"
    colcon build --packages-select ardupilot_msgs
    source install/setup.bash
    log_success "ardupilot_msgs installed"
fi

log_success "DDS bridge installation complete"
