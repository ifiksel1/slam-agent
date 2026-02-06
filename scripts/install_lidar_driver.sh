#!/bin/bash
# Install LiDAR Driver
# Part of SLAM Integration Installation

set -e

LIDAR_TYPE=$1
WORKSPACE_PATH=$2
ROS_VERSION=$3

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }

log_info "Installing $LIDAR_TYPE driver..."

cd "$WORKSPACE_PATH/src"

case "$LIDAR_TYPE" in
    "Ouster"|"ouster")
        if [[ -d "ouster-ros" ]] || rospack find ouster_ros &> /dev/null 2>&1; then
            log_success "Ouster driver already installed"
        else
            log_info "Cloning Ouster ROS driver..."
            git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
            log_success "Ouster driver cloned"
        fi
        ;;
    
    "Velodyne"|"velodyne")
        if [[ -d "velodyne" ]] || rospack find velodyne_driver &> /dev/null 2>&1; then
            log_success "Velodyne driver already installed"
        else
            log_info "Cloning Velodyne ROS driver..."
            git clone https://github.com/ros-drivers/velodyne.git
            log_success "Velodyne driver cloned"
        fi
        ;;
    
    "Livox"|"livox")
        if [[ -d "livox_ros_driver" ]] || rospack find livox_ros_driver &> /dev/null 2>&1; then
            log_success "Livox driver already installed"
        else
            log_info "Cloning Livox ROS driver..."
            git clone https://github.com/Livox-SDK/livox_ros_driver.git
            log_success "Livox driver cloned"
        fi
        ;;
    
    "Hesai"|"hesai")
        if [[ -d "hesai_ros_driver" ]] || rospack find hesai_ros_driver &> /dev/null 2>&1; then
            log_success "Hesai driver already installed"
        else
            log_info "Cloning Hesai ROS driver..."
            git clone https://github.com/HesaiTechnology/HesaiLidar_General_ROS.git hesai_ros_driver
            log_success "Hesai driver cloned"
        fi
        ;;
    
    *)
        log_warn "Unknown LiDAR type: $LIDAR_TYPE"
        log_info "Please install driver manually or add support to this script"
        exit 0
        ;;
esac

# Install dependencies
log_info "Installing LiDAR driver dependencies..."
cd "$WORKSPACE_PATH"
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    if command -v rosdep &> /dev/null; then
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y || log_warn "Some dependencies may need manual installation"
    fi
else
    if command -v rosdep &> /dev/null; then
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y || log_warn "Some dependencies may need manual installation"
    fi
fi

log_success "LiDAR driver installation complete"
