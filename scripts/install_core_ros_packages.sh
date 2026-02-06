#!/bin/bash
# Install Core ROS Packages
# Part of SLAM Integration Installation

set -e

ROS_VERSION=$1
ROS_DISTRO=$2

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }

log_info "Installing core ROS packages for $ROS_DISTRO..."

# Check and install build tools
log_info "Checking build tools..."
MISSING_TOOLS=()
for tool in cmake git; do
    if ! command -v "$tool" &> /dev/null; then
        MISSING_TOOLS+=("$tool")
    fi
done

if [[ ${#MISSING_TOOLS[@]} -gt 0 ]]; then
    log_info "Installing build tools: ${MISSING_TOOLS[*]}"
    sudo apt update
    sudo apt install -y build-essential cmake git
else
    log_success "Build tools already installed"
fi

# Check and install ROS build tools
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    if ! command -v catkin &> /dev/null && ! command -v catkin_make &> /dev/null; then
        log_info "Installing catkin tools..."
        sudo apt install -y python3-catkin-tools python3-osrf-pycommon python3-wstool
    else
        log_success "Catkin tools already installed"
    fi
else
    if ! command -v colcon &> /dev/null; then
        log_info "Installing colcon..."
        sudo apt install -y python3-colcon-common-extensions
    else
        log_success "Colcon already installed"
    fi
fi

# Install core ROS packages
log_info "Installing core ROS packages..."
PACKAGES_TO_INSTALL=()

# TF2 packages
if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-tf2-ros"; then
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-tf2-ros")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-tf2-geometry-msgs")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-tf2-sensor-msgs")
fi

# Robot state publisher
if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-robot-state-publisher"; then
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-robot-state-publisher")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-joint-state-publisher")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-xacro")
fi

# Visualization tools
if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-rviz"; then
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-rviz")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-rqt")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-rqt-common-plugins")
fi

# Common libraries
if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-pcl-ros"; then
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-pcl-ros")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-pcl-conversions")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-eigen-conversions")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-cv-bridge")
    PACKAGES_TO_INSTALL+=("ros-${ROS_DISTRO}-image-transport")
fi

if [[ ${#PACKAGES_TO_INSTALL[@]} -gt 0 ]]; then
    log_info "Installing ${#PACKAGES_TO_INSTALL[@]} packages..."
    sudo apt update
    sudo apt install -y "${PACKAGES_TO_INSTALL[@]}"
    log_success "Core ROS packages installed"
else
    log_success "All core ROS packages already installed"
fi
