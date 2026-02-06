#!/bin/bash
# SLAM Integration Installation Script
# Automates Phase 4: Workspace Setup & Package Installation
# Version: 1.0
# 
# Usage:
#   ./install_slam_integration.sh [config_file.yaml]
#   If no config file provided, will prompt for configuration

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${1:-}"

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

check_ros_package() {
    local pkg=$1
    if [[ "$ROS_VERSION" == "ROS1" ]]; then
        rospack find "$pkg" &> /dev/null
    else
        ros2 pkg list | grep -q "^${pkg}$"
    fi
}

check_system_package() {
    dpkg -l | grep -q "^ii.*$1"
}

# Load configuration
if [[ -n "$CONFIG_FILE" && -f "$CONFIG_FILE" ]]; then
    log_info "Loading configuration from $CONFIG_FILE"
    source <(python3 -c "
import yaml
import sys
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
    # Export variables
    for key, value in config.items():
        if isinstance(value, dict):
            for k, v in value.items():
                print(f'export {key.upper()}_{k.upper()}={v}')
        else:
            print(f'export {key.upper()}={value}')
" 2>/dev/null || echo "# Failed to parse YAML, will prompt for config")
else
    log_warn "No config file provided. Will prompt for configuration."
fi

# Prompt for configuration if not set
if [[ -z "$ROS_VERSION" ]]; then
    echo "Select ROS version:"
    echo "1) ROS1 Noetic"
    echo "2) ROS1 Melodic"
    echo "3) ROS2 Humble"
    echo "4) ROS2 Iron"
    read -p "Choice [1-4]: " ros_choice
    case $ros_choice in
        1) export ROS_VERSION="ROS1" && export ROS_DISTRO="noetic" ;;
        2) export ROS_VERSION="ROS1" && export ROS_DISTRO="melodic" ;;
        3) export ROS_VERSION="ROS2" && export ROS_DISTRO="humble" ;;
        4) export ROS_VERSION="ROS2" && export ROS_DISTRO="iron" ;;
        *) log_error "Invalid choice"; exit 1 ;;
    esac
fi

if [[ -z "$WORKSPACE_PATH" ]]; then
    read -p "Workspace path [default: ~/catkin_ws]: " ws_path
    export WORKSPACE_PATH="${ws_path:-~/catkin_ws}"
fi
WORKSPACE_PATH="${WORKSPACE_PATH/#\~/$HOME}"

if [[ -z "$FLIGHT_CONTROLLER" ]]; then
    echo "Select flight controller:"
    echo "1) ArduPilot"
    echo "2) PX4"
    read -p "Choice [1-2]: " fc_choice
    case $fc_choice in
        1) export FLIGHT_CONTROLLER="ArduPilot" ;;
        2) export FLIGHT_CONTROLLER="PX4" ;;
        *) log_error "Invalid choice"; exit 1 ;;
    esac
fi

if [[ -z "$USE_DDS" ]]; then
    if [[ "$ROS_VERSION" == "ROS2" && "$FLIGHT_CONTROLLER" == "PX4" ]]; then
        export USE_DDS="true"
        log_info "ROS2 + PX4 detected. Using DDS (micro-ROS Agent)."
    else
        read -p "Use DDS instead of MAVROS? (y/n) [default: n]: " use_dds
        export USE_DDS="${use_dds:-n}"
        [[ "$USE_DDS" == "y" ]] && export USE_DDS="true" || export USE_DDS="false"
    fi
fi

# Source ROS
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || {
        log_error "ROS $ROS_DISTRO not found. Please install ROS first."
        exit 1
    }
else
    source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || {
        log_error "ROS2 $ROS_DISTRO not found. Please install ROS2 first."
        exit 1
    }
fi

log_info "Configuration:"
log_info "  ROS Version: $ROS_VERSION ($ROS_DISTRO)"
log_info "  Flight Controller: $FLIGHT_CONTROLLER"
log_info "  Use DDS: $USE_DDS"
log_info "  Workspace: $WORKSPACE_PATH"
echo ""

# Step 1: Create/Check Workspace
log_info "Step 1: Checking/Creating ROS workspace..."
if [[ -d "$WORKSPACE_PATH/src" ]]; then
    log_success "Workspace already exists: $WORKSPACE_PATH"
else
    log_info "Creating workspace at $WORKSPACE_PATH"
    mkdir -p "$WORKSPACE_PATH/src"
    cd "$WORKSPACE_PATH"
    if [[ "$ROS_VERSION" == "ROS1" ]]; then
        catkin init || catkin_make
        source devel/setup.bash 2>/dev/null || true
    else
        # ROS2 workspace doesn't need init
        log_info "ROS2 workspace created. Will build later."
    fi
    log_success "Workspace created"
fi

# Step 2: Install Core ROS Packages
log_info "Step 2: Installing core ROS packages..."
"$SCRIPT_DIR/install_core_ros_packages.sh" "$ROS_VERSION" "$ROS_DISTRO"

# Step 3: Install MAVROS or DDS
log_info "Step 3: Installing flight controller bridge..."
if [[ "$USE_DDS" == "true" ]]; then
    "$SCRIPT_DIR/install_dds_bridge.sh" "$ROS_VERSION" "$ROS_DISTRO" "$FLIGHT_CONTROLLER" "$WORKSPACE_PATH"
else
    "$SCRIPT_DIR/install_mavros.sh" "$ROS_VERSION" "$ROS_DISTRO"
fi

# Step 4: Install Sensor Drivers
log_info "Step 4: Installing sensor drivers..."
if [[ -n "$LIDAR_TYPE" ]]; then
    "$SCRIPT_DIR/install_lidar_driver.sh" "$LIDAR_TYPE" "$WORKSPACE_PATH" "$ROS_VERSION"
fi
if [[ -n "$CAMERA_TYPE" ]]; then
    "$SCRIPT_DIR/install_camera_driver.sh" "$CAMERA_TYPE" "$WORKSPACE_PATH" "$ROS_VERSION"
fi

# Step 5: Install SLAM Algorithm
log_info "Step 5: Installing SLAM algorithm..."
if [[ -n "$SLAM_ALGORITHM" ]]; then
    "$SCRIPT_DIR/install_slam_algorithm.sh" "$SLAM_ALGORITHM" "$WORKSPACE_PATH" "$ROS_VERSION"
fi

# Step 6: Install vision_to_mavros (if not using DDS)
log_info "Step 6: Installing vision_to_mavros bridge..."
if [[ "$USE_DDS" != "true" ]]; then
    "$SCRIPT_DIR/install_vision_to_mavros.sh" "$WORKSPACE_PATH" "$ROS_VERSION"
fi

# Step 7: Build Workspace
log_info "Step 7: Building workspace..."
cd "$WORKSPACE_PATH"
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    catkin build || catkin_make
    source devel/setup.bash
else
    colcon build
    source install/setup.bash
fi
log_success "Workspace built successfully"

# Step 8: Verify Installation
log_info "Step 8: Verifying installation..."
"$SCRIPT_DIR/verify_installation.sh" "$ROS_VERSION" "$ROS_DISTRO" "$WORKSPACE_PATH"

log_success "Installation complete!"
log_info "Next steps:"
log_info "  1. Source your workspace: source $WORKSPACE_PATH/devel/setup.bash (ROS1) or source $WORKSPACE_PATH/install/setup.bash (ROS2)"
log_info "  2. Configure LiDAR network (if using Ethernet LiDAR)"
log_info "  3. Run diagnostics: $SCRIPT_DIR/slam_diagnostics.sh"
log_info "  4. Follow testing protocol from Phase 5"
