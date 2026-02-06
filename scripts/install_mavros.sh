#!/bin/bash
# Install MAVROS
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

log_info "Installing MAVROS for $ROS_DISTRO..."

# Check if MAVROS already installed
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    if rospack find mavros &> /dev/null; then
        MAVROS_VERSION=$(dpkg -l | grep "ros-${ROS_DISTRO}-mavros" | awk '{print $3}')
        log_success "MAVROS already installed (version: $MAVROS_VERSION)"
        SKIP_MAVROS=true
    fi
else
    if ros2 pkg list | grep -q "^mavros$"; then
        log_success "MAVROS already installed"
        SKIP_MAVROS=true
    fi
fi

if [[ "$SKIP_MAVROS" != "true" ]]; then
    log_info "Installing MAVROS..."
    sudo apt update
    sudo apt install -y \
        ros-${ROS_DISTRO}-mavros \
        ros-${ROS_DISTRO}-mavros-extras
    
    log_success "MAVROS installed"
fi

# Install GeographicLib datasets (REQUIRED)
log_info "Installing GeographicLib datasets..."
if [[ -f "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" ]]; then
    sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh
    log_success "GeographicLib datasets installed"
else
    log_warn "GeographicLib install script not found. May need manual installation."
fi

# Optional: Install MAVProxy
read -p "Install MAVProxy for testing? (y/n) [default: n]: " install_mavproxy
if [[ "${install_mavproxy:-n}" == "y" ]]; then
    if ! command -v mavproxy.py &> /dev/null; then
        log_info "Installing MAVProxy..."
        sudo apt install -y python3-pip
        pip3 install --user MAVProxy
        log_success "MAVProxy installed"
    else
        log_success "MAVProxy already installed"
    fi
fi

log_success "MAVROS installation complete"
