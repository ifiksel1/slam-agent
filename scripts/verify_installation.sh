#!/bin/bash
# Verify Installation
# Part of SLAM Integration Installation

set +e  # Don't exit on errors

ROS_VERSION=$1
ROS_DISTRO=$2
WORKSPACE_PATH=$3

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

log_info "Verifying installation..."

ERRORS=0
WARNINGS=0

# Source ROS
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
    source "$WORKSPACE_PATH/devel/setup.bash" 2>/dev/null
else
    source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
    source "$WORKSPACE_PATH/install/setup.bash" 2>/dev/null
fi

# Check workspace
if [[ -d "$WORKSPACE_PATH/src" ]]; then
    log_success "Workspace exists: $WORKSPACE_PATH"
else
    log_error "Workspace not found: $WORKSPACE_PATH"
    ((ERRORS++))
fi

# Check core ROS packages
log_info "Checking core ROS packages..."

# Detect actual ROS version from environment
ACTUAL_ROS_VERSION=""
if command -v rospack &> /dev/null && [[ -n "$ROS_DISTRO" ]]; then
    # ROS1 detected (rospack exists)
    ACTUAL_ROS_VERSION="ROS1"
elif command -v ros2 &> /dev/null && [[ -n "$ROS_DISTRO" ]]; then
    # ROS2 detected (ros2 command exists)
    ACTUAL_ROS_VERSION="ROS2"
else
    log_error "Cannot detect ROS environment. Make sure ROS is sourced."
    ((ERRORS++))
fi

# Check if detected version matches parameter
if [[ -n "$ACTUAL_ROS_VERSION" && "$ROS_VERSION" != "$ACTUAL_ROS_VERSION" ]]; then
    log_warn "ROS version mismatch: expected $ROS_VERSION, detected $ACTUAL_ROS_VERSION"
    log_warn "Using detected version: $ACTUAL_ROS_VERSION"
    ROS_VERSION="$ACTUAL_ROS_VERSION"
fi

if [[ "$ROS_VERSION" == "ROS1" && "$ACTUAL_ROS_VERSION" == "ROS1" ]]; then
    for pkg in tf2_ros robot_state_publisher; do
        if rospack find "$pkg" &> /dev/null; then
            log_success "$pkg installed"
        else
            log_error "$pkg not found"
            ((ERRORS++))
        fi
    done
elif [[ "$ROS_VERSION" == "ROS2" && "$ACTUAL_ROS_VERSION" == "ROS2" ]]; then
    for pkg in tf2_ros robot_state_publisher; do
        if ros2 pkg list | grep -q "^${pkg}$"; then
            log_success "$pkg installed"
        else
            log_error "$pkg not found"
            ((ERRORS++))
        fi
    done
fi

# Check MAVROS or DDS
log_info "Checking flight controller bridge..."
if [[ "$ROS_VERSION" == "ROS1" && "$ACTUAL_ROS_VERSION" == "ROS1" ]]; then
    if rospack find mavros &> /dev/null; then
        log_success "MAVROS installed"
    else
        log_error "MAVROS not found"
        ((ERRORS++))
    fi
elif [[ "$ROS_VERSION" == "ROS2" && "$ACTUAL_ROS_VERSION" == "ROS2" ]]; then
    if ros2 pkg list | grep -q "^micro_ros_agent$"; then
        log_success "micro-ROS Agent (DDS) installed"
    elif ros2 pkg list | grep -q "^mavros$"; then
        log_success "MAVROS installed"
    else
        log_error "No flight controller bridge found"
        ((ERRORS++))
    fi
fi

# Summary
echo ""
if [[ $ERRORS -eq 0 ]]; then
    log_success "Verification complete! All critical components installed."
    exit 0
else
    log_error "Verification found $ERRORS error(s). Please review above."
    exit 1
fi
