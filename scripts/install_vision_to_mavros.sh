#!/bin/bash
# Install vision_to_mavros Bridge
# Part of SLAM Integration Installation

set -e

WORKSPACE_PATH=$1
ROS_VERSION=$2

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[⚠]${NC} $1"; }

log_info "Installing vision_to_mavros bridge..."

cd "$WORKSPACE_PATH/src"

# Check if already installed
if [[ -d "vision_to_mavros" ]] || rospack find vision_to_mavros &> /dev/null 2>&1; then
    log_success "vision_to_mavros already installed"
    exit 0
fi

# Clone vision_to_mavros
log_info "Cloning vision_to_mavros..."
git clone https://github.com/thien94/vision_to_mavros.git

log_success "vision_to_mavros cloned"

# Install dependencies
log_info "Installing dependencies..."
cd "$WORKSPACE_PATH"
if [[ "$ROS_VERSION" == "ROS1" ]]; then
    if command -v rosdep &> /dev/null; then
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y || log_warn "Some dependencies may need manual installation"
    fi
else
    log_warn "vision_to_mavros is primarily for ROS1. For ROS2, you may need a custom DDS publisher."
fi

log_success "vision_to_mavros installation complete"
