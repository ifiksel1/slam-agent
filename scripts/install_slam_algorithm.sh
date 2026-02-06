#!/bin/bash
# Install SLAM Algorithm
# Part of SLAM Integration Installation

set -e

SLAM_ALGORITHM=$1
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

log_info "Installing $SLAM_ALGORITHM..."

cd "$WORKSPACE_PATH/src"

# Common SLAM algorithm repositories
case "$SLAM_ALGORITHM" in
    "FAST-LIO"|"FAST-LIO2"|"fast-lio"|"fast-lio2")
        if [[ -d "FAST_LIO" ]] || rospack find fast_lio &> /dev/null 2>&1; then
            log_success "FAST-LIO already installed"
        else
            log_info "Cloning FAST-LIO..."
            git clone https://github.com/hku-mars/FAST_LIO.git
            log_success "FAST-LIO cloned"
        fi
        ;;
    
    "LIO-SAM"|"lio-sam")
        if [[ -d "lio_sam" ]] || rospack find lio_sam &> /dev/null 2>&1; then
            log_success "LIO-SAM already installed"
        else
            log_info "Cloning LIO-SAM..."
            git clone https://github.com/TixiaoShan/LIO-SAM.git
            log_success "LIO-SAM cloned"
        fi
        ;;
    
    "LVI-SAM"|"lvi-sam")
        if [[ -d "lvio_sam" ]] || rospack find lvio_sam &> /dev/null 2>&1; then
            log_success "LVI-SAM already installed"
        else
            log_info "Cloning LVI-SAM..."
            git clone https://github.com/TixiaoShan/LVI-SAM.git
            log_success "LVI-SAM cloned"
        fi
        ;;
    
    "OpenVINS"|"openvins")
        log_warn "OpenVINS installation is complex. Checking for existing installation..."
        if rospack find ov_msckf &> /dev/null 2>&1; then
            log_success "OpenVINS already installed"
        else
            log_info "Cloning OpenVINS..."
            git clone https://github.com/rpng/open_vins.git
            log_warn "OpenVINS requires additional setup. See: https://docs.openvins.com/"
            log_success "OpenVINS cloned"
        fi
        ;;
    
    *)
        log_warn "Unknown SLAM algorithm: $SLAM_ALGORITHM"
        log_info "Please install manually or check: https://github.com/engcang/SLAM-application"
        log_info "For installation guides for 20+ SLAM systems"
        exit 0
        ;;
esac

# Install dependencies
log_info "Installing SLAM algorithm dependencies..."
cd "$WORKSPACE_PATH"

# Install common SLAM dependencies
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev

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

log_success "SLAM algorithm installation complete"
log_info "Note: Some SLAM algorithms may require additional dependencies."
log_info "Check the algorithm's README for specific requirements."
