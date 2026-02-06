#!/bin/bash
# Install Camera Driver
# Part of SLAM Integration Installation

set -e

CAMERA_TYPE=$1
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

log_info "Installing $CAMERA_TYPE driver..."

case "$CAMERA_TYPE" in
    "RealSense"|"realsense"|"Intel RealSense")
        log_info "Installing RealSense SDK..."
        # Check if already installed
        if dpkg -l | grep -q "librealsense2"; then
            log_success "RealSense SDK already installed"
        else
            # Install RealSense SDK
            sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F34CF4
            echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main" | \
                sudo tee /etc/apt/sources.list.d/realsense-public.list
            sudo apt update
            sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
            log_success "RealSense SDK installed"
        fi
        
        # Install ROS wrapper
        cd "$WORKSPACE_PATH/src"
        if [[ -d "realsense-ros" ]] || rospack find realsense2_camera &> /dev/null 2>&1; then
            log_success "RealSense ROS wrapper already installed"
        else
            log_info "Cloning RealSense ROS wrapper..."
            git clone https://github.com/IntelRealSense/realsense-ros.git -b $(rosversion -d)-devel
            log_success "RealSense ROS wrapper cloned"
        fi
        ;;
    
    "ZED"|"zed")
        log_info "Installing ZED SDK..."
        # ZED SDK installation is more complex, provide instructions
        log_warn "ZED SDK requires manual installation from: https://www.stereolabs.com/developers/release/"
        log_info "After installing ZED SDK, install ROS wrapper:"
        log_info "  cd $WORKSPACE_PATH/src"
        log_info "  git clone https://github.com/stereolabs/zed-ros-wrapper.git"
        ;;
    
    *)
        log_warn "Unknown camera type: $CAMERA_TYPE"
        log_info "Please install driver manually or add support to this script"
        exit 0
        ;;
esac

# Install dependencies
log_info "Installing camera driver dependencies..."
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

log_success "Camera driver installation complete"
