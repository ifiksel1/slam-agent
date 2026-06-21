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

    "EllipseLIO"|"ellipse-lio"|"ellipse_lio"|"ellipselio")
        # ROS 2 only (Humble + Jazzy). Adaptive LiDAR-inertial odometry with an
        # ellipsoid representation. Built on FAST-LIO2 + IKFoM + i-Octree, which
        # are VENDORED in include/ — no GTSAM/Ceres/Sophus needed.
        if [[ "$ROS_VERSION" == "ROS1" ]]; then
            log_warn "EllipseLIO is ROS 2 only — no ROS 1 / Noetic support exists. Aborting."
            exit 1
        fi
        if [[ -d "ellipselio" ]]; then
            log_success "EllipseLIO already cloned"
        else
            log_info "Cloning EllipseLIO..."
            git clone https://github.com/v4rl-ucy/ellipselio.git
            log_success "EllipseLIO cloned"
        fi
        # OpenMP is required (-fopenmp throughout); OpenCV only for camera path.
        sudo apt update
        sudo apt install -y libomp-dev libeigen3-dev libpcl-dev libopencv-dev \
            ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
            ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport
        log_info "Build with: colcon build --packages-select ellipselio \\"
        log_info "  --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install"
        log_warn "Jetson: if the EKF diverges on aarch64, override -Ofast -> -O3 (see frameworks doc)."
        log_success "EllipseLIO ready to build"
        ;;

    "SuperOdom"|"super-odom"|"super_odometry"|"superodom"|"SuperOdometry")
        # ROS 2 Humble only. LiDAR-inertial odometry with 6-DOF degeneracy /
        # uncertainty estimation. Needs source-built GTSAM + Sophus (+ Ceres),
        # plus livox_ros_driver2 and rviz_2d_overlay_plugins in the workspace.
        if [[ "$ROS_VERSION" == "ROS1" ]]; then
            log_warn "SuperOdom is ROS 2 Humble only — no ROS 1 support. Aborting."
            exit 1
        fi
        if [[ -d "SuperOdom" ]]; then
            log_success "SuperOdom already cloned"
        else
            log_info "Cloning SuperOdom + workspace companions..."
            git clone https://github.com/v4rl-ucy/SuperOdom.git
        fi
        # livox_ros_driver2 is a HARD build dependency even for Ouster-only use.
        [[ -d "livox_ros_driver2" ]] || git clone https://github.com/Livox-SDK/livox_ros_driver2.git
        [[ -d "rviz_2d_overlay_plugins" ]] || git clone https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git
        # System deps
        sudo apt update
        sudo apt install -y \
            ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-pcl-conversions \
            ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-rviz2 \
            ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-transport-plugins \
            ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-grid-map-msgs ros-${ROS_DISTRO}-grid-map \
            libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev \
            libsuitesparse-dev libglew-dev libceres-dev libtbb-dev
        # Sophus (pinned commit) — reuse the repo's ARM64-safe cmake shim if present.
        if [[ ! -d "/usr/local/include/sophus" ]]; then
            log_info "Building Sophus @97e7161 from source..."
            TMP_S=$(mktemp -d); git clone https://github.com/strasdat/Sophus.git "$TMP_S/Sophus"
            ( cd "$TMP_S/Sophus" && git checkout 97e7161 && mkdir -p build && cd build && \
              cmake .. -DBUILD_TESTS=OFF && make -j"$(nproc)" && sudo make install )
        fi
        # GTSAM (pinned commit) — MARCH_NATIVE=OFF is MANDATORY on aarch64.
        if [[ ! -d "/usr/local/include/gtsam" ]]; then
            log_info "Building GTSAM @4abef92 from source (this takes 20-30 min on Jetson)..."
            TMP_G=$(mktemp -d); git clone https://github.com/borglab/gtsam.git "$TMP_G/gtsam"
            ( cd "$TMP_G/gtsam" && git checkout 4abef92 && mkdir -p build && cd build && \
              cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. && \
              make -j"$(( $(nproc) > 6 ? 6 : $(nproc) ))" && sudo make install )
        fi
        sudo ldconfig
        log_info "Build Livox SDK2 + driver first:  cd src/livox_ros_driver2 && ./build.sh ${ROS_DISTRO}"
        log_info "Then build the workspace:  colcon build"
        log_warn "SuperOdom calibration is a SEPARATE OpenCV YAML (calibration_file arg), not the main params."
        log_success "SuperOdom cloned; heavy deps installed"
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
