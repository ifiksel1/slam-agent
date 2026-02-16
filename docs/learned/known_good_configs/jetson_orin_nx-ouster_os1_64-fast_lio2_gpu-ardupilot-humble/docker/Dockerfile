# FAST-LIO-GPU Dockerfile for Jetson Orin NX
# Architecture: ARM64 (aarch64)
# Base: ros2-jetson-humble (L4T R35.4.1, CUDA 11.4, ROS 2 Humble)
# Target: GPU-accelerated SLAM with Ouster OS1-64 + ArduPilot
#
# IMPORTANT: This Dockerfile uses /bin/sh compatible syntax in all RUN
# commands. No bash-specific constructs (arrays, ${var//}, etc.).
# All variable expansions use hardcoded values where possible.

# ==============================================================================
# BASE IMAGE
# ==============================================================================
# Use the locally-built ros2-jetson-humble image which already contains:
# - Ubuntu 20.04 (Focal) for ARM64
# - CUDA 11.4 toolkit (nvcc, cublas, cusolver, cuDNN)
# - ROS 2 Humble (built from source at /opt/ros/humble/install/)
# - PCL 1.10, Eigen 3.3.7, OpenCV, colcon, rosdep, vcstool
# - All core ROS 2 packages (rclcpp, tf2, pcl_conversions, etc.)
FROM ros2-jetson-humble:latest

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# NVIDIA container runtime environment
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ==============================================================================
# SHELL CONFIGURATION
# ==============================================================================
# Use bash for RUN commands so we can source setup files properly.
# This avoids "Bad substitution" errors from /bin/sh (dash).
SHELL ["/bin/bash", "-c"]

# ==============================================================================
# SYSTEM DEPENDENCIES
# ==============================================================================
# Install packages needed by FAST_LIO_GPU and Ouster driver that may not
# be in the base image. Use || true for packages that might not exist on ARM64.

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    ca-certificates \
    iputils-ping \
    # Ouster ROS 2 driver dependencies
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    libtins-dev \
    # FAST_LIO_GPU dependencies
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    # Python build tools (colcon, etc. already in base image)
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ==============================================================================
# ROS 2 SETUP PATH
# ==============================================================================
# The base image has ROS 2 Humble built from source at a non-standard path.
# All subsequent ROS operations must source this path.
ENV ROS_SETUP=/opt/ros/humble/install/setup.bash

# Verify ROS 2 and CUDA are accessible
RUN source ${ROS_SETUP} && \
    echo "ROS 2 distro: ${ROS_DISTRO}" && \
    echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH}" && \
    nvcc --version && \
    echo "CUDA and ROS 2 verified OK"

# ==============================================================================
# CREATE WORKSPACE
# ==============================================================================
RUN mkdir -p /opt/slam_ws/src
WORKDIR /opt/slam_ws

# ==============================================================================
# CLONE OUSTER ROS 2 DRIVER
# ==============================================================================
# The ouster-ros driver provides /ouster/points and /ouster/imu topics.
# Using --recurse-submodules for the ouster SDK dependency.
RUN cd /opt/slam_ws/src && \
    git clone --branch ros2 --depth 1 --recurse-submodules \
        https://github.com/ouster-lidar/ouster-ros.git && \
    echo "Ouster ROS 2 driver cloned"

# ==============================================================================
# CLONE FAST_LIO_GPU
# ==============================================================================
# CRITICAL: Must use --recursive to pull ikd-Tree submodule.
# Without it, the build will fail with missing headers.
RUN cd /opt/slam_ws/src && \
    git clone --recursive --depth 1 \
        https://github.com/OmerMersin/FAST_LIO_GPU.git && \
    # Patch: replace pcl_ros with its actual transitive deps
    # pcl_ros is listed but never #included; tf2_ros IS used but was transitive
    # PCL filters component needed for VoxelGrid (was transitive via pcl_ros)
    sed -i 's/find_package(pcl_ros REQUIRED)/find_package(tf2_ros REQUIRED)/' FAST_LIO_GPU/CMakeLists.txt && \
    sed -i 's/^  pcl_ros$/  tf2_ros/' FAST_LIO_GPU/CMakeLists.txt && \
    sed -i 's/<depend>pcl_ros<\/depend>/<depend>tf2_ros<\/depend>/' FAST_LIO_GPU/package.xml && \
    sed -i 's/find_package(PCL REQUIRED COMPONENTS common io)/find_package(PCL REQUIRED COMPONENTS common io filters)/' FAST_LIO_GPU/CMakeLists.txt && \
    echo "FAST_LIO_GPU cloned and patched (pcl_ros -> tf2_ros, added PCL filters)"

# ==============================================================================
# INSTALL WORKSPACE DEPENDENCIES VIA ROSDEP
# ==============================================================================
# Initialize rosdep if not already done, then install any missing dependencies.
# Skip keys that are already installed or unavailable on ARM64.
RUN rosdep init 2>/dev/null || true && \
    rosdep update --rosdistro humble && \
    source ${ROS_SETUP} && \
    cd /opt/slam_ws && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "libtins-dev livox_ros_driver2" \
    || echo "Some rosdep keys skipped (expected on ARM64)"

# ==============================================================================
# BUILD OUSTER DRIVER FIRST (separate step for better caching)
# ==============================================================================
# Build Ouster driver alone first. It has complex C++ dependencies and
# building it separately helps isolate failures.
RUN source ${ROS_SETUP} && \
    cd /opt/slam_ws && \
    colcon build \
        --packages-select ouster_ros ouster_sensor_msgs \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_FLAGS="-O2" \
        --parallel-workers 2 \
        --symlink-install \
    && echo "Ouster driver built successfully"

# ==============================================================================
# BUILD FAST_LIO_GPU WITH CUDA
# ==============================================================================
# Build FAST_LIO_GPU (package name: fast_lio) with CUDA enabled.
#
# CUDA Architecture Notes for Jetson Orin NX:
#   - Compute Capability: 8.7 (Ampere / Orin)
#   - CUDA 11.4 supports SM 8.7 via -arch=sm_87
#   - CMAKE_CUDA_ARCHITECTURES=87 tells CMake to target this GPU
#
# Memory-constrained build:
#   - Use --parallel-workers 2 to limit RAM usage on 8GB Jetson
#   - Sequential executor prevents parallel package builds from OOMing
#   - -j2 limits make parallelism within each package
RUN source ${ROS_SETUP} && \
    source /opt/slam_ws/install/setup.bash 2>/dev/null || true && \
    cd /opt/slam_ws && \
    export CUDACXX=/usr/local/cuda/bin/nvcc && \
    colcon build \
        --packages-select fast_lio \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DFASTLIO_USE_CUDA=ON \
            -DCMAKE_CUDA_ARCHITECTURES=87 \
            -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
            -DCMAKE_CXX_FLAGS="-O2 -fopenmp" \
        --parallel-workers 1 \
        --executor sequential \
        --symlink-install \
    && echo "FAST_LIO_GPU built successfully with CUDA"

# ==============================================================================
# MAVROS - DEFERRED
# ==============================================================================
# MAVROS is NOT built in this image. Building from source on from-source
# Humble ARM64 has version incompatibilities (mavlink::standard namespace
# mismatch between MAVROS ros2 HEAD and Humble mavlink GBP release).
#
# The core SLAM pipeline (Ouster → FAST_LIO_GPU → odometry) works without
# MAVROS. ArduPilot integration can be added later via:
#   1. A separate mavros container with Iron/Jazzy base (apt install)
#   2. A lightweight MAVLink bridge (pymavlink or MAVSDK)
#   3. Rebuilding with pinned MAVROS 2.7.x + matching mavlink
#
# For Phase 5 bench testing, MAVROS is not required.

# ==============================================================================
# ENVIRONMENT SETUP
# ==============================================================================
# Configure bashrc for interactive use
RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/slam_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

# Create mount points for runtime config, launch files, data
RUN mkdir -p /opt/slam_ws/config /opt/slam_ws/launch /opt/slam_ws/bags

# Runtime environment variables
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/opt/slam_ws/install/lib:${LD_LIBRARY_PATH}

# ==============================================================================
# ENTRYPOINT
# ==============================================================================
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
