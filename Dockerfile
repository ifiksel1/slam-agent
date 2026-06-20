# Complete SLAM System Docker Image - FAST-LIO-SLAM (FAST-LIO + SC-PGO) + Hesai JT128 + MAVRos
# Builds the RevoluteRobotics/FAST_LIO_SLAM fork, which bundles:
#   - fast_lio          (FAST-LIO LiDAR-inertial odometry)
#   - aloam_velodyne    (SC-PGO Scan-Context pose-graph loop closure)
#   - hesai_ros_driver  (HesaiLidar_ROS_2.0 driver; outputs directly in FAST-LIO format)
# plus livox_ros_driver (FAST-LIO CustomMsg dependency) and vision_to_mavros (MAVRos bridge).
# Optimized for ARM64 (Jetson) and x86_64. Ubuntu 20.04 + ROS1 Noetic.

FROM ros:noetic-ros-base-focal

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Set up labels
LABEL maintainer="SLAM Agent"
LABEL description="FAST-LIO-SLAM (FAST-LIO + SC-PGO) for Hesai JT128, with MAVRos (ArduPilot)"
LABEL version="3.0"

# Install system dependencies (per RevoluteRobotics/FAST_LIO_SLAM README + driver/SC-PGO needs)
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    git \
    wget \
    curl \
    software-properties-common \
    # ROS build tools
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # FAST-LIO / SC-PGO / Ceres / GTSAM math deps
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libtbb-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libopenblas-dev \
    liblapack-dev \
    # Hesai SDK 2.0 (UDP capture)
    libpcap-dev \
    # ROS packages used by the workspace
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    # Foxglove WebSocket bridge (remote map/point-cloud streaming to laptop/Android)
    ros-${ROS_DISTRO}-foxglove-bridge \
    # MAVRos (ArduPilot connectivity)
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    # Networking + debugging tools
    net-tools \
    iputils-ping \
    iproute2 \
    vim \
    htop \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# Install GTSAM 4.0 (pose graph optimization for SC-PGO) from the borglab PPA
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Ceres Solver 2.2.0 (required by SC-PGO / A-LOAM)
WORKDIR /tmp
RUN wget -q http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && \
    tar zxf ceres-solver-2.2.0.tar.gz && \
    mkdir ceres-bin && cd ceres-bin && \
    cmake ../ceres-solver-2.2.0 \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTING=OFF \
          -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf ceres-bin ceres-solver-2.2.0*

# Install Livox-SDK (build dependency of livox_ros_driver, which FAST-LIO needs for CustomMsg)
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK/build && \
    cmake .. && make -j$(nproc) && make install && \
    cd /tmp && rm -rf Livox-SDK

# Create workspace and add sources.
# RevoluteRobotics/FAST_LIO_SLAM is PRIVATE, so the sources are vendored on the host
# (run scripts/fetch_docker_sources.sh, which clones with your gh credentials) and
# COPYed in here rather than cloned inside the build, where no credentials exist.
# This also keeps no GitHub token in the image history.
WORKDIR /root/slam_ws/src
COPY docker_src/ /root/slam_ws/src/

# Fix googletest CMake compatibility (common issue on Ubuntu 20.04)
RUN if [ -f /usr/src/googletest/CMakeLists.txt ]; then \
        sed -i 's/cmake_minimum_required(VERSION 2.8.3)/cmake_minimum_required(VERSION 3.5)/' /usr/src/googletest/CMakeLists.txt; \
    fi

# Fix a missing message-generation dependency in the fork's FAST-LIO CMakeLists.
# fastlio_mapping compiles laserMapping.cpp, which includes the generated header
# fast_lio/Pose6D.h, but the upstream add_dependencies() was dropped — so a parallel
# build races ahead of message generation ("fatal error: fast_lio/Pose6D.h: No such
# file or directory"). Re-add it so the message headers are built first.
RUN FASTLIO_CML=/root/slam_ws/src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt && \
    grep -q 'add_dependencies(fastlio_mapping fast_lio_generate_messages_cpp)' "$FASTLIO_CML" || \
    printf '\nadd_dependencies(fastlio_mapping fast_lio_generate_messages_cpp)\n' >> "$FASTLIO_CML"

# Install GeographicLib datasets for MAVRos
WORKDIR /tmp
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Resolve any remaining ROS dependencies
WORKDIR /root/slam_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y || echo "rosdep finished with warnings"

# Build the workspace. livox_ros_driver builds first so its CustomMsg headers are
# available to fast_lio; catkin resolves the rest of the dependency order.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build livox_ros_driver -j$(nproc) && \
    catkin build -j$(nproc)

# Copy preflight check script
COPY scripts/preflight_check_docker.sh /root/preflight_check.sh
RUN chmod +x /root/preflight_check.sh

# Setup entrypoint
COPY scripts/docker_entrypoint.sh /root/docker_entrypoint.sh
RUN chmod +x /root/docker_entrypoint.sh

# Source ROS workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/slam_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export ROS_HOSTNAME=\${ROS_HOSTNAME:-localhost}" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=\${ROS_MASTER_URI:-http://localhost:11311}" >> /root/.bashrc

# Set working directory
WORKDIR /root/slam_ws

# Expose ROS master port and MAVRos port
EXPOSE 11311 5760

# Set entrypoint
ENTRYPOINT ["/root/docker_entrypoint.sh"]
CMD ["bash"]
