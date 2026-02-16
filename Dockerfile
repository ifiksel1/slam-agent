# Complete SLAM System Docker Image - FAST-LIO2 + STD Loop-Closure + Ouster + MAVRos
# Optimized for ARM64 (Jetson) and x86_64
# Based on Ubuntu 20.04 with ROS1 Noetic

FROM ros:noetic-ros-base-focal

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Set up labels
LABEL maintainer="SLAM Agent"
LABEL description="Complete SLAM system: FAST-LIO2, STD loop-closure, Ouster OS1-64, MAVRos (ArduPilot 4.6.2)"
LABEL version="2.0"

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build essentials
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # ROS build tools
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # SLAM dependencies
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    libtbb-dev \
    # GTSAM dependencies (pose graph optimization)
    libopenblas-dev \
    liblapack-dev \
    # Ouster driver dependencies
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-eigen-conversions \
    # MAVRos dependencies (ArduPilot connectivity)
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    # Networking tools
    net-tools \
    iputils-ping \
    iproute2 \
    # Debugging tools
    vim \
    htop \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# Create workspace
WORKDIR /root/slam_ws/src

# Clone repositories
RUN git clone --recursive https://github.com/hku-mars/FAST_LIO_SLAM.git && \
    git clone https://github.com/hku-mars/STD.git && \
    git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
    cd ouster-ros && git checkout 20220826_v0.9.0 && cd .. && \
    git clone https://github.com/thien94/vision_to_mavros.git

# Fix googletest CMake compatibility (common issue on Ubuntu 20.04)
RUN if [ -f /usr/src/googletest/CMakeLists.txt ]; then \
        sed -i 's/cmake_minimum_required(VERSION 2.8.3)/cmake_minimum_required(VERSION 3.5)/' /usr/src/googletest/CMakeLists.txt; \
    fi

# Install GTSAM 4.2a9 for pose graph optimization (required by STD)
WORKDIR /tmp
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2a9 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf gtsam

# Install GeographicLib datasets for MAVRos
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install dependencies via rosdep
WORKDIR /root/slam_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace with all components
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build fast_lio std ouster_ros mavros mavros_extras vision_to_mavros -j$(nproc)

# Copy SLAM configuration with corrected topics
COPY config/ouster64.yaml /root/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml

# Copy launch files
COPY launch/mapping_ouster64_docker.launch /root/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/launch/

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
