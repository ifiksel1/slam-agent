# EllipseLIO (adaptive ellipsoid LiDAR-Inertial Odometry) — ROS 2 Humble, ARM64/Jetson.
#
# Far lighter than the SuperOdom image: EllipseLIO vendors IKFoM + i-Octree in include/,
# so there is NO GTSAM / Ceres / Sophus / Livox-SDK to source-build. We only need a C++17
# toolchain, OpenMP, Eigen, PCL, cv_bridge, pcl_ros, plus the Ouster ROS2 driver deps.
#
# Mirrors the proven SuperOdom container pattern on this rig:
#   - ros:humble-ros-base (native arm64 manifest, no QEMU)
#   - CycloneDDS as default rmw (Fast-DDS stalls the large /ouster/points publish on Orin NX)
#   - host workspace mounted at /root/ros2_ws; colcon build runs in-container
FROM ros:humble-ros-base

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Base tooling
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl vim less htop tmux sudo \
        libsm6 libxext6 libgl1-mesa-glx libxrender-dev \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Timezone (non-interactive)
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update && apt-get install -y --no-install-recommends tzdata \
 && ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# EllipseLIO build deps: OpenMP (-fopenmp throughout), Eigen, PCL, OpenCV (camera path),
# pcl_ros + cv_bridge + tf2, rviz/robot_state_publisher for visualization.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libomp-dev libeigen3-dev libpcl-dev libgoogle-glog-dev \
        ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
        ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-pcl-ros \
        ros-humble-xacro ros-humble-robot-state-publisher \
        ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins \
        python3-pip python3-rosdep python3-colcon-common-extensions \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Ouster ROS2 driver build deps (driver is built in-workspace alongside EllipseLIO)
RUN apt-get update && apt-get install -y --no-install-recommends \
        libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
        libtins-dev libpcap-dev libflatbuffers-dev libzip-dev \
        ros-humble-ament-cmake ros-humble-ament-lint-auto ros-humble-ament-cmake-gtest \
        ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy scipy matplotlib pyyaml || true

# CRITICAL (per SuperOdom A/B on this rig): default Fast-DDS stalls the large
# /ouster/points PointCloud2 publish to ~15Hz when multiple DDS participants are
# active. CycloneDDS holds a steady 20Hz. Make it the default rmw for this image.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /root/ros2_ws

COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
