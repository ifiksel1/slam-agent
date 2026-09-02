# GenZ-ICP (RA-L 2025, cocel-postech) — ROS1 Noetic, NATIVE arm64 for Jetson Orin NX.
#
# Base is ros:noetic-ros-base-focal (arm64 native — verified). Deliberately NOT
# osrf/ros:*-desktop-full (amd64-only -> qemu emulation trap) and NOT
# ros:noetic-perception (missing tf_conversions/TBB bits we hit on BIEVR-LIO).
#
# GenZ-ICP is LiDAR-only and pulls NO PCL / OpenCV / Ceres, so the OpenCV 4.2-vs-4.5
# ABI conflict does not apply here.
FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build toolchain. Eigen/Sophus/TBB/tsl-robin are pulled by the project's own
# FetchContent (3rdparty/*.cmake) — that is the configuration upstream actually
# tests, so we let it fetch rather than forcing system libs (focal ships Eigen
# 3.3.7 / TBB 2020.1; GenZ wants Eigen 3.4.90 / oneTBB 2021.8).
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget ca-certificates \
      python3-catkin-tools python3-osrf-pycommon \
      ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-rosbag \
      python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws/src
RUN git clone --depth 1 https://github.com/cocel-postech/genz-icp.git

WORKDIR /root/catkin_ws
RUN source /opt/ros/noetic/setup.bash && \
    catkin config --extend /opt/ros/noetic && \
    catkin build genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
