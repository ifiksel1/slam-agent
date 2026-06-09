# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

# NOTE: switched from osrf/ros:humble-desktop-full (amd64-ONLY -> built under QEMU
# on aarch64) to the official multi-arch ros:humble-ros-base, which has a native
# arm64 manifest. All desktop deps SuperOdom needs (rviz2, pcl_ros, cv_bridge,
# image_transport, grid_map...) are apt-installed explicitly below.
FROM ros:humble-ros-base

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
    
# Install PCL & Eigen & essential libraries
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        sudo \
        vim \
        wget \
        tmux \
        curl \
        less \
        htop \
        libsm6 libxext6 libgl1-mesa-glx libxrender-dev \ 
        curl \
 && apt-get clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean
 
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros

RUN apt-get install -y libceres-dev 
    

RUN apt-get install -y python-tk python3-pip

# Python 3.
RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir numpy scipy matplotlib ipython opencv-python || true
# Dropped numba / scikit-image / scikit-learn / visdom: no aarch64 wheels (build
# from source needs LLVM/llvmlite) and unused by SuperOdom's C++ odometry nodes.
# Kept pandas/Pillow (have arm64 wheels); all made non-fatal for ARM64 robustness.
RUN pip3 install --no-cache-dir pandas Pillow || true
RUN pip3 install --no-cache-dir pyyaml pycryptodome bagpy Cython || true

RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# RUN sudo rosdep init
RUN rosdep update
RUN pip3 install -U catkin_tools gym
RUN apt-get -y install ros-humble-grid-map-msgs ros-humble-grid-map

# Install livox SDK
WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j2 && make install

WORKDIR /root/
RUN git clone http://github.com/strasdat/Sophus.git \
    && cd Sophus && git checkout 97e7161 \
    && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF \
    && make -j4 && sudo make install
WORKDIR /root/
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && git checkout 4abef92 &&\
    mkdir build && \
    cd build && \
    cmake  \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
           .. && \
    make -j4 && \
    sudo make install


RUN /bin/bash -c '. /opt/ros/humble/setup.bash;'

RUN pip install rerun-sdk || true
RUN pip install numpy==1.24.3 matplotlib==3.7.1 || true
RUN pip install --upgrade packaging || true
# torch removed: not needed by SuperOdom's C++ odometry nodes; the aarch64 wheel
# is a large/fragile download on Jetson. Re-add if a python analysis script needs it.
# RUN pip3 install torch --ignore-installed

# --- ouster_ros (ROS 2) build deps, baked in so the driver builds in-container ---
# Appended after the heavy Sophus/GTSAM layers so those stay cached on rebuild.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
        libtins-dev libpcap-dev libflatbuffers-dev libzip-dev \
        ros-humble-pcl-ros ros-humble-tf2-eigen ros-humble-rviz2 \
        ros-humble-ament-cmake ros-humble-ament-lint-auto ros-humble-ament-cmake-gtest \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-foxglove-bridge \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# CRITICAL (2026-06-09): default Fast-DDS (rmw_fastrtps_cpp) stalls the large /ouster/points
# PointCloud2 publish when multiple SuperOdom DDS participants are active -> /ouster/points
# collapses to ~15Hz regardless of CPU/clock/QoS. CycloneDDS holds a steady 20Hz. Make it the
# default rmw for this image. Proven by A/B on Orin NX 8GB / OS1-64.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /root/ros2_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

























