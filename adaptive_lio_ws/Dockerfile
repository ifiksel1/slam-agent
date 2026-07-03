# Adaptive-LIO (chengwei0427/adaptive_lio) — dedicated ROS1 Noetic image for the Dragunfly rig.
# Ouster OS1-64 + Cube Orange (ArduPilot). ARM64 (Jetson Orin NX) and x86_64.
#
# Why its own image (not the COIN-LIO one): Adaptive-LIO needs Ceres 2.1 + glog/gflags built
# from source and the livox_ros_driver CustomMsg messages as a COMPILE dep — heavier and
# version-pinned, so it gets an isolated container (name: adaptive_lio, image adaptive-lio:noetic).
# Sophus and tessil robin_map are VENDORED in the repo (thirdparty/), so they are NOT installed.
#
# Build:   docker build -f adaptive_lio_ws/Dockerfile -t adaptive-lio:noetic adaptive_lio_ws
#   live Ouster driver too:  --build-arg BUILD_OUSTER_DRIVER=1
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
LABEL maintainer="SLAM Agent"
LABEL description="Adaptive-LIO (adaptive LiDAR-Inertial Odometry) for Ouster OS1-64 + ArduPilot"

# ---- system + math + ROS build deps ----------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl ca-certificates \
    python3-catkin-tools python3-rosdep python3-osrf-pycommon \
    libeigen3-dev libpcl-dev libomp-dev libtbb-dev libboost-all-dev \
    libyaml-cpp-dev libopencv-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev libopenblas-dev liblapack-dev \
    ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-tf ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-eigen-conversions ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
 && geographiclib-get-geoids egm96-5 || true \
 && rm -rf /var/lib/apt/lists/*
# MAVROS geographiclib datasets (guarded above; ignore on minimal images without the helper)
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/geo.sh \
 && bash /tmp/geo.sh || true

# ---- Ceres 2.1.0 from source (Noetic apt ships 1.14 — too old for Adaptive-LIO) -------------
# README says "ceres 2.10"; the shipped tag family is 2.1.x. Pin 2.1.0 (2.2.0 also builds).
RUN cd /tmp && git clone --depth 1 -b 2.1.0 https://github.com/ceres-solver/ceres-solver.git \
 && cd ceres-solver && mkdir build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
 && make -j"$(nproc)" && make install && ldconfig \
 && rm -rf /tmp/ceres-solver

# ---- catkin workspace: livox_ros_driver (CustomMsg dep) + Adaptive-LIO ----------------------
ARG ADAPTIVE_LIO_REPO=https://github.com/chengwei0427/adaptive_lio.git
ARG ADAPTIVE_LIO_REF=main
ARG BUILD_OUSTER_DRIVER=0
ENV CATKIN_WS=/root/catkin_ws
WORKDIR ${CATKIN_WS}/src

# NOTE: livox_ros_driver::CustomMsg (referenced by cloud_convert's AviaHandler) is provided by a
# message-only package that Adaptive-LIO VENDORS at thirdparty/livox_ros_driver — the --recursive
# clone below pulls it in. Do NOT also clone livox_ros_driver into src/: two copies of the same
# package collide on the livox_ros_driver_generate_messages target (CMake add_custom_target error).
# Livox-SDK is likewise NOT needed — the vendored package builds messages only, no driver node.

# Adaptive-LIO source (--recursive pulls thirdparty/livox_ros_driver + sophus + tessil)
RUN git clone --recursive ${ADAPTIVE_LIO_REPO} adaptive_lio \
 && cd adaptive_lio && git checkout ${ADAPTIVE_LIO_REF}

# Rename hardcoded frames -> alio_* so they don't collide with MAVROS map / base_link, and bake
# the Ouster config over the compile-time default config/mapping_m.yaml.
COPY patches/namespace_frames.sh /tmp/namespace_frames.sh
COPY patches/fix_ouster_ring.py  /tmp/fix_ouster_ring.py
COPY patches/fix_ouster_timespan.py /tmp/fix_ouster_timespan.py
COPY patches/fix_queue_cap.py    /tmp/fix_queue_cap.py
COPY config/mapping_ouster.yaml  /tmp/mapping_ouster.yaml
RUN bash /tmp/namespace_frames.sh ${CATKIN_WS}/src/adaptive_lio \
 && cp /tmp/mapping_ouster.yaml ${CATKIN_WS}/src/adaptive_lio/config/mapping_m.yaml \
 # lidar_buffer_ (input queue) is unbounded -> backlog balloons RAM when processing < 20Hz ingress.
 # Cap it to a rolling 40-scan window (drop oldest under overload) to hard-bound memory. (Fable dx.)
 && python3 /tmp/fix_queue_cap.py ${CATKIN_WS}/src/adaptive_lio \
 # ouster_ros::Point.ring is uint8_t upstream but modern Ouster drivers / this rig's bags publish
 # ring as UINT16 -> PCL field-type mismatch -> garbage ring -> SIGSEGV on the first cloud. Fix to uint16.
 && python3 /tmp/fix_ouster_ring.py ${CATKIN_WS}/src/adaptive_lio \
 # Oust64Handler derives sweep timespan from points.back().t (wrong for organized clouds) and has a
 # broken id clamp -> out-of-bounds v_cloud write -> SIGSEGV. Use max-t + hard clamp id.
 && python3 /tmp/fix_ouster_timespan.py ${CATKIN_WS}/src/adaptive_lio \
 # Upstream CMakeLists hardcodes -std=c++14, but eigen_types.h uses a constexpr captureless lambda
 # (less_vec2i) that is only a literal type from C++17 on. The code is really C++17 — bump it.
 && sed -i 's/c++14/c++17/g; s/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/g' \
      ${CATKIN_WS}/src/adaptive_lio/CMakeLists.txt

# Optional: Ouster ROS1 driver (only needed for LIVE sensor; bag replay does not need it).
RUN if [ "${BUILD_OUSTER_DRIVER}" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-pcl-ros libpcap-dev libtins-dev libglfw3-dev libglew-dev && \
      git clone --depth 1 -b ROS1 https://github.com/ifiksel1/ouster-ros.git ${CATKIN_WS}/src/ouster-ros --recurse-submodules && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

# vision_to_mavros bridge (nav_msgs/Odometry|TF -> /mavros/vision_pose/pose) for the full stack
RUN git clone --depth 1 https://github.com/thien94/vision_to_mavros.git ${CATKIN_WS}/src/vision_to_mavros

# ---- build ----------------------------------------------------------------
# -O3 (not -Ofast) on aarch64: -Ofast's fast-math reassociation has tipped EKF divergence on this
# Jetson before (see EllipseLIO lessons). Limit parallelism on the 8 GB module to avoid OOM.
WORKDIR ${CATKIN_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3" -j2 \
 && echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
