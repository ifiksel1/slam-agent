# LI-Init (hku-mars/LiDAR_IMU_Init) — LiDAR<->IMU extrinsic + time-offset calibration.
#
# DELIBERATELY A SEPARATE IMAGE from the flight container (slam-hesai-fastlio).
# LI-Init needs Ceres 2.0.0 built from source; the flight container runs the live
# vision->FC pipeline and must not gain heavyweight build deps. This image is used
# OFFLINE, replaying a recorded bag. It never talks to the flight controller.
#
# Build:  docker build -t li-init:latest docker/li_init/
# Run:    scripts/li_init/run_li_init.sh <bag>
FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget ca-certificates \
      libeigen3-dev libpcl-dev libgoogle-glog-dev libgflags-dev \
      libatlas-base-dev libsuitesparse-dev \
      ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
      python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Ceres 2.0.0 from source. Noetic/Focal ships 1.14 via libceres-dev, but LI-Init is
# tested against 2.0.0 and the 1.14 API differs enough to break the build.
ARG CERES_VERSION=2.0.0
RUN git clone --depth 1 --branch ${CERES_VERSION} \
      https://github.com/ceres-solver/ceres-solver.git /tmp/ceres \
    && cmake -S /tmp/ceres -B /tmp/ceres/build \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/ceres/build -j"$(nproc)" \
    && cmake --install /tmp/ceres/build \
    && rm -rf /tmp/ceres

WORKDIR /root/li_ws/src

# livox_ros_driver is a hard build dependency even for non-Livox sensors: LI-Init
# includes its custom message headers unconditionally.
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone --depth 1 https://github.com/hku-mars/LiDAR_IMU_Init.git

# Record the exact upstream commits actually built, so a calibration result can be
# traced back to the code that produced it.
RUN { echo "livox_ros_driver  $(git -C livox_ros_driver rev-parse HEAD)"; \
      echo "LiDAR_IMU_Init    $(git -C LiDAR_IMU_Init  rev-parse HEAD)"; \
      echo "ceres             ${CERES_VERSION}"; } > /root/li_ws/BUILD_VERSIONS.txt

RUN source /opt/ros/noetic/setup.bash \
    && cd /root/li_ws \
    && catkin_make -j"$(nproc)" -DCMAKE_BUILD_TYPE=Release

# Our JT128 config is mounted at run time (see run_li_init.sh) rather than baked in,
# so it can be edited and re-run without rebuilding the image.
RUN mkdir -p /root/li_ws/src/LiDAR_IMU_Init/result

RUN echo 'source /opt/ros/noetic/setup.bash; source /root/li_ws/devel/setup.bash' >> /root/.bashrc
WORKDIR /root/li_ws
CMD ["bash"]
