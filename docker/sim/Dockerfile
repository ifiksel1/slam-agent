# Mission simulation (L4 autonomy, D11): ROG-Map + SUPER + MARSIM on ROS1 Noetic.
#
# DELIBERATELY A SEPARATE IMAGE from the flight container (slam-hesai-fastlio).
# It runs on the sim host (Shannon), never on the robot's NUC: MARSIM renders the
# lidar with OpenGL, here Mesa software rendering under Xvfb (no GPU runtime needed).
# It never talks to a real flight controller; in closed loop it talks only to
# QuadRover SITL.
#
# SUPER has no licence (all rights reserved): internal use only, never shipped (D2).
#
# Build:  docker build -t l4-sim:latest docker/sim/
# Smoke:  docker run --rm l4-sim:latest glxinfo -B
FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# SUPER's README dependencies, plus Xvfb + Mesa for a headless OpenGL context
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-catkin-tools \
    libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev \
    libeigen3-dev libdw-dev \
    ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-pcl-ros ros-noetic-rosfmt \
    xvfb mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

# SUPER (bundles ROG-Map and MARSIM's marsim_render / perfect_drone_sim), pinned
ARG SUPER_SHA=2ad3419c127a617c6d7df6925e81a14175a9c096
RUN mkdir -p /super_ws/src && cd /super_ws/src \
    && git clone https://github.com/hku-mars/SUPER.git \
    && cd SUPER && git checkout ${SUPER_SHA} \
    && bash scripts/select_ros_version.sh ROS1

# The simulated lidar cropped to the JT128's forward dome (see patch_dome_crop.py)
COPY patch_dome_crop.py /tmp/patch_dome_crop.py
RUN python3 /tmp/patch_dome_crop.py /super_ws/src/SUPER/mars_uav_sim/perfect_drone_sim/include/perfect_drone_sim/ros1_perfect_drone_model.hpp
# SUPER aborted on long straight path segments, e.g. a vertical climb (see patch_seed_line.py)
COPY patch_seed_line.py /tmp/patch_seed_line.py
RUN python3 /tmp/patch_seed_line.py /super_ws/src/SUPER/super_planner/src/super_core/corridor_generator.cpp

RUN source /opt/ros/noetic/setup.bash && cd /super_ws \
    && catkin_make -DBUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release -j12

# A virtual display for MARSIM's GLFW window; every command runs with it
ENV DISPLAY=:99
RUN printf '#!/bin/bash\nXvfb :99 -screen 0 1280x1024x24 -nolisten tcp >/dev/null 2>&1 &\nsleep 1\nsource /opt/ros/noetic/setup.bash\nsource /super_ws/devel/setup.bash\nexec "$@"\n' > /entrypoint.sh \
    && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
