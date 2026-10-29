# L4 planner stack for the robot's NUC: ROG-Map + SUPER (no MARSIM, no Xvfb) and our C++ nodes (planner/ros):
# the planner bridge, the mission executive, the track planner and the landing check.
#
# Runs beside slam-hesai-fastlio on the companion, pinned to CPUs 8-11 (FAST-LIO owns 0,2,4,6).
# SUPER has no licence (all rights reserved): internal use only, never shipped (D2).
#
# Build (context = a staged folder, see docker/planner/stage.sh; the repo root holds multi-GB bags):
#   docker build -t l4-planner:latest <stage>
FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-catkin-tools python3-yaml python3-numpy \
    libeigen3-dev libdw-dev libncurses5-dev libncursesw5-dev \
    ros-noetic-mavros-msgs ros-noetic-pcl-ros ros-noetic-rosfmt libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen

# SUPER (bundles ROG-Map), pinned as the sim image
ARG SUPER_SHA=2ad3419c127a617c6d7df6925e81a14175a9c096
RUN mkdir -p /super_ws/src && cd /super_ws/src \
    && git clone https://github.com/hku-mars/SUPER.git \
    && cd SUPER && git checkout ${SUPER_SHA} \
    && bash scripts/select_ros_version.sh ROS1 \
    && touch mars_uav_sim/marsim_render/CATKIN_IGNORE mars_uav_sim/perfect_drone_sim/CATKIN_IGNORE mission_planner/CATKIN_IGNORE

# SUPER aborted on long straight path segments, e.g. a vertical climb (see docker/sim/patch_seed_line.py)
COPY patch_seed_line.py /tmp/patch_seed_line.py
RUN python3 /tmp/patch_seed_line.py /super_ws/src/SUPER/super_planner/src/super_core/corridor_generator.cpp

RUN source /opt/ros/noetic/setup.bash && cd /super_ws \
    && catkin_make -DBUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release -j8

# the robot's SUPER config (docker/sim/config/super_revolute.yaml) and our nodes
COPY super_revolute.yaml /super_ws/src/SUPER/super_planner/config/super_revolute.yaml
COPY planner /opt/planner

# the C++ planner nodes (planner/ros: l4_track = track_planner, l4_landing = landing_check, l4_bridge =
# planner_bridge, l4_executive = mission_executive, l4_common = their shared headers, l4_bringup = the
# launch files), built against SUPER's quadrotor_msgs and installed to /opt/l4_ws (roslaunch l4_bringup
# l4_planner.launch; rosrun l4_track track_planner, ...); their tests are not staged (stage.sh), so testing
# is off here. cp -L: l4_bringup/config/planner_bridge.yaml links to planner/config/planner_bridge.yaml
RUN mkdir -p /l4_ws/src \
    && cp -rL /opt/planner/ros/l4_track /opt/planner/ros/l4_landing /opt/planner/ros/l4_common /opt/planner/ros/l4_bridge \
       /opt/planner/ros/l4_executive /opt/planner/ros/l4_bringup /l4_ws/src/ \
    && source /opt/ros/noetic/setup.bash && source /super_ws/devel/setup.bash && cd /l4_ws \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/opt/l4_ws -j8 install \
    && rm -rf /l4_ws

RUN printf '#!/bin/bash\nsource /opt/ros/noetic/setup.bash\nsource /super_ws/devel/setup.bash\nsource /opt/l4_ws/setup.bash\nexec "$@"\n' > /entrypoint.sh \
    && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
