#!/bin/bash
# EllipseLIO live bringup on Jetson Orin NX 8GB / Ouster OS1-64.
#
# Prereqs:
#   - Image built: ellipselio:humble  (from ellipselio_integration/Dockerfile)
#   - Workspace at ~/ellipselio_ws/src with: ellipselio, ouster-ros (ros2 branch),
#     built via colcon (see build_ws.sh).
#   - The Ouster must be FREE — stop any other SLAM holding it:
#       docker stop superodom slam_gpu_system fast_livo2_slam_autostart 2>/dev/null
#
# Host is Ubuntu 20.04 (Noetic only) -> ROS2 runs ONLY in this container.
set -e
CTR=ellipselio
SENSOR_IP=192.168.2.60
HOST_IP=192.168.2.50
CONFIG=${1:-/root/ros2_ws/src/ellipselio/config/os1_64_ouster.yaml}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Stage the Foxglove params + helper nodes into the mounted workspace so the container
# sees them at /root/ros2_ws/ (same pattern run_bag.sh uses for the sampler).
cp "$HERE/../config/foxglove_params.yaml" "$HOME/ellipselio_ws/foxglove_params.yaml"
cp "$HERE/odom_to_path.py"               "$HOME/ellipselio_ws/odom_to_path.py"
cp "$HERE/cloud_throttle.py"             "$HOME/ellipselio_ws/cloud_throttle.py"

# 1. Start the container. host net so the in-container ROS2 driver reaches the sensor.
#    --init: tini as PID 1 reaps killed nodes instead of leaving <defunct> zombies.
#    Mount the WHOLE workspace so build/ + install/ persist on the host across restarts.
docker rm -f "$CTR" 2>/dev/null || true
docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$HOME/ellipselio_ws:/root/ros2_ws" \
  ellipselio:humble sleep infinity

# 2. ROS2 Ouster driver (publishes /ouster/points 20Hz, /ouster/imu ~100Hz).
#    CycloneDDS holds 20Hz; default Fast-DDS stalls the large cloud publish (proven on this rig).
docker exec -d "$CTR" bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  ros2 launch ouster_ros driver.launch.py \
    params_file:=/root/ros2_ws/src/ouster_os1_64_driver.yaml viz:=false'
echo "Ouster driver starting (sensor $SENSOR_IP -> udp_dest $HOST_IP); waiting for init..."
sleep 25

# 3. EllipseLIO. use_sim_time:=false is MANDATORY for live sensors (default true = bag playback).
docker exec -d "$CTR" bash -lc "
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_path:=/root/ros2_ws/src/ellipselio/config \
    config_file:=os1_64_ouster.yaml \
    use_sim_time:=false rviz:=false"
echo "EllipseLIO starting. Odometry on /ellipselio_odom (nav_msgs/Odometry, IMU rate)."
sleep 8

# 4. Viz helpers: odom->Path republisher (/ellipselio_path; EllipseLIO has no Path)
#    and /cloud_scan throttle (/cloud_scan_lite @5Hz; full /cloud_scan is ~63Mbps,
#    too heavy for a remote Foxglove link).
docker exec -d "$CTR" bash -lc "
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  python3 /root/ros2_ws/odom_to_path.py"
docker exec -d "$CTR" bash -lc "
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  python3 /root/ros2_ws/cloud_throttle.py 5"

# 5. Foxglove bridge (port 8765), WHITELISTED. The raw /ouster/points organized cloud
#    (~480Mbps) saturates remote links and stalls the websocket, so the bridge streams
#    only the viz topics (foxglove_params.yaml). Connect Foxglove to ws://<jetson-ip>:8765.
#    NOTE: foxglove_params.yaml has use_sim_time:true (bag); for LIVE, set it false.
docker exec -d "$CTR" bash -lc "
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  ros2 run foxglove_bridge foxglove_bridge --ros-args \
    --params-file /root/ros2_ws/foxglove_params.yaml -p use_sim_time:=false"
echo
echo "Foxglove: connect to ws://<jetson-ip>:8765  (fixed frame: odom_ellipselio)"
echo "  topics: /cloud_scan_lite (dense scan @5Hz)  /cloud_map (set Decay=0)  /ellipselio_path  /ellipselio_odom"
echo "Verify:  docker exec $CTR bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic hz /ellipselio_odom'"
echo "Map:     /cloud_map   Scan: /cloud_scan   Analytics: /analytics (watch ram_usage)"
