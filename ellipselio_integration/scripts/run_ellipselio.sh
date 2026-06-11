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
echo
echo "Verify:  docker exec $CTR bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic hz /ellipselio_odom'"
echo "Map:     /cloud_map   Scan: /cloud_scan   Analytics: /analytics (watch ram_usage)"
