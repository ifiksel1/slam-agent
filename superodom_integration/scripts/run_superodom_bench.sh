#!/bin/bash
# SuperOdom (super_odometry) live bench bringup on Jetson Orin NX 8GB / Ouster OS1-64.
# Validated 2026-06-09: build -> live ROS2 Ouster driver -> SuperOdom, 25Hz /state_estimation.
#
# Prereqs:
#   - Image built: superodom:humble  (from superodom_integration/Dockerfile)
#   - Workspace at ~/superodom_ws/src with: SuperOdom, livox_ros_driver2,
#     rviz_2d_overlay_plugins, ouster-ros (ros2 branch), built via colcon (unified,
#     -DROS_EDITION=ROS2, libzip-dev present).
#   - FAST-LIVO2 (ROS1) STOPPED so the Ouster is free:
#       docker update --restart=no fast_livo2_slam_autostart && docker stop fast_livo2_slam_autostart
#
# Host is Ubuntu 20.04 (Noetic only) -> ROS2 runs ONLY in this container.
set -e
CTR=superodom
SENSOR_IP=192.168.2.60
HOST_IP=192.168.2.50

# 1. Start the container (host net so the in-container ROS2 driver reaches the sensor)
docker rm -f "$CTR" 2>/dev/null || true
docker run -d --name "$CTR" --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 \
  -v "$HOME/superodom_ws:/root/ros2_ws" \
  superodom:humble sleep infinity
# NOTE: mount the WHOLE workspace (not just src/) so build/ + install/ persist on the
# host — otherwise a container restart loses the colcon build and needs a full rebuild.

# 2. ROS2 Ouster driver (publishes /ouster/points ~18Hz, /ouster/imu ~100Hz)
docker exec -d "$CTR" bash -lc '
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  ros2 launch ouster_ros driver.launch.py \
    params_file:=/root/ros2_ws/src/ouster_os1_64_driver.yaml viz:=false'
echo "Ouster driver starting (sensor $SENSOR_IP -> udp_dest $HOST_IP); waiting for init..."
sleep 25

# 3. SuperOdom (3 nodes: feature_extraction, laser_mapping, imu_preintegration)
docker exec -d "$CTR" bash -lc '
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  ros2 launch super_odometry os1_128.launch.py \
    config_file:=/root/ros2_ws/src/SuperOdom/super_odometry/config/os1_64.yaml \
    calibration_file:=/root/ros2_ws/src/SuperOdom/super_odometry/config/ouster/os1_64_calibration.yaml'
echo "SuperOdom launched."
echo
echo "Verify:  docker exec $CTR bash -lc 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic hz /state_estimation'"
echo "Health:  ... ros2 topic echo /state_estimation_health --once   (expect data: true)"
echo "Outputs: /state_estimation (Odometry, ~25Hz)  /state_estimation_health  /super_odometry_stats  TF map->sensor"
