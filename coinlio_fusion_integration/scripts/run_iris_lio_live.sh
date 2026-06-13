#!/bin/bash
# IRIS-LIO LIVE bringup on Jetson Orin NX / Ouster OS1-64 (192.168.2.60).
# IRIS-LIO = EllipseLIO geometry + COIN-LIO LiDAR-intensity photometric residual, fused in
# one IKFoM update. Runs in the coinlio_fusion container (the photometric build), driver +
# node + Foxglove all in-container on host-net DDS (CycloneDDS holds 20Hz; Fast-DDS stalls
# the organized-cloud publish on this rig).
#
# Adapted from ellipselio_integration/scripts/run_ellipselio.sh, but:
#   - targets the EXISTING coinlio_fusion container (has photometric code), not a fresh one
#   - photometric is ON (os1_64_ouster.yaml: enable=true, photo_scale=1e-9)
#   - NO odom_to_path.py (ellipselio now publishes /ellipselio_path natively)
#
# Prereq: the Ouster must be FREE (this script assumes slam_gpu_system/superodom are stopped).
# Keep the CPU boosted for the photometric pipeline's scheduling headroom:
#   sudo ~/superodom_ws/set_cpu_clock.sh boost   (see jetson-cpu-clock-boost)
set -e
CTR=coinlio_fusion
SENSOR_IP=192.168.2.60
HOST_IP=192.168.2.50
WS=$HOME/coinlio_fusion_ws
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DDS='export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0; source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'

# Stage Foxglove params + cloud throttle into the mounted workspace (-> /root/ros2_ws/).
cp "$HERE/../../ellipselio_integration/config/foxglove_params.yaml" "$WS/foxglove_params.yaml"
cp "$HERE/../../ellipselio_integration/scripts/cloud_throttle.py"   "$WS/cloud_throttle.py"

# 1. Clean DDS slate (the dev container persists; restart clears any stale nodes).
echo "[iris-live] restarting $CTR for a clean DDS slate..."
docker restart "$CTR" >/dev/null; sleep 6

# 2. ROS2 Ouster driver -> /ouster/points (20Hz, organized 64x1024, destaggered), /ouster/imu.
docker exec -d "$CTR" bash -lc "$DDS &&
  ros2 launch ouster_ros driver.launch.py \
    params_file:=/root/ros2_ws/src/ouster_os1_64_driver.yaml viz:=false"
echo "[iris-live] Ouster driver starting (sensor $SENSOR_IP -> udp_dest $HOST_IP); init ~25s..."
sleep 25

# 3. IRIS-LIO node. use_sim_time:=false is MANDATORY for live (default true = bag playback).
docker exec -d "$CTR" bash -lc "$DDS &&
  ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_path:=/root/ros2_ws/src/ellipselio/config \
    config_file:=os1_64_ouster.yaml \
    use_sim_time:=false rviz:=false"
echo "[iris-live] IRIS-LIO starting. Odometry on /ellipselio_odom; photometric images on /photometric/*."
sleep 8

# 4. Cloud throttle (/cloud_scan -> /cloud_scan_lite @5Hz; full scan ~63Mbps is too heavy remote).
docker exec -d "$CTR" bash -lc "$DDS && python3 /root/ros2_ws/cloud_throttle.py 5"

# 5. Foxglove bridge (port 8765), whitelisted. foxglove_params.yaml is sim-time -> force false.
docker exec -d "$CTR" bash -lc "$DDS &&
  ros2 run foxglove_bridge foxglove_bridge --ros-args \
    --params-file /root/ros2_ws/foxglove_params.yaml -p use_sim_time:=false -p max_qos_depth:=25"
echo
echo "[iris-live] Foxglove: connect (Foxglove WebSocket) to ws://$HOST_IP:8765   (fixed frame: odom_ellipselio)"
echo "  3D:    /cloud_scan_lite (dense @5Hz) + /cloud_map (Decay=0) + /ellipselio_path + /ellipselio_odom"
echo "  Image: /photometric/intensity_image  /photometric/feature_image"
echo "  Raw:   /analytics (watch obs_min/obs_score)"
echo "[iris-live] Verify: docker exec $CTR bash -lc 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic hz /ellipselio_odom'"
