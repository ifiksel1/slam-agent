#!/bin/bash
# Replay the degeneracy bag through SuperOdom and view it in Foxglove — repeatable, no flicker.
#
# Each run: (1) kills any prior SuperOdom/bag via superodom_cleanup.sh (the anti-flicker guard),
# (2) ensures the foxglove_bridge (:8765) + map_view flip TF are up, (3) starts ONE fresh SuperOdom
# (empty map), (4) plays the bag ONCE from t=0 so the map builds from scratch.
#
# NOTE: the bag is played WITHOUT --loop on purpose. On loop, the bag's timestamps jump backward
# and SuperOdom's feature_extraction rejects every scan ("sync unsuccessful") — so loops 2+ produce
# nothing. Re-run this script to replay from scratch.
#
# Connect Foxglove to ws://<jetson-wifi-ip>:8765. In the 3D panel: Fixed frame = map_view,
# add /laser_cloud_map (the SLAM map, ~1Hz), /registered_scan (live sweep), /laser_odom_path.
set -euo pipefail
CTR="${1:-superodom}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW="rmw_cyclonedds_cpp"   # CRITICAL: Fast-DDS stalls /ouster/points; cyclone holds 20Hz
BAG="/root/ros2_ws/degen_bag"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
SO_CFG=/root/ros2_ws/src/SuperOdom/super_odometry/config
LAUNCH="ros2 launch super_odometry os1_128.launch.py config_file:=$SO_CFG/os1_64.yaml calibration_file:=$SO_CFG/ouster/os1_64_calibration.yaml"

docker ps --format '{{.Names}}' | grep -qx "$CTR" || { echo "container '$CTR' not running"; exit 1; }

# (1) anti-flicker guard — kill any prior SuperOdom + bag
bash "$HERE/superodom_cleanup.sh" "$CTR"

# (2) ensure foxglove bridge + map_view flip TF are up (idempotent — start only if missing)
docker exec "$CTR" bash -c "pgrep -f foxglove_bridge/foxglove_bridge >/dev/null" || {
  echo "starting foxglove_bridge :8765"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > /tmp/fox_bridge.txt 2>&1"
}
docker exec "$CTR" bash -c "pgrep -f 'static_transform_publisher.* map map_view' >/dev/null" || {
  echo "starting map_view flip TF"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 run tf2_ros static_transform_publisher 0 0 0 1 0 0 0 map map_view > /tmp/fox_tf.txt 2>&1"
}

# (3) fresh SuperOdom (empty map)
echo "starting fresh SuperOdom"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && $LAUNCH > /tmp/fox_superodom.txt 2>&1"
sleep 7
n=$(docker exec "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 node list 2>/dev/null" | grep -cE 'feature_extraction|laser_mapping|imu_preintegration' || true)
echo "SuperOdom nodes up: $n (want 3)"

# (4) play bag once from t=0
echo "playing bag from t=0 (~176s; map builds ~45s in)"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 bag play $BAG --rate 1.0 > /tmp/fox_bagplay.txt 2>&1"
sleep 4
docker exec "$CTR" bash -c "pgrep -f 'bag play' >/dev/null" && echo "REPLAY ACTIVE — connect Foxglove to ws://<jetson-ip>:8765, add /laser_cloud_map" || echo "ERROR: bag not playing"
