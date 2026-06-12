#!/bin/bash
# Foxglove demo of the COIN-LIO photometric fusion on the apartment bag.
#
# Starts a PERSISTENT whitelisted foxglove_bridge, then LOOPS: relaunch the EllipseLIO
# node fresh + play the bag once. (A persistent node + `ros2 bag play --loop` is NOT
# usable: sim-time jumps backward at loop restart and the node floods "Imu time out of
# order" and stops. Relaunching per cycle keeps each node lifetime monotonic.)
#
# Connect Foxglove ("Open connection" -> Foxglove WebSocket) to ws://<host-ip>:8765 and
# add Image panels on /photometric/intensity_image and /photometric/feature_image, a 3D
# panel on /cloud_map + /ellipselio_odom (fixed frame: odom or camera_init per tf), and a
# Raw-Message panel on /analytics (watch obs_score).
#
# Runs in the foreground (Ctrl-C to stop), or launch in the background and stop later.
set -uo pipefail
CTR=coinlio_fusion
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker restart "$CTR" >/dev/null 2>&1; sleep 6
cp "$HERE/../../ellipselio_integration/config/foxglove_params.yaml" \
   "$HOME/coinlio_fusion_ws/foxglove_params.yaml"

docker exec "$CTR" bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
  CFGDIR=/root/ros2_ws/src/ellipselio/config
  BAG=/root/bags/apartment_20260611_034239

  # Viz-only frame: the map is flipped 180 deg in PITCH (about Y) vs LiDAR-forward, from the
  # IMU<-LiDAR mounting. Publish a static odom_ellipselio -> map_lidar (180 deg pitch) so
  # Foxglove renders the map LiDAR-forward by setting fixed frame = map_lidar. No effect on
  # the SLAM (TF only). No leading slash -> tf2 requires it (Foxglove strips the published one).
  ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --yaw 0 --pitch 3.14159265 --roll 0 \
      --frame-id odom_ellipselio --child-frame-id map_lidar > /tmp/static_tf.log 2>&1 &

  # IMPORTANT (QoS): EllipseLIO publishes its viz topics BEST_EFFORT (SensorDataQoS).
  # foxglove_bridge auto-detects a topic`s QoS from a LIVE publisher when it first
  # subscribes -- if the bridge starts before any publisher exists, it defaults to
  # RELIABLE and silently drops every best-effort topic (you only see /tf). So the bridge
  # must be started AFTER the node is publishing, i.e. a few seconds into the first bag
  # play. (This is the ordering the production run_ellipselio.sh uses.)
  bridge_started=0

  # loop: fresh node + one bag play per cycle (a persistent node + bag re-play breaks on
  # the sim-time jump at loop restart -> relaunch per cycle keeps each lifetime monotonic).
  while true; do
    ros2 launch ellipselio ellipselio_standalone.launch.py \
        config_path:=$CFGDIR config_file:=os1_64_ouster.yaml \
        use_sim_time:=true rviz:=false > /tmp/ell_demo.log 2>&1 &
    sleep 8
    echo "[demo] playing apartment bag (photometric fusion)..."
    ros2 bag play "$BAG" --clock --rate 1.0 > /tmp/play_demo.log 2>&1 &
    BAGPID=$!
    if [ $bridge_started -eq 0 ]; then
      sleep 8   # let the best-effort publishers come up under /clock
      ros2 run foxglove_bridge foxglove_bridge --ros-args \
          --params-file /root/ros2_ws/foxglove_params.yaml \
          -p use_sim_time:=true -p max_qos_depth:=25 \
          > /tmp/foxglove_bridge.log 2>&1 &
      bridge_started=1
      echo "[demo] foxglove_bridge up on :8765 (QoS auto-detected from live publishers)"
    fi
    wait $BAGPID
    echo "[demo] bag done; relaunching node for next cycle"
    for pid in $(ps -eo pid,comm | awk "/component_container/{print \$1}"); do kill -INT $pid; done
    sleep 3
  done
'
