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

  # 1. persistent, whitelisted bridge (stays connected across bag cycles)
  ros2 run foxglove_bridge foxglove_bridge --ros-args \
      --params-file /root/ros2_ws/foxglove_params.yaml -p use_sim_time:=true \
      > /tmp/foxglove_bridge.log 2>&1 &
  sleep 3
  echo "[demo] foxglove_bridge up on :8765"

  # 2. loop: fresh node + one bag play per cycle
  while true; do
    ros2 launch ellipselio ellipselio_standalone.launch.py \
        config_path:=$CFGDIR config_file:=os1_64_ouster.yaml \
        use_sim_time:=true rviz:=false > /tmp/ell_demo.log 2>&1 &
    sleep 8
    echo "[demo] playing apartment bag (photometric fusion)..."
    ros2 bag play "$BAG" --clock --rate 1.0 > /tmp/play_demo.log 2>&1
    echo "[demo] bag done; relaunching node for next cycle"
    for pid in $(ps -eo pid,comm | awk "/component_container/{print \$1}"); do kill -INT $pid; done
    sleep 3
  done
'
