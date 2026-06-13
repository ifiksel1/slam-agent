#!/bin/bash
# Foxglove viewer for STANDALONE COIN-LIO (ROS1 Noetic, ~/coinlio_ws) on the degenerate bag.
# COIN-LIO is ROS1, so Foxglove connects via ROSBRIDGE (not the ROS2 foxglove_bridge):
#
#   Foxglove -> "Open connection" -> Rosbridge -> ws://<host-ip>:9090
#     - 3D panel:   /Laser_map (+ /cloud_registered) + /Odometry   (fixed frame: camera_init)
#     - Image:      /filtered_image (LiDAR-intensity image), /feature_image
#     - Raw msg:    /Odometry
#
# Design (mirrors foxglove_degen_demo.sh): roscore + rosbridge are PERSISTENT; each cycle
# relaunches the coin_lio node + a fresh bag play, then kills ONLY the node (so the sim-time
# /clock restart doesn't break a long-lived node -> "Imu time out of order"). ROS1 nodes have
# stable names, so killall-by-name is clean (no ps -o comm 15-char-truncation trap).
#
# REQUIRES the ROS2 degen bag converted to ROS1 (degen_bag_ros1.bag) and column_shift:=512
# (pre-destaggered cloud + 180deg yaw mount). Foreground; Ctrl-C to stop.
set -uo pipefail
BAG=/home/dev/superodom_ws/degen_bag_ros1.bag
SHIFT=512
[ -f "$BAG" ] || { echo "MISSING $BAG (convert the ROS2 bag first)"; exit 1; }

source /opt/ros/noetic/setup.bash
source /home/dev/coinlio_ws/devel/setup.bash

cleanup_all(){ killall -q roslaunch rosmaster roscore rosbridge_websocket laserMapping rosbag 2>/dev/null; }
cleanup_node(){ killall -q laserMapping rosbag 2>/dev/null; }   # per-cycle: keep roscore+rosbridge
trap 'echo; echo "[coinlio-demo] stopping"; cleanup_all; exit 0' INT TERM

cleanup_all; sleep 2

# --- persistent master + rosbridge (started ONCE) ---
roscore > /tmp/coinlio_demo_roscore.log 2>&1 &
sleep 4
rosparam set /use_sim_time true
roslaunch rosbridge_server rosbridge_websocket.launch > /tmp/coinlio_demo_rosbridge.log 2>&1 &
sleep 4
echo "[coinlio-demo] rosbridge up on :9090  ->  connect Foxglove (Rosbridge) to ws://<host-ip>:9090"
HOSTIP=$(hostname -I | awk '{print $1}'); echo "[coinlio-demo] this host: ws://$HOSTIP:9090   (fixed frame: camera_init)"

# --- loop: fresh node + one bag play per cycle (the launch's own player; uses the live master) ---
while true; do
  echo "[coinlio-demo] launching coin_lio + playing degen bag (column_shift=$SHIFT)..."
  roslaunch coin_lio mapping_ouster.launch bag_file:="$BAG" rviz:=false column_shift:=$SHIFT \
      > /tmp/coinlio_demo.log 2>&1 &
  LP=$!
  # the bag is ~176s + 2s player delay; wait it out, then drop the node for a clean restart
  sleep 185
  echo "[coinlio-demo] cycle done; relaunching node"
  kill -INT $LP 2>/dev/null
  cleanup_node
  sleep 3
done
