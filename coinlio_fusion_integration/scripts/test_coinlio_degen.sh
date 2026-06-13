#!/bin/bash
# Standalone COIN-LIO (the original ETH Zurich ROS1 stack, ~/coinlio_ws) on the DEGENERATE
# bag — the third leg of the EllipseLIO vs COIN-LIO vs IRIS-LIO GATE-0 comparison.
#
# COIN-LIO is ROS1 Noetic (native on this host). The degen bag is ROS2, so it must first be
# converted to a ROS1 .bag (rosbags-convert; done separately). This harness runs COIN-LIO's
# stock replay launch N times and records /Odometry (nav_msgs/Odometry) so the same host-side
# analyze_traj.py can score it with the identical frame-invariant metrics as the ROS2 legs.
#
#   ./test_coinlio_degen.sh           # 3 runs (coinlio_b1..b3)
#   ./test_coinlio_degen.sh 1         # single run
#
# Records to /home/dev/coinlio_fusion_ws/results_degen/coinlio_bN.bag (ROS1 bag files), the
# SAME results_degen dir the ROS2 legs use, so one analyze_traj.py call covers all three.
set -uo pipefail
N="${1:-3}"
BAG=/home/dev/superodom_ws/degen_bag_ros1.bag
# NOTE: results_degen/ is root-owned (created by the ROS2 container running as root), so a
# native rosbag record running as `dev` cannot write there. Use a dev-owned dir; the
# analyzer takes explicit paths so the three legs need not share a directory.
OUTBASE=/home/dev/coinlio_fusion_ws/results_coinlio
LAUNCH="coin_lio mapping_ouster.launch"   # loads os1-64-1024x20.json + params.yaml; rviz off

if [ ! -f "$BAG" ]; then echo "[coinlio] MISSING $BAG — convert the ROS2 bag first"; exit 1; fi
source /opt/ros/noetic/setup.bash
source /home/dev/coinlio_ws/devel/setup.bash
mkdir -p "$OUTBASE"

cleanup() { killall -q roslaunch rosmaster roscore rosbag record 2>/dev/null; }

for i in $(seq 1 "$N"); do
  L=coinlio_b$i
  echo "############## COIN-LIO run $L ##############"
  cleanup; sleep 3                       # clean ROS1 master/node slate per run
  rm -f "$OUTBASE/$L.bag" "$OUTBASE/$L.bag.active"

  # Stock replay launch: brings up roscore + coin_lio mapping node + rosbag play (--clock -d 2).
  # column_shift:=512 is REQUIRED — these bags are pre-destaggered + IMU is 180deg-yaw-mounted, so
  # the image azimuth origin is offset exactly half of 1024 cols (confirmed by circular-mean calib =
  # 512.0). The launch default (4) overrides params.yaml and reproduces the divergence. Extrinsic +
  # photo_scale come from params.yaml (rosparam-loaded). destagger stays false (launch default).
  roslaunch $LAUNCH bag_file:="$BAG" rviz:=false column_shift:=512 > /tmp/coinlio_$L.log 2>&1 &
  LP=$!
  sleep 9                                # master + node + 2s player delay warmup

  # Record COIN-LIO odometry. -O writes a single .bag; header stamps are the LiDAR-frame time.
  rosbag record -O "$OUTBASE/$L.bag" /Odometry > /tmp/coinlio_rec_$L.log 2>&1 &
  RP=$!
  echo "[coinlio] $L: launched (pid $LP), recording /Odometry; playing ~176s bag..."

  sleep 185                              # bag is ~176s + 2s delay; cover it
  kill -INT $RP 2>/dev/null; sleep 3     # flush the record bag cleanly
  cleanup; sleep 3

  if [ -f "$OUTBASE/$L.bag" ]; then
    echo "[coinlio] recorded -> $OUTBASE/$L.bag"
  else
    echo "[coinlio] WARNING: no bag produced for $L — check /tmp/coinlio_$L.log"
    grep -iE "error|fail|no.*topic|abort" /tmp/coinlio_$L.log 2>/dev/null | head
  fi
done

echo "############## COIN-LIO DONE — ANALYSIS ##############"
for i in $(seq 1 "$N"); do
  python3 /home/dev/coinlio_fusion_ws/analyze_traj.py "$OUTBASE/coinlio_b$i.bag" 2>/dev/null
done
