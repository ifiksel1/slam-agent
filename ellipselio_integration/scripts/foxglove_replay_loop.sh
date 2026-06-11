#!/bin/bash
# Re-watchable EllipseLIO bag replay for Foxglove WITHOUT the looping-bag time-jump bug.
# Each cycle: relaunch EllipseLIO + odom->path republisher FRESH, then play the bag
# ONCE (monotonic sim-time), then tear down and repeat. ros2 bag play --loop is NOT
# usable here: a persistent node sees sim-time jump backwards at loop restart and
# floods "Imu time out of order", killing the cloud/odom output.
#
# The foxglove_bridge is started SEPARATELY (persistent, whitelisted) so Foxglove stays
# connected across cycles -- see run_bag.sh / the bringup that calls this.
# Runs INSIDE the ellipselio container; expects the workspace mounted at /root/ros2_ws.
set -uo pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
CFG=/root/ros2_ws/src/ellipselio/config
BAG=${1:-/root/field/apartment_20260611_034239}

while true; do
  pkill -9 -f 'ellipselio_standalone|component_container_mt|odom_to_path|ros2 bag play' 2>/dev/null
  sleep 2
  bash -lc "$SRC && ros2 launch ellipselio ellipselio_standalone.launch.py \
      config_path:=$CFG config_file:=os1_64_ouster.yaml \
      use_sim_time:=true rviz:=false > /tmp/ellipselio_fox.log 2>&1" &
  sleep 8
  bash -lc "$SRC && python3 /root/ros2_ws/odom_to_path.py > /tmp/odom_to_path.log 2>&1" &
  sleep 2
  echo "[loop] playing bag once (monotonic sim-time)..."
  bash -lc "$SRC && ros2 bag play $BAG --clock --rate 1.0 > /tmp/bagplay_fox.log 2>&1"
  echo "[loop] bag done; settling 4s then restarting"
  sleep 4
done
