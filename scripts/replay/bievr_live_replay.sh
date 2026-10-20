#!/bin/bash
# Real-time replay of the BIEVR-LIO FC chain on a flight bag, inside slam-system:bievr:
#   rosbag play (raw /lidar_points + /lidar_imu only) -> bievr_lio_ros/process_topics
#   -> bievr_health.py (/bievr_health)  +  fastlio_mavros_bridge.py with the bievr-mode params
# Records /bievr_health, /bievr_lio/odom and the bridge output to <prefix>_chain.bag.
#
# The bridge publishes to /replay/vision_pose/pose and /replay/vision_speed/speed_vector (remapped)
# so the recording can never be confused with the flown /mavros/* topics. Run in a container with
# --network none and no devices (see bievr_run_one.sh); there is no mavros and no FC here.
# NEVER run on the NUC while the live stack is up.
#
# Usage: bievr_live_replay.sh <bag>[,<part2.bag>] <prefix> [fc_mount_yaw_deg=0]
#   fc_mount_yaw_deg defaults to 0 because flights before 2026-09-29 flew without it, so the
#   bridge output is then directly comparable with the flown /mavros/vision_pose/pose.
set -u
IN="$1"; PREFIX="$2"; MOUNT_YAW="${3:-0}"
PORT=11312
export ROS_MASTER_URI="http://localhost:${PORT}"
export ROS_HOSTNAME=localhost
set +u
source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash
set -u

for f in ${IN//,/ }; do [ -f "$f" ] || { echo "[live_replay] input bag not found: $f"; exit 1; }; done
BAGS="${IN//,/ }"

cleanup() {
  rosnode kill /chain_rec >/dev/null 2>&1 || true
  sleep 2
  for p in ${PIDS:-}; do kill -INT "$p" >/dev/null 2>&1 || true; done
  sleep 2
  [ -n "${ROSCORE_PID:-}" ] && kill -INT "$ROSCORE_PID" >/dev/null 2>&1 || true
  sleep 1
}
trap cleanup EXIT

roscore -p $PORT >"${PREFIX}_roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 4
rosparam set /use_sim_time true
PIDS=""

rosrun bievr_lio_ros process_topics --sensor_config_file /cfg/bievr_jt128.yaml \
  --params_file /cfg/bievr_params.yaml >"${PREFIX}_bievr.log" 2>&1 &
PIDS="$PIDS $!"
rosrun fast_lio bievr_health.py >"${PREFIX}_health.log" 2>&1 &
PIDS="$PIDS $!"
# Same params as the slam_backend:=bievr bridge in config/fastlio_to_fc.launch.
rosrun fast_lio fastlio_mavros_bridge.py __name:=fastlio_mavros_bridge \
  _odom_topic:=/bievr_lio/odom _twist_frame:=body _remap_axes:=true \
  _position_matrix:="[0,-1,0, 1,0,0, 0,0,1]" \
  _orientation_matrix:="[0,-1,0, 1,0,0, 0,0,1]" \
  _orientation_body_matrix:="[0,0,1, 1,0,0, 0,1,0]" \
  _fc_mount_yaw_deg:=${MOUNT_YAW} _restamp_system_time:=false \
  /mavros/vision_pose/pose:=/replay/vision_pose/pose \
  /mavros/vision_speed/speed_vector:=/replay/vision_speed/speed_vector >"${PREFIX}_bridge.log" 2>&1 &
PIDS="$PIDS $!"
sleep 3

rosbag record -O "${PREFIX}_chain.bag" --buffsize=1024 __name:=chain_rec \
  /bievr_health /bievr_lio/odom /replay/vision_pose/pose /replay/vision_speed/speed_vector \
  >"${PREFIX}_rec.log" 2>&1 &
sleep 2

echo "[live_replay] playing $BAGS"
rosbag play --clock --quiet $BAGS --topics /lidar_points /lidar_imu
echo "[live_replay] play finished; draining"
sleep 3
cleanup
trap - EXIT
[ -f "${PREFIX}_chain.bag.active" ] && rosbag reindex "${PREFIX}_chain.bag.active" >/dev/null 2>&1 \
  && mv "${PREFIX}_chain.bag.active" "${PREFIX}_chain.bag"
echo "[live_replay] DONE -> ${PREFIX}_chain.bag"
