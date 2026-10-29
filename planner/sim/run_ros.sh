#!/usr/bin/env bash
# Inside the L2 ROS container (see sitl_harness.sh): roscore, MAVROS to SITL, the robot's EKF origin
# publisher, the robot's own SLAM-to-FC bridge, and the render node standing in for FAST-LIO.
# Env: MAP_PCD (container path), START_X, START_Y, START_YAW_DEG, LIDAR_UP, LATENCY_S, FAULT, FAULT_START_S, FAULT_RATE.
set -uo pipefail
source /opt/ros/noetic/setup.bash
source /super_ws/devel/setup.bash
L=/run/logs; mkdir -p $L
export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311

roscore > $L/roscore.log 2>&1 &
for _ in $(seq 1 30); do rosparam list >/dev/null 2>&1 && break; sleep 1; done

# MAVROS as on the robot (config/fastlio_to_fc.launch), pointed at SITL's SERIAL0
rosparam load /slam/config/mavros_apm_config.yaml /mavros
rosrun mavros mavros_node __name:=mavros _fcu_url:=tcp://127.0.0.1:5760 _gcs_url:="" \
  _target_system_id:=1 _target_component_id:=1 > $L/mavros.log 2>&1 &

# EKF origin for GPS-denied ExternalNav, as on the robot (same placeholder coordinates as SITL's home)
rostopic pub -r 0.1 /mavros/global_position/set_gp_origin geographic_msgs/GeoPointStamped \
  '{position: {latitude: 42.56335, longitude: 12.64329, altitude: 163.0}}' > $L/origin.log 2>&1 &

# the robot's bridge, its QuadRover params
rosparam load /planner_sim/bridge_params.yaml /fastlio_mavros_bridge
python3 -u /slam/overrides/FAST-LIO/src/fastlio_mavros_bridge.py __name:=fastlio_mavros_bridge > $L/bridge.log 2>&1 &

# FAST-LIO's stand-in (RENDER_SCRIPT: faults/render_fault.py adds live faults for the failure tests)
python3 -u ${RENDER_SCRIPT:-/planner_sim/render_node.py} __name:=render_node _map_pcd:=$MAP_PCD \
  _start_x:=${START_X:-0} _start_y:=${START_Y:-0} _start_yaw_deg:=${START_YAW_DEG:-0} \
  _lidar_up:=${LIDAR_UP:-0} _latency_s:=${LATENCY_S:-0.07} _fault:=${FAULT:-none} _fault_start_s:=${FAULT_START_S:-1e9} _fault_rate:=${FAULT_RATE:-0.05} \
  > $L/render.log 2>&1 &

echo "ROS side up: logs in $L"
wait
