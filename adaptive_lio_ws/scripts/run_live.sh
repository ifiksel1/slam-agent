#!/usr/bin/env bash
# LIVE Adaptive-LIO full stack on the Dragunfly rig: Ouster ROS1 driver -> Adaptive-LIO -> MAVROS
# vision bridge. Requires an image built with the Ouster driver:  ./build_image.sh --live
#
# This does NOT arm the vehicle — it only feeds /mavros/vision_pose/pose. Before ANY Loiter/Guided
# flight, verify the TF chain (map -> alio_odom -> alio_body -> base_link) with check_tf_tree and
# confirm the yaw sign against a known physical turn (see master.launch header notes).
#
# Env overrides: SENSOR_IP (default 192.168.2.60), HOST_IP (192.168.2.50), LIDAR_MODE (1024x20).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-adaptive-lio:noetic}"
CTR="${CTR:-adaptive_lio}"
SENSOR_IP="${SENSOR_IP:-192.168.2.60}"
HOST_IP="${HOST_IP:-192.168.2.50}"
LIDAR_MODE="${LIDAR_MODE:-1024x20}"

F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" \
  || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost' (Adaptive-LIO is CPU-bound)"

if ! docker ps --format '{{.Names}}' | grep -qx "$CTR"; then
  docker rm -f "$CTR" >/dev/null 2>&1 || true
  docker run -d --name "$CTR" --network host --privileged \
    -v "$HERE":/root/catkin_ws/src/adaptive_lio_ws:ro \
    -v "$HERE/config/mapping_ouster.yaml":/root/catkin_ws/src/adaptive_lio/config/mapping_m.yaml:ro \
    -v /dev:/dev \
    "$IMAGE" sleep infinity >/dev/null
fi

docker exec -e SENSOR_IP="$SENSOR_IP" -e HOST_IP="$HOST_IP" -e LIDAR_MODE="$LIDAR_MODE" -it "$CTR" bash -lc '
  source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash
  export ROS_PACKAGE_PATH=/root/catkin_ws/src/adaptive_lio_ws:$ROS_PACKAGE_PATH
  if ! rospack find ouster_ros >/dev/null 2>&1; then
    echo "ERROR: Ouster ROS1 driver not in image. Rebuild:  ./build_image.sh --live"; exit 1
  fi
  echo "=== Ouster driver ($SENSOR_IP -> $HOST_IP, $LIDAR_MODE) ==="
  roslaunch ouster_ros sensor.launch \
     sensor_hostname:=$SENSOR_IP udp_dest:=$HOST_IP \
     lidar_mode:=$LIDAR_MODE timestamp_mode:=TIME_FROM_INTERNAL_OSC viz:=false \
     > /tmp/ouster.log 2>&1 &
  for i in $(seq 1 30); do rostopic list 2>/dev/null | grep -q /ouster/points && break; sleep 1; done
  rostopic list | grep -q /ouster/points || { echo "no /ouster/points — check /tmp/ouster.log"; exit 1; }
  echo "=== full stack (MAVROS + Adaptive-LIO + vision bridge) ==="
  roslaunch adaptive_lio_ws master.launch
'
