#!/bin/bash
# Re-watchable IRIS-LIO (photometric-fusion EllipseLIO) bag replay for Foxglove.
# Supersedes foxglove_bag_demo.sh: parameterized bag + a teardown that ACTUALLY works.
#
# BUG this fixes: foxglove_bag_demo.sh tore the per-cycle node down with
#   for pid in $(ps -eo pid,comm | awk '/component_container/{print $1}'); do kill -INT $pid; done
# but `ps -o comm` truncates to 15 chars ("component_conta"), so /component_container/ never
# matched and the old node was NEVER killed -> every cycle leaked another component_container_mt,
# giving 2+ estimators publishing /ellipselio_odom on the same domain (fighting publishers,
# garbled pose in Foxglove). Here we capture each cycle's launch PID and reap by PID
# (kill -INT $LAUNCH_PID + pkill -P $LAUNCH_PID) — no `pkill -f` pattern, so nothing can
# self-match this script's own command line.
#
# Usage: run_iris_lio_foxglove.sh <bag_dir_name>   (a folder under ~/superodom_ws/field/)
#   Connect Foxglove -> Foxglove WebSocket -> ws://<jetson-ip>:8765, fixed frame = map_lidar.
#   Stop: docker exec coinlio_fusion bash -c 'kill $(cat /tmp/iris_fox_loop.pid)'  (or stop the container)
set -uo pipefail
BAGNAME="${1:?usage: run_iris_lio_foxglove.sh <bag_dir_name>}"
CTR=coinlio_fusion
IMG=coinlio-fusion:humble
WS="$HOME/coinlio_fusion_ws"
FIELD_HOST="$HOME/superodom_ws/field"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }

# container up with the field bag mounted? (re)create if missing the mount.
if ! docker inspect "$CTR" --format '{{range .Mounts}}{{.Source}} {{end}}' 2>/dev/null | grep -q "$FIELD_HOST"; then
  echo "=== (re)creating $CTR with field-bag mount ==="
  docker rm -f "$CTR" 2>/dev/null || true
  docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v "$WS:/root/ros2_ws" -v "$FIELD_HOST:/root/field:ro" \
    -v /home/dev/slam-gpu/bags:/root/bags:ro "$IMG" sleep infinity
else
  docker start "$CTR" >/dev/null 2>&1 || true
fi

cp "$HERE/../../ellipselio_integration/config/foxglove_params.yaml" "$WS/foxglove_params.yaml"

# hard reset: kill anything from a previous session (PID-safe inside the container)
docker exec "$CTR" bash -c '
  for p in $(pgrep -f "component_container_mt|ellipselio_standalone|ros2 bag play|foxglove_bridge|static_transform_publisher|iris_fox_loop" 2>/dev/null); do kill -9 $p 2>/dev/null; done
  sleep 1; true'

echo "=== launching IRIS-LIO Foxglove loop on $BAGNAME ==="
docker exec -d "$CTR" bash -lc '
  echo $$ > /tmp/iris_fox_loop.pid
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
  CFGDIR=/root/ros2_ws/src/ellipselio/config
  BAG=/root/field/'"$BAGNAME"'

  # viz-only LiDAR-forward frame (180deg pitch); TF only, no effect on SLAM
  ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --yaw 0 --pitch 3.14159265 --roll 0 \
      --frame-id odom_ellipselio --child-frame-id map_lidar > /tmp/static_tf.log 2>&1 &

  while true; do
    rm -f /tmp/launch_params_* 2>/dev/null   # ros2 launch leaks one per cycle; clear before each
    # fresh node each cycle => monotonic sim-time (a persistent node breaks on the
    # bag-restart /clock jump with "Imu time out of order").
    ros2 launch ellipselio ellipselio_standalone.launch.py \
        config_path:=$CFGDIR config_file:=os1_64_ouster.yaml \
        use_sim_time:=true rviz:=false > /tmp/ell_demo.log 2>&1 &
    NODE_PID=$!
    sleep 8
    # RESTART THE BRIDGE EACH CYCLE, AFTER the node publishers are live. foxglove_bridge
    # auto-detects best-effort QoS only from a live publisher at subscribe time AND does NOT
    # reliably re-subscribe when the node relaunches with new publisher GUIDs -> a PERSISTENT
    # bridge goes blank in Foxglove after the first cycle. Restarting it each cycle re-detects
    # QoS against the fresh publishers; the Foxglove client just auto-reconnects (~2s blackout).
    ros2 run foxglove_bridge foxglove_bridge --ros-args \
        --params-file /root/ros2_ws/foxglove_params.yaml \
        -p use_sim_time:=true -p max_qos_depth:=25 > /tmp/foxglove_bridge.log 2>&1 &
    BRIDGE_PID=$!
    sleep 2
    echo "[iris-fox] node+bridge up; playing '"$BAGNAME"' (photometric fusion)..."
    ros2 bag play $BAG --clock --rate 1.0 > /tmp/play_demo.log 2>&1 &
    BAGPID=$!
    wait $BAGPID
    echo "[iris-fox] bag done; reaping node + bridge (PID-based) + relaunching"
    # Reap BOTH by PID: not truncated comm, and not pkill -f which self-matches the loop cmdline.
    kill -INT $NODE_PID $BRIDGE_PID 2>/dev/null; sleep 3
    pkill -9 -P $NODE_PID 2>/dev/null; pkill -9 -P $BRIDGE_PID 2>/dev/null
    kill -9 $NODE_PID $BRIDGE_PID 2>/dev/null
    sleep 2
  done
' > /tmp/iris_fox.log 2>&1
echo "launched. Verifying single publisher after warmup..."
