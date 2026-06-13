#!/bin/bash
# Foxglove viewer for the DEGENERATE bag (IRIS-LIO). Points at /root/bags_extra/degen_bag
# and lets you pick the mode:
#
#   ./foxglove_degen_demo.sh photo   # photometric pipeline ON  (v1 scale 1e-9)  [default]
#   ./foxglove_degen_demo.sh geo     # geometry-only baseline (deterministically stable)
#   ./foxglove_degen_demo.sh s1e-8 1.0e-8 0   # custom: label scale deg_gain
#
# Use this to WATCH whether the photometric pipeline stays stable under the boosted CPU
# clock (the v2 GATE-0 compute-starvation test). Geometry-only is the known-stable
# reference; flip to photo and watch /ellipselio_odom for divergence + /analytics obs_score.
#
# Connect Foxglove ("Open connection" -> Foxglove WebSocket) to ws://<host-ip>:8765:
#   - Image panels: /photometric/intensity_image, /photometric/feature_image
#   - 3D panel: /cloud_map + /ellipselio_odom  (fixed frame: map_lidar for LiDAR-forward)
#   - Raw-Message panel: /analytics  (watch obs_score / obs_min floor at 0.1 = degenerate)
#
# LOOP DESIGN (why it's structured this way):
#   * bridge + static-TF start ONCE and persist (Foxglove stays connected across cycles).
#   * Per cycle: fresh node + one bag play, then the node is HARD-KILLED BY ITS REAL PID
#     (parsed from the launch log) and we WAIT until `ros2 node list` shows no /ellipselio
#     before relaunching. This is mandatory: `ps -o comm` truncates to 15 chars
#     ("component_conta"), so the old `awk /component_container/` kill silently matched
#     NOTHING -> nodes leaked every cycle -> multiple /ellipselio fought over the replayed
#     /ouster/imu (which jumps 174s backward each loop) -> "Imu time out of order" + garbage.
#     One monotonic node lifetime per cycle is the whole point.
#
# Foreground (Ctrl-C to stop), or background + stop later.
set -uo pipefail
CTR=coinlio_fusion
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LABEL="${1:-photo}"
SCALE="${2:-1.0e-9}"
DEGGAIN="${3:-0.0}"
EN=true
[[ "$LABEL" == geo* ]] && EN=false   # any geo* label => photometric pipeline fully OFF

docker restart "$CTR" >/dev/null 2>&1; sleep 6   # clean DDS slate (no stale nodes)
cp "$HERE/../../ellipselio_integration/config/foxglove_params.yaml" \
   "$HOME/coinlio_fusion_ws/foxglove_params.yaml"

docker exec "$CTR" bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
  LABEL="'"$LABEL"'"; EN="'"$EN"'"; SCALE="'"$SCALE"'"; DEGGAIN="'"$DEGGAIN"'"
  CFGDIR=/root/ros2_ws/src/ellipselio/config
  BAG=/root/bags_extra/degen_bag

  # Per-run config: start from the canonical yaml, sed in enable/scale/deg_gain.
  USECFG=_fox_degen_$LABEL.yaml
  sed -e "s/enable: true/enable: $EN/" \
      -e "s/photo_scale: .*/photo_scale: $SCALE/" \
      -e "s/photo_deg_gain: .*/photo_deg_gain: $DEGGAIN/" \
      "$CFGDIR/os1_64_ouster.yaml" > "$CFGDIR/$USECFG"
  echo "[degen-demo] $LABEL: enable=$EN scale=$SCALE deg_gain=$DEGGAIN"

  # --- start-once infrastructure: static viz frame + foxglove bridge ---------------------
  # Viz-only frame: map is flipped 180 deg in PITCH (about Y) vs LiDAR-forward. Static
  # odom_ellipselio -> map_lidar so fixed frame = map_lidar renders LiDAR-forward (TF only).
  ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --yaw 0 --pitch 3.14159265 --roll 0 \
      --frame-id odom_ellipselio --child-frame-id map_lidar > /tmp/static_tf.log 2>&1 &

  # QoS: EllipseLIO publishes viz topics BEST_EFFORT. foxglove_bridge auto-detects QoS from
  # a LIVE publisher on first subscribe -- start it BEFORE the node and it defaults RELIABLE
  # and drops every best-effort topic. So start the bridge a few seconds into the FIRST play.
  bridge_started=0

  while true; do
    ros2 launch ellipselio ellipselio_standalone.launch.py \
        config_path:=$CFGDIR config_file:=$USECFG \
        use_sim_time:=true rviz:=false > /tmp/ell_degen_demo.log 2>&1 &
    LAUNCH_PID=$!
    sleep 8
    # Real container PID (the LOG keeps the full name; ps -o comm does not).
    CPID=$(grep -oE "process started with pid \[[0-9]+\]" /tmp/ell_degen_demo.log | grep -oE "[0-9]+" | head -1)
    echo "[degen-demo] launch_pid=$LAUNCH_PID container_pid=$CPID"

    echo "[degen-demo] playing degen bag ($LABEL)..."
    ros2 bag play "$BAG" --clock --rate 1.0 > /tmp/play_degen_demo.log 2>&1 &
    BAGPID=$!
    if [ $bridge_started -eq 0 ]; then
      sleep 8   # let the best-effort publishers come up under /clock
      ros2 run foxglove_bridge foxglove_bridge --ros-args \
          --params-file /root/ros2_ws/foxglove_params.yaml \
          -p use_sim_time:=true -p max_qos_depth:=25 \
          > /tmp/foxglove_bridge.log 2>&1 &
      bridge_started=1
      echo "[degen-demo] foxglove_bridge up on :8765 (QoS auto-detected from live publishers)"
    fi
    wait $BAGPID
    echo "[degen-demo] bag done; killing node (pid $CPID) for clean next cycle"

    # Hard-kill THIS node by real PID, then confirm it is gone from DDS before relaunch.
    [ -n "$CPID" ] && kill -9 $CPID 2>/dev/null
    kill -9 $LAUNCH_PID 2>/dev/null
    for i in $(seq 1 20); do
      n=$(timeout 4 ros2 node list 2>/dev/null | grep -c "/ellipselio")
      [ "${n:-1}" -eq 0 ] && break
      sleep 1
    done
    echo "[degen-demo] node down; relaunching"
  done
'
