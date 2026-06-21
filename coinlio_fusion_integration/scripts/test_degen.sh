#!/bin/bash
# Headless A/B/sweep of IRIS-LIO (EllipseLIO geometry + COIN-LIO LiDAR-intensity
# photometric) on the DEGENERATE bag — the recording where SuperOdom's health flag
# dropped (geometry-degenerate scene). This is the v2 GATE-0 / obs_score-gain target.
#
# Records /ellipselio_odom (trajectory) + /analytics (obs_min degeneracy signal) so we
# can compare in-plane drift through the degenerate window across configs.
# Cleanup is PID-based (NOT pkill -f, which self-matches this script's command line).
#
#   ./test_degen.sh geo                    # photometric.enable=false (geometric baseline)
#   ./test_degen.sh photo                  # enable=true, scale=1e-9, deg_gain=0 (v1 constant)
#   ./test_degen.sh s1e-8  1.0e-8  0       # custom: label scale deg_gain
#   ./test_degen.sh g200   1.0e-9  200     # obs_score-modulated (v2): scale + deg_gain
set -uo pipefail
CTR=coinlio_fusion
LABEL="${1:-photo}"
SCALE="${2:-1.0e-9}"
DEGGAIN="${3:-0.0}"
EN=true
[[ "$LABEL" == geo* ]] && EN=false   # any geo* label => photometric pipeline fully OFF
BAG=/root/bags_extra/degen_bag
CFGDIR=/root/ros2_ws/src/ellipselio/config

docker exec "$CTR" bash -lc '
  set -o pipefail
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
  LABEL="'"$LABEL"'"; EN="'"$EN"'"; SCALE="'"$SCALE"'"; DEGGAIN="'"$DEGGAIN"'"
  BAG="'"$BAG"'"; CFGDIR="'"$CFGDIR"'"
  OUT=/root/ros2_ws/results_degen/$LABEL
  rm -rf "$OUT"; mkdir -p /root/ros2_ws/results_degen

  # Per-run config: start from the canonical yaml, sed in enable/scale/deg_gain.
  USECFG=_degen_$LABEL.yaml
  sed -e "s/enable: true/enable: $EN/" \
      -e "s/photo_scale: .*/photo_scale: $SCALE/" \
      -e "s/photo_deg_gain: .*/photo_deg_gain: $DEGGAIN/" \
      "$CFGDIR/os1_64_ouster.yaml" > "$CFGDIR/$USECFG"
  echo "[degen] $LABEL: enable=$EN scale=$SCALE deg_gain=$DEGGAIN"

  ros2 launch ellipselio ellipselio_standalone.launch.py \
      config_path:=$CFGDIR config_file:=$USECFG \
      use_sim_time:=true rviz:=false > /tmp/ell_$LABEL.log 2>&1 &
  LAUNCH_PID=$!
  sleep 10
  CPID=$(grep -oE "process started with pid \[[0-9]+\]" /tmp/ell_$LABEL.log | grep -oE "[0-9]+" | head -1)
  echo "[degen] launch_pid=$LAUNCH_PID container_pid=$CPID"

  ros2 bag record -o "$OUT" /ellipselio_odom /analytics > /tmp/rec_$LABEL.log 2>&1 &
  REC_PID=$!
  sleep 2
  echo "[degen] playing degen bag ($LABEL)..."
  ros2 bag play "$BAG" --clock --rate 1.0 > /tmp/play_$LABEL.log 2>&1
  echo "[degen] bag done; settling"; sleep 3

  kill -INT $REC_PID 2>/dev/null; sleep 2
  [ -n "$CPID" ] && kill -9 $CPID 2>/dev/null
  kill -9 $LAUNCH_PID 2>/dev/null
  sleep 1
  echo "[degen] === photometric line ==="; grep -iE "\[photometric\]" /tmp/ell_$LABEL.log | head
  echo "[degen] === errors ==="; grep -iE "nan|terminate|what\(|abort|exception|runaway" /tmp/ell_$LABEL.log | head
  echo "[degen] recorded -> $OUT"
'
