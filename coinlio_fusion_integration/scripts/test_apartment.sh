#!/bin/bash
# Headless A/B test of photometric-fusion EllipseLIO on the apartment bag.
# Runs node once, plays bag with --clock, records /ellipselio_odom + /analytics.
# Cleanup is PID-based (NOT pkill -f, which self-matches this script's command line).
#
#   ./test_apartment.sh photo   # photometric.enable=true  (fusion)
#   ./test_apartment.sh geo     # photometric.enable=false (geometric baseline)
set -uo pipefail
CTR=coinlio_fusion
MODE="${1:-photo}"
BAG=/root/bags/apartment_20260611_034239
CFGDIR=/root/ros2_ws/src/ellipselio/config
EN=true; [[ "$MODE" == "geo" ]] && EN=false

docker exec "$CTR" bash -lc '
  set -o pipefail
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
  MODE="'"$MODE"'"; EN="'"$EN"'"; BAG="'"$BAG"'"; CFGDIR="'"$CFGDIR"'"
  OUT=/root/ros2_ws/results_photo/$MODE
  rm -rf "$OUT"; mkdir -p /root/ros2_ws/results_photo

  # config with the right enable flag (sed a copy; keep everything else)
  USECFG=os1_64_ouster.yaml
  if [ "$MODE" = geo ]; then
    USECFG=_geo.yaml
    sed "s/enable: true/enable: false/" "$CFGDIR/os1_64_ouster.yaml" > "$CFGDIR/$USECFG"
  fi

  ros2 launch ellipselio ellipselio_standalone.launch.py \
      config_path:=$CFGDIR config_file:=$USECFG \
      use_sim_time:=true rviz:=false > /tmp/ell_$MODE.log 2>&1 &
  LAUNCH_PID=$!
  sleep 10
  CPID=$(grep -oE "process started with pid \[[0-9]+\]" /tmp/ell_$MODE.log | grep -oE "[0-9]+" | head -1)
  echo "[test] launch_pid=$LAUNCH_PID container_pid=$CPID"

  ros2 bag record -o "$OUT" /ellipselio_odom /analytics > /tmp/rec_$MODE.log 2>&1 &
  REC_PID=$!
  sleep 2
  echo "[test] playing apartment bag ($MODE, enable=$EN)..."
  ros2 bag play "$BAG" --clock --rate 1.0 > /tmp/play_$MODE.log 2>&1
  echo "[test] bag done; settling"; sleep 3

  kill -INT $REC_PID 2>/dev/null; sleep 2
  [ -n "$CPID" ] && kill -9 $CPID 2>/dev/null
  kill -9 $LAUNCH_PID 2>/dev/null
  sleep 1
  echo "[test] === photometric line ==="; grep -iE "\[photometric\]" /tmp/ell_$MODE.log | head
  echo "[test] === errors ==="; grep -iE "nan|terminate|what\(|abort|exception|runaway" /tmp/ell_$MODE.log | head
  echo "[test] recorded -> $OUT"
'
