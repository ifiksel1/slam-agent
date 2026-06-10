#!/bin/bash
# #1 Repeatability + #3 degeneracy-stats capture for SuperOdom, all from degen_bag
# (no physical motion needed). Runs the 176s bag N times through a FRESH SuperOdom
# each time and records the return-to-origin error (net displacement = accumulated
# drift, since the corridor walk returns near its start). Each run also dumps a
# full per-scan CSV (uncertainty_*, plane matches, ICP residual, latency) under
# ~/superodom_ws/results/repeat/, which feeds the degeneracy / EKF-gating analysis.
#
# Why net displacement is the accuracy metric: with no mocap, the only ground truth
# is "the operator ended where they started," so end-distance-from-origin is total
# drift. Run-to-run spread of that number = SuperOdom's (nondeterministic, threaded)
# repeatability.
#
# Bag is played at --rate 1.0 (no --loop; loop rewinds timestamps -> feature sync
# fails). Uses CycloneDDS to match the driver/image default.
#
# Deploy: the in-container sampler must be at the workspace root the container mounts:
#   cp superodom_integration/scripts/repeat_sampler.py ~/superodom_ws/repeat_sampler.py
# (the repo dir is NOT mounted into the container; only ~/superodom_ws is -> /root/ros2_ws).
#
# Usage: repeatability.sh [N] [CONTAINER]   (default N=5, container=superodom)
set -euo pipefail
N="${1:-5}"
CTR="${2:-superodom}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW="rmw_cyclonedds_cpp"
BAG="/root/ros2_ws/degen_bag"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
SO_CFG=/root/ros2_ws/src/SuperOdom/super_odometry/config
LAUNCH="ros2 launch super_odometry os1_128.launch.py config_file:=$SO_CFG/os1_64.yaml calibration_file:=$SO_CFG/ouster/os1_64_calibration.yaml"
OUTDIR=/root/ros2_ws/results/repeat            # in-container path (maps to ~/superodom_ws/results/repeat)
HOSTOUT="$HOME/superodom_ws/results/repeat"

docker ps --format '{{.Names}}' | grep -qx "$CTR" || { echo "container '$CTR' not running"; exit 1; }
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"
mkdir -p "$HOSTOUT"
echo "=== SuperOdom repeatability: $N runs of the 176s bag ==="

declare -a RESULTS
for i in $(seq 1 "$N"); do
  echo
  echo "--- run $i/$N ---"
  bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null
  # fresh SuperOdom (empty map)
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && $LAUNCH > /tmp/repeat_so_$i.txt 2>&1"
  sleep 7
  up=$(docker exec "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 node list 2>/dev/null" \
        | grep -cE 'feature_extraction|laser_mapping|imu_preintegration' || true)
  echo "  nodes up: $up (want 3)"
  # sampler subscribed BEFORE the bag plays (so it catches every scan); self-terminates on idle
  csv="$OUTDIR/run_$i.csv"
  # CRITICAL: clear any prior csv + .done FIRST. The wait below keys on ${csv}.done, so a
  # stale marker (e.g. from an earlier batch that reused these run indices) makes the wait
  # break instantly -> the run races through without playing the bag and the next cleanup
  # kills it mid-launch. Removing it here makes each run robust to leftovers.
  docker exec "$CTR" bash -c "rm -f ${csv} ${csv}.done"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/repeat_sampler.py $csv > /tmp/repeat_log_$i.txt 2>&1"
  sleep 2
  echo "  playing bag (rate 1.0, ~176s)..."
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 bag play $BAG --rate 1.0 > /tmp/repeat_bag_$i.txt 2>&1"
  # Wait ONLY on the sampler's .done marker — it writes that ~8s after the last scan
  # (i.e. after the bag ends), then finalizes. This is zombie-proof: we do NOT poll the
  # bag-play process, because on a container without --init the finished `ros2 bag play`
  # lingers as a <defunct> zombie that a naive `pgrep -f 'bag play'` would match forever.
  # Cap ~360s (bag 176s + sampler HARD_S 240s safety margin).
  echo "  waiting for sampler .done marker..."
  for w in $(seq 1 120); do
    docker exec "$CTR" bash -c "[ -f ${csv}.done ]" && break
    sleep 3
  done
  res=$(docker exec "$CTR" bash -c "grep -h '^RESULT' /tmp/repeat_log_$i.txt 2>/dev/null | tail -1" || true)
  echo "  $res"
  RESULTS[$i]="$res"
done

# cleanup the last instance so nothing is left running
bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null || true

echo
echo "========================= REPEATABILITY SUMMARY ========================="
printf "%-5s %8s %8s %8s %10s %10s\n" run path net peak end_yaw hfalse_s
nets=""
for i in $(seq 1 "$N"); do
  line="${RESULTS[$i]:-}"
  path=$(grep -oE 'path=[0-9.]+' <<<"$line" | cut -d= -f2)
  net=$(grep -oE 'net=[0-9.]+' <<<"$line" | cut -d= -f2)
  peak=$(grep -oE 'peak=[0-9.]+' <<<"$line" | cut -d= -f2)
  yaw=$(grep -oE 'end_yaw=[+-]?[0-9.]+' <<<"$line" | cut -d= -f2)
  hf=$(grep -oE 'health_false_s=[0-9.]+' <<<"$line" | cut -d= -f2)
  printf "%-5s %8s %8s %8s %10s %10s\n" "$i" "${path:-NA}" "${net:-NA}" "${peak:-NA}" "${yaw:-NA}" "${hf:-NA}"
  [ -n "${net:-}" ] && nets="$nets $net"
done
# net displacement spread (the headline reliability number)
if [ -n "$nets" ]; then
  echo "$nets" | tr ' ' '\n' | grep -E '[0-9]' | awk '
    {a[NR]=$1; s+=$1; if(NR==1||$1<mn)mn=$1; if(NR==1||$1>mx)mx=$1}
    END{m=s/NR; for(i=1;i<=NR;i++)v+=(a[i]-m)^2; sd=(NR>1)?sqrt(v/(NR-1)):0;
        printf "\nnet-displacement (return-to-origin drift):  mean=%.2fm  sd=%.2fm  min=%.2fm  max=%.2fm  spread=%.2fm  n=%d\n", m, sd, mn, mx, mx-mn, NR}'
fi
echo
echo "Per-scan CSVs (for #3 degeneracy analysis): $HOSTOUT/run_*.csv"
echo "========================================================================"
