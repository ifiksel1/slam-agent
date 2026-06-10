#!/bin/bash
# ISOLATION EXPERIMENT: does SuperOdom's transient-divergence rate correlate with
# accumulated CONTAINER STATE (zombie/PID buildup on a no-init container), or is it
# inherent algorithm nondeterminism?
#
# Method: same degen_bag replay as repeatability.sh, but BEFORE each run it snapshots
# container state (zombie count, RAM, CPU temp) and logs it next to the run outcome
# (peak excursion, path). Intended to be run on a FRESH `--init` container (zombies get
# reaped -> stay ~0) and compared against the dirty-container backfill (155 zombies, 3/6
# anomalies). If the fresh-container anomaly rate collapses -> container state was the
# cause. If it stays ~the same -> the divergence is inherent to SuperOdom.
#
# Outcome tiering (from peak excursion vs the real ~7.2m corridor):
#   SEVERE  peak > 9 m   (pose blew past the corridor — crash-grade)
#   jitter  path > 100 m AND peak <= 9 m  (wiggly but bounded)
#   clean   otherwise
#
# Output CSV: results/repeat/state_corr_<label>.csv with columns
#   run,zombies_before,ram_used_mb,temp_c,dur,net,peak,path,tier
#
# Usage: state_correlation.sh <N> <CONTAINER> <LABEL>
set -euo pipefail
N="${1:-15}"
CTR="${2:-superodom}"
LABEL="${3:-fresh}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW="rmw_cyclonedds_cpp"
BAG="/root/ros2_ws/degen_bag"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
SO_CFG=/root/ros2_ws/src/SuperOdom/super_odometry/config
LAUNCH="ros2 launch super_odometry os1_128.launch.py config_file:=$SO_CFG/os1_64.yaml calibration_file:=$SO_CFG/ouster/os1_64_calibration.yaml"
OUTDIR=/root/ros2_ws/results/repeat
OUTCSV="$HOME/superodom_ws/results/repeat/state_corr_${LABEL}.csv"

docker ps --format '{{.Names}}' | grep -qx "$CTR" || { echo "container '$CTR' not running"; exit 1; }
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"
echo "run,zombies_before,ram_used_mb,temp_c,dur,net,peak,path,tier" > "$OUTCSV"
echo "=== state-correlation: $N runs on '$CTR' (label=$LABEL) ==="

for i in $(seq 1 "$N"); do
  echo "--- run $i/$N ($LABEL) ---"
  bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null
  # STATE SNAPSHOT (before launching this run)
  zomb=$(docker exec "$CTR" bash -c 'ps -eo stat | awk "/Z/" | wc -l' 2>/dev/null || echo -1)
  ram=$(free -m | awk '/Mem/{print $3}')
  temp=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp 2>/dev/null | awk '{printf "%.1f",$1/1000}')
  echo "  state: zombies=$zomb ram_used=${ram}MB temp=${temp}C"

  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && $LAUNCH > /tmp/sc_so_$i.txt 2>&1"
  sleep 7
  csv="$OUTDIR/sc_${LABEL}_$i.csv"
  docker exec "$CTR" bash -c "rm -f ${csv} ${csv}.done"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/repeat_sampler.py $csv > /tmp/sc_log_$i.txt 2>&1"
  sleep 2
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 bag play $BAG --rate 1.0 > /tmp/sc_bag_$i.txt 2>&1"
  for w in $(seq 1 120); do
    docker exec "$CTR" bash -c "[ -f ${csv}.done ]" && break
    sleep 3
  done
  line=$(docker exec "$CTR" bash -c "grep -h '^RESULT' /tmp/sc_log_$i.txt 2>/dev/null | tail -1" || true)
  dur=$(grep -oE 'dur=[0-9.]+' <<<"$line" | cut -d= -f2)
  net=$(grep -oE 'net=[0-9.]+' <<<"$line" | cut -d= -f2)
  path=$(grep -oE 'path=[0-9.]+' <<<"$line" | cut -d= -f2)
  peak=$(grep -oE 'peak=[0-9.]+' <<<"$line" | cut -d= -f2)
  tier=$(awk -v p="${peak:-0}" -v pa="${path:-0}" 'BEGIN{if(p>9)print "SEVERE"; else if(pa>100)print "jitter"; else print "clean"}')
  echo "  -> dur=$dur peak=$peak path=$path tier=$tier"
  echo "$i,$zomb,$ram,$temp,$dur,$net,$peak,$path,$tier" >> "$OUTCSV"
done
bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null || true

echo
echo "===== $LABEL SUMMARY ====="
awk -F, 'NR>1{n++; z+=$2; if($9=="SEVERE")s++; if($9=="jitter")j++}
  END{printf "runs=%d  mean_zombies_before=%.0f  SEVERE=%d (%.0f%%)  jitter=%d (%.0f%%)  anomaly=%d (%.0f%%)\n",
      n, z/n, s, 100*s/n, j, 100*j/n, s+j, 100*(s+j)/n}' "$OUTCSV"
echo "CSV: $OUTCSV"