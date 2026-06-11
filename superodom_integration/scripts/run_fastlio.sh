#!/bin/bash
# OFFLINE FAST-LIO replay-eval: run a field bag through FAST_LIO_GPU (the slam_gpu_system
# container), build + save the map PCD, record the trajectory, print divergence/loop metrics.
# Parallel to process_field_bag.sh (which does the same for SuperOdom) so you can benchmark the
# SAME clean bag through both frameworks and compare apples-to-apples. Boost the clock first.
#
# Usage: run_fastlio.sh <bag_dir_name> [CONTAINER]
#   bag_dir_name = a folder under ~/superodom_ws/field/ (what field_record.sh created)
# NOTE: uses slam_gpu_system + slam-gpu workspace. Does NOT touch the FAST-LIVO2 container.
# set -e is intentionally OFF: the orchestration makes many docker-exec calls that return
# non-zero benignly (pkill with nothing to kill, grep-not-found in waits) and must not abort.
set -uo pipefail
BAGNAME="${1:?usage: run_fastlio.sh <bag_dir_name>}"
CTR="${2:-slam_gpu_system}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW=rmw_cyclonedds_cpp
DOM=0
SRC='source /opt/ros/humble/install/setup.bash && source /opt/slam_ws/install/setup.bash'
ENVV="export RMW_IMPLEMENTATION=$RMW ROS_DOMAIN_ID=$DOM"

FIELD_HOST="$HOME/superodom_ws/field/$BAGNAME"          # source bag (recorded by field_record.sh)
GPU_HOST="$HOME/slam-gpu"                                # slam-gpu workspace (mounted into the container)
STAGE_HOST="$GPU_HOST/bags/$BAGNAME"                     # where we stage the bag for this container
STAGE_CTR="/opt/slam_ws/bags/$BAGNAME"
MON_HOST="$GPU_HOST/bags/fastlio_monitor.py"; MON_CTR="/opt/slam_ws/bags/fastlio_monitor.py"
# bench config = live config + map_en:true. The config/ mount is READ-ONLY, so generate it into
# the writable bags/ dir (on the host) and point FAST-LIO at it via the container path.
CFG_HOST="$GPU_HOST/bags/fast_lio_gpu_bench.yaml"; CFG_CTR="/opt/slam_ws/bags/fast_lio_gpu_bench.yaml"

[ -f "$FIELD_HOST/metadata.yaml" ] || { echo "no bag at $FIELD_HOST (need metadata.yaml)"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$CTR" || docker start "$CTR" >/dev/null
sleep 2

# --- make the slam-gpu bags dir host-writable (container writes it root:root), then stage bag ---
docker exec "$CTR" bash -c "chown -R 1000:1000 /opt/slam_ws/bags 2>/dev/null; true"
mkdir -p "$STAGE_HOST"
rm -f "$STAGE_HOST"/*.db3
# hard-link the db3 (instant, same NVMe) — falls back to copy across filesystems
ln "$FIELD_HOST"/*.db3 "$STAGE_HOST"/ 2>/dev/null || cp "$FIELD_HOST"/*.db3 "$STAGE_HOST"/
cp "$FIELD_HOST/metadata.yaml" "$STAGE_HOST/"
cp "$HERE/fastlio_monitor.py" "$MON_HOST"               # keep the container copy in sync with the repo

# --- bench config: same as the live FAST-LIO config but with the map published so we can save it ---
sed 's/map_en: false/map_en: true/' "$GPU_HOST/config/fast_lio_gpu.yaml" > "$CFG_HOST"

# clock advisory (matches process_field_bag.sh)
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost' for clean results"

echo "=== FAST-LIO on field bag: $BAGNAME ==="
docker exec "$CTR" bash -c "pkill -9 -f 'fastlio_mapping|fastlio_monitor|ros2 bag play' 2>/dev/null; sleep 1; true"
docker exec "$CTR" bash -lc "$SRC && ros2 bag info $STAGE_CTR 2>/dev/null | grep -oE 'Duration:.*' | head -1" | sed 's/^/  /'

# fresh FAST-LIO
docker exec -d "$CTR" bash -lc "$ENVV; $SRC && ros2 run fast_lio fastlio_mapping --ros-args --params-file $CFG_CTR > /tmp/fl.log 2>&1"
sleep 6
# monitor (/Odometry trajectory + /Laser_map -> PCD), subscribed BEFORE replay. Bounded DUR so it can't orphan.
docker exec -d "$CTR" bash -lc "$ENVV; $SRC && python3 $MON_CTR $STAGE_CTR/fastlio_traj.csv $STAGE_CTR/fastlio_map.pcd 1200 > /tmp/fl_mon.log 2>&1"
sleep 2
echo "  replaying (rate 1.0)..."
docker exec -d "$CTR" bash -lc "$ENVV; $SRC && ros2 bag play $STAGE_CTR --rate 1.0 > /tmp/fl_bag.log 2>&1"
sleep 5
# wait for the (non-zombie) bag-play to finish
while docker exec "$CTR" bash -c 'ps -eo stat,args | grep "ros2 bag play" | grep -v grep | awk "\$1 !~ /Z/" | grep -q .'; do sleep 3; done
echo "  bag done; settling + flushing..."
sleep 6
docker exec "$CTR" bash -c "pkill -INT -f fastlio_monitor 2>/dev/null; true"
for w in $(seq 1 15); do
  docker exec "$CTR" bash -c "grep -q 'FASTLIO SUMMARY' /tmp/fl_mon.log 2>/dev/null" && break; sleep 1
done
docker exec "$CTR" bash -c "pkill -9 -f 'fastlio_monitor|fastlio_mapping' 2>/dev/null; true"

echo
echo "===== $BAGNAME — FAST-LIO RESULTS ====="
docker exec "$CTR" bash -c "grep -E 'FASTLIO SUMMARY|SAVED MAP|NO MAP' /tmp/fl_mon.log 2>/dev/null || echo '  (no summary — see /tmp/fl_mon.log, /tmp/fl.log)'"
echo "  outputs (host): $STAGE_HOST/"
echo "    fastlio_map.pcd   — open in CloudCompare/meshlab"
echo "    fastlio_traj.csv  — t,x,y,z,yaw,health,disp_from_start,step"
echo "  EVAL: 'final_disp_from_start' ~ loop-closure drift IF you returned to start;"
echo "        'max_single_step' >0.3m ~ a divergence/jump. Compare vs SuperOdom (process_field_bag.sh)."
