#!/bin/bash
# OFFLINE (at home, with the agent): replay a field bag through SuperOdom, build + save the
# map PCD, record the trajectory, and print evaluation metrics (path, loop-closure drift,
# divergence/jump check). Re-run freely — deterministic from the same bag. Boost the clock
# first for clean results (see jetson_cpu_clock_boost memory).
#
# Usage: process_field_bag.sh <bag_dir_name> [CONTAINER]
#   bag_dir_name = a folder under ~/superodom_ws/field/ (what field_record.sh created)
set -euo pipefail
BAGNAME="${1:?usage: process_field_bag.sh <bag_dir_name>}"
CTR="${2:-superodom}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW=rmw_cyclonedds_cpp
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
SO_CFG=/root/ros2_ws/src/SuperOdom/super_odometry/config
LAUNCH="ros2 launch super_odometry os1_128.launch.py config_file:=$SO_CFG/os1_64.yaml calibration_file:=$SO_CFG/ouster/os1_64_calibration.yaml"
BAG=/root/ros2_ws/field/$BAGNAME
OUTDIR=/root/ros2_ws/results/field/$BAGNAME
HOSTOUT="$HOME/superodom_ws/results/field/$BAGNAME"

docker ps --format '{{.Names}}' | grep -qx "$CTR" || { echo "container '$CTR' not running"; exit 1; }
docker exec "$CTR" bash -c "[ -f $BAG/metadata.yaml ]" || { echo "no bag at $BAG (need metadata.yaml)"; exit 1; }
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"; mkdir -p "$HOSTOUT"

# clock advisory
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost' for clean results"

bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null
echo "=== processing field bag: $BAGNAME ==="
DUR=$(docker exec "$CTR" bash -lc "$SRC && ros2 bag info $BAG 2>/dev/null | grep -oE 'Duration:.*' | head -1")
echo "  $DUR"

# fresh SuperOdom
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && $LAUNCH > $OUTDIR/superodom.log 2>&1"
sleep 7
# trajectory monitor (loop-closure + divergence) and map saver, both subscribed BEFORE replay.
# BOUNDED DUR (not 9000) so a helper can never orphan into a multi-hour spin if this script dies.
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/live_monitor.py $OUTDIR/trajectory.csv 1200 > $OUTDIR/monitor.log 2>&1"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/save_map.py $OUTDIR/map.pcd /laser_cloud_map 1200 > $OUTDIR/savemap.log 2>&1"
sleep 2

echo "  replaying (rate 1.0)..."
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 bag play $BAG --rate 1.0 > $OUTDIR/bagplay.log 2>&1"
# wait for the bag to finish: poll until no NON-ZOMBIE bag-play remains (zombie-safe)
sleep 5
while docker exec "$CTR" bash -c 'ps -eo pid,stat,args | grep "ros2 bag play" | grep -v grep | awk "\$2 !~ /Z/" | grep -q .'; do sleep 3; done
echo "  bag done; letting map settle, then flushing..."
sleep 6
# signal monitor + map saver (they write their outputs on SIGINT), then WAIT for the output
# lines to appear (they write fast), then SIGKILL as a fallback so nothing can orphan.
docker exec "$CTR" bash -c "pkill -INT -f live_monitor.py 2>/dev/null; pkill -INT -f save_map.py 2>/dev/null" || true
for w in $(seq 1 15); do
  docker exec "$CTR" bash -c "grep -q '^SAVED MAP\|^NO MAP' $OUTDIR/savemap.log 2>/dev/null && grep -q 'LIVE MOTION SUMMARY' $OUTDIR/monitor.log 2>/dev/null" && break
  sleep 1
done
docker exec "$CTR" bash -c "pkill -9 -f 'live_monitor.py|save_map.py' 2>/dev/null" || true   # fallback: never orphan
bash "$HERE/superodom_cleanup.sh" "$CTR" >/dev/null || true

echo
echo "===== $BAGNAME RESULTS ====="
docker exec "$CTR" bash -c "grep -h 'LIVE MOTION SUMMARY' -A1 $OUTDIR/monitor.log 2>/dev/null | tail -1 || true"
docker exec "$CTR" bash -c "grep -h '^SAVED MAP\|^NO MAP' $OUTDIR/savemap.log 2>/dev/null | tail -1 || true"
echo "  outputs (host): $HOSTOUT/"
echo "    map.pcd       — open in CloudCompare/meshlab"
echo "    trajectory.csv — t,x,y,z,yaw,health,disp_from_start,step"
echo "  EVAL: 'final_disp_from_start' ~ loop-closure drift IF you returned to start;"
echo "        'max_single_step' >0.3m ~ a divergence/jump (the SuperOdom transient)."
