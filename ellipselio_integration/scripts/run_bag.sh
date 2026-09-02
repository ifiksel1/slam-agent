#!/bin/bash
# OFFLINE: replay a recorded field bag through EllipseLIO and print frame-invariant
# evaluation metrics (path / peak excursion / final disp / max single step) — directly
# comparable to the SuperOdom & FAST-LIO numbers from benchmark_bag.sh.
#
# Deterministic from the same bag; re-run freely. Boost the clock first for clean results:
#   sudo ~/superodom_ws/set_cpu_clock.sh boost
#
# Usage: run_bag.sh <bag_dir_name>   (a folder under ~/superodom_ws/field/)
set -uo pipefail
BAGNAME="${1:?usage: run_bag.sh <bag_dir_name> [config_file] [out_tag]}"
CFG_FILE="${2:-os1_64_ouster.yaml}"   # A/B: swap the SLAM config (default = baseline)
TAG="${3:-}"                          # A/B: isolate outputs per arm/run (e.g. _allan_run3)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTR=ellipselio
RMW=rmw_cyclonedds_cpp
WS="${ELLIPSELIO_WS:-$HOME/ellipselio_ws}"
IMAGE="${ELLIPSELIO_IMAGE:-ellipselio:humble}"
FIELD_HOST="${FIELD_HOST:-${SUPERODOM_WS:-$HOME/superodom_ws}/field}"
BAG=/root/field/$BAGNAME
OUTDIR=/root/ros2_ws/results/field/${BAGNAME}${TAG}
HOSTOUT="$WS/results/field/${BAGNAME}${TAG}"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
CFG=/root/ros2_ws/src/ellipselio/config

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }
mkdir -p "$HOSTOUT"
# put the standalone sampler where the (mounted) workspace can see it
cp "$HERE/ellipselio_traj_sampler.py" "$WS/ellipselio_traj_sampler.py"

# Recreate the ellipselio container WITH the field bags mounted read-only (the live driver
# must stop for replay anyway; EllipseLIO stays the active stack). Whole-ws mount persists build/install.
echo "=== (re)starting $CTR with bag mount ==="
docker rm -f "$CTR" 2>/dev/null || true
docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=$RMW \
  -v "$WS:/root/ros2_ws" \
  -v "$FIELD_HOST:/root/field:ro" \
  "$IMAGE" sleep infinity
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"

# clock advisory
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

DUR=$(docker exec "$CTR" bash -lc "$SRC && ros2 bag info $BAG 2>/dev/null | grep -oE 'Duration:.*' | head -1")
echo "  bag: $BAGNAME   $DUR"

# 1. EllipseLIO in BAG mode: use_sim_time:=true so it follows the bag's /clock.
echo "=== launching EllipseLIO (use_sim_time:=true) ==="
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_path:=$CFG config_file:=$CFG_FILE \
    use_sim_time:=true rviz:=false > $OUTDIR/ellipselio.log 2>&1"
sleep 8

# 2. trajectory sampler on /ellipselio_odom (bounded DUR so it can never orphan)
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  python3 /root/ros2_ws/ellipselio_traj_sampler.py $OUTDIR/trajectory.csv 1200 /ellipselio_odom \
    > $OUTDIR/sampler.log 2>&1"
sleep 2

# 3. replay the bag with --clock (drives sim time) at rate 1.0
echo "=== replaying (rate 1.0, --clock) ==="
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 bag play $BAG --clock --rate 1.0 > $OUTDIR/bagplay.log 2>&1"

# wait for bag-play to finish (zombie-safe: ignore Z-state procs)
sleep 5
while docker exec "$CTR" bash -c 'ps -eo stat,args | grep "ros2 bag play" | grep -v grep | awk "\$1 !~ /Z/" | grep -q .'; do sleep 3; done
echo "  bag done; settling, flushing sampler..."
sleep 4
docker exec "$CTR" bash -c "pkill -INT -f ellipselio_traj_sampler.py 2>/dev/null" || true
for w in $(seq 1 20); do
  docker exec "$CTR" bash -c "grep -q 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null" && break
  sleep 1
done
docker exec "$CTR" bash -c "pkill -9 -f ellipselio_traj_sampler.py 2>/dev/null" || true

echo
echo "===== $BAGNAME — EllipseLIO RESULTS ====="
docker exec "$CTR" bash -c "grep -h 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null | tail -1 || echo '  (no summary — check $OUTDIR/ellipselio.log)'"
echo "  outputs (host): $HOSTOUT/   (trajectory.csv: t,x,y,z,yaw,disp_from_start,step)"
echo "  COMPARE vs SuperOdom apartment: loop 0.13m / peak 9.60m / max-step 0.20m"
echo "          vs FAST-LIO  apartment: loop 1.85m / peak 9.55m / max-step 0.14m"
