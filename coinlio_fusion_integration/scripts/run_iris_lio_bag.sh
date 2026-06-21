#!/bin/bash
# OFFLINE: replay a recorded field bag through IRIS-LIO (photometric-fusion EllipseLIO in the
# coinlio_fusion container) and print the SAME frame-invariant evaluation metrics as the other
# frameworks — directly comparable to the SuperOdom / FAST-LIO / EllipseLIO numbers from
# benchmark_bag.sh + ellipselio run_bag.sh (same ellipselio_traj_sampler.py on /ellipselio_odom).
#
# IRIS-LIO == EllipseLIO IKFoM + COIN-LIO LiDAR-intensity photometric residual (photometric.enable=true
# in coinlio_fusion_ws/src/ellipselio/config/os1_64_ouster.yaml, photo_scale=1e-9 = the v1 stable config).
#
# CAVEAT (see coinlio_ellipselio_photometric_fusion memory, GATE-0 2026-06-13): on a DEGENERATE
# scene the photometric pipeline can destabilize the estimate NON-DETERMINISTICALLY even at zero
# weight (timing/threading, not the math). On a feature-rich bag like an indoor floor it is stable,
# but if IRIS-LIO disagrees wildly with EllipseLIO, suspect the pipeline transient, not the scene.
#
# Deterministic-ish from the same bag; re-run to check repeatability. Boost the clock first:
#   sudo ~/superodom_ws/set_cpu_clock.sh boost
#
# Usage: run_iris_lio_bag.sh <bag_dir_name>   (a folder under ~/superodom_ws/field/)
set -uo pipefail
BAGNAME="${1:?usage: run_iris_lio_bag.sh <bag_dir_name>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTR=coinlio_fusion
IMG=coinlio-fusion:humble
RMW=rmw_cyclonedds_cpp
WS="$HOME/coinlio_fusion_ws"
FIELD_HOST="$HOME/superodom_ws/field"
BAG=/root/field/$BAGNAME
OUTDIR=/root/ros2_ws/results/field/$BAGNAME
HOSTOUT="$WS/results/field/$BAGNAME"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
CFG=/root/ros2_ws/src/ellipselio/config

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }
mkdir -p "$HOSTOUT"
# put the SAME standalone sampler the ellipselio/superodom runs use into the mounted workspace
cp "$HERE/../../ellipselio_integration/scripts/ellipselio_traj_sampler.py" "$WS/ellipselio_traj_sampler.py"

# Recreate coinlio_fusion WITH the field bags mounted read-only (keep the existing dev mounts so
# nothing else breaks). Whole-ws mount persists build/install; this is the same pattern dev_up.sh
# uses, just with the field dir added.
echo "=== (re)starting $CTR with field-bag mount ==="
docker rm -f "$CTR" 2>/dev/null || true
docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=$RMW \
  -v "$WS:/root/ros2_ws" \
  -v "$FIELD_HOST:/root/field:ro" \
  -v /home/dev/slam-gpu/bags:/root/bags:ro \
  "$IMG" sleep infinity
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"

# confirm photometric fusion is actually ON (this is what makes it IRIS-LIO, not plain EllipseLIO)
PHOTO=$(grep -A1 'photometric:' "$WS/src/ellipselio/config/os1_64_ouster.yaml" | grep -oE 'enable:\s*\w+' | head -1)
echo "  photometric: $PHOTO   (must be 'enable: true' for IRIS-LIO)"

# clock advisory
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

DUR=$(docker exec "$CTR" bash -lc "$SRC && ros2 bag info $BAG 2>/dev/null | grep -oE 'Duration:.*' | head -1")
echo "  bag: $BAGNAME   $DUR"

# 1. IRIS-LIO in BAG mode: use_sim_time:=true so it follows the bag's /clock.
echo "=== launching IRIS-LIO (photometric EllipseLIO, use_sim_time:=true) ==="
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_path:=$CFG config_file:=os1_64_ouster.yaml \
    use_sim_time:=true rviz:=false > $OUTDIR/iris_lio.log 2>&1"
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
docker exec "$CTR" bash -c "pkill -9 -f 'ellipselio_traj_sampler.py' 2>/dev/null" || true
# stop the node so it can't hold the next run's resources
docker exec "$CTR" bash -c "pkill -9 -f 'ellipselio_standalone|laserMapping|ellipselio' 2>/dev/null" || true

echo
echo "===== $BAGNAME — IRIS-LIO RESULTS ====="
docker exec "$CTR" bash -c "grep -h 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null | tail -1 || echo '  (no summary — check $OUTDIR/iris_lio.log)'"
echo "  photometric activity:"
docker exec "$CTR" bash -c "grep -iE '\[photometric\]' $OUTDIR/iris_lio.log 2>/dev/null | tail -3 | sed 's/^/    /' || true"
docker exec "$CTR" bash -c "grep -iE 'nan|terminate|abort|exception|runaway' $OUTDIR/iris_lio.log 2>/dev/null | tail -3 | sed 's/^/    ERR: /' || true"
echo "  outputs (host): $HOSTOUT/   (trajectory.csv: t,x,y,z,yaw,disp_from_start,step)"
echo "  COMPARE vs the EllipseLIO run (run_bag.sh) — same sampler, same metrics."
