#!/bin/bash
# WATCH + ANALYZE: replay a recorded field bag through EllipseLIO ONCE with a live Foxglove
# bridge (so you can watch in Foxglove) AND the frame-invariant trajectory sampler (so you
# get the same path / peak excursion / final disp / max-step metrics as run_bag.sh).
# Single monotonic play (NOT --loop: a looping bag rewinds sim-time and floods the node).
#
# Foxglove: connect to ws://<jetson-ip>:8765, Fixed Frame = map_view (upright; 180deg pitch).
# Boost the clock first for clean (less divergence-prone) results:
#   sudo ~/superodom_ws/set_cpu_clock.sh boost
#
# Usage: run_bag_foxglove.sh <bag_dir_name>   (a folder under ~/superodom_ws/field/)
set -uo pipefail
BAGNAME="${1:?usage: run_bag_foxglove.sh <bag_dir_name>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTR=ellipselio
RMW=rmw_cyclonedds_cpp
WS="${ELLIPSELIO_WS:-$HOME/ellipselio_ws}"
IMAGE="${ELLIPSELIO_IMAGE:-ellipselio:humble}"
FIELD_HOST="${FIELD_HOST:-${SUPERODOM_WS:-$HOME/superodom_ws}/field}"
BAG=/root/field/$BAGNAME
OUTDIR=/root/ros2_ws/results/field/$BAGNAME
HOSTOUT="$WS/results/field/$BAGNAME"
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
CFG=/root/ros2_ws/src/ellipselio/config

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }
mkdir -p "$HOSTOUT"
# stage helper nodes + foxglove params into the mounted workspace
cp "$HERE/ellipselio_traj_sampler.py" "$WS/ellipselio_traj_sampler.py"
cp "$HERE/ellipselio_diag.py"         "$WS/ellipselio_diag.py"
cp "$HERE/odom_to_path.py"            "$WS/odom_to_path.py"
cp "$HERE/cloud_throttle.py"          "$WS/cloud_throttle.py"
cp "$HERE/../config/foxglove_params.yaml" "$WS/foxglove_params.yaml"

# Recreate the container WITH the field bags mounted read-only (the live driver must stop
# for replay anyway). Whole-ws mount persists build/install.
echo "=== (re)starting $CTR with bag mount ==="
# OMP_NUM_THREADS must be a CONTAINER-level -e: launch_ros spawns component_container_mt with a
# sanitized env, so an `export` in the launching shell does NOT reach it (only docker -e does,
# same as RMW_IMPLEMENTATION). Set ELL_OMP_THREADS=1 to force deterministic OpenMP reductions.
OMPENV=""; [ -n "${ELL_OMP_THREADS:-}" ] && OMPENV="-e OMP_NUM_THREADS=${ELL_OMP_THREADS}"
docker rm -f "$CTR" 2>/dev/null || true
docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=$RMW $OMPENV \
  -v "$WS:/root/ros2_ws" \
  -v "$FIELD_HOST:/root/field:ro" \
  "$IMAGE" sleep infinity
docker exec "$CTR" bash -c "mkdir -p $OUTDIR"

F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"
DUR=$(docker exec "$CTR" bash -lc "$SRC && ros2 bag info $BAG 2>/dev/null | grep -oE 'Duration:.*' | head -1")
echo "  bag: $BAGNAME   $DUR"

# 1. EllipseLIO in BAG mode (follows the bag's /clock).
#    Determinism knobs (env-overridable, defaults = stock multi-threaded behavior):
#      ELL_OMP_THREADS   -> export OMP_NUM_THREADS (e.g. 1 to force deterministic OpenMP reductions)
#      ELL_CONTAINER_EXEC-> component_container_mt (default) | component_container (single-threaded)
CEXEC="${ELL_CONTAINER_EXEC:-component_container_mt}"   # component_container starves EllipseLIO (FAST-LIO needs the MT executor); keep _mt
echo "=== launching EllipseLIO (use_sim_time:=true, container=$CEXEC, OMP=${ELL_OMP_THREADS:-default}) ==="
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_path:=$CFG config_file:=os1_64_ouster.yaml \
    container_executable:=$CEXEC \
    use_sim_time:=true rviz:=false > $OUTDIR/ellipselio.log 2>&1"
sleep 8

# 2. viz helpers (native) + map_view upright TF (180 pitch, /tf_static timeless) + foxglove bridge
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/odom_to_path.py > $OUTDIR/odom_to_path.log 2>&1"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && python3 /root/ros2_ws/cloud_throttle.py 5 > $OUTDIR/cloud_throttle.log 2>&1"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 3.14159265 --yaw 0 \
    --frame-id map_view --child-frame-id odom_ellipselio > /dev/null 2>&1"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 run foxglove_bridge foxglove_bridge --ros-args \
    --params-file /root/ros2_ws/foxglove_params.yaml -p use_sim_time:=true > $OUTDIR/foxglove.log 2>&1"
echo "=== Foxglove: ws://<jetson-ip>:8765  Fixed Frame = map_view  (watch /cloud_scan_lite, /cloud_map Decay 0, /ellipselio_path) ==="
sleep 3

# 3. trajectory sampler + /analytics diagnostic (both bounded DUR so they can never orphan).
#    diag correlates obs_score/num_feats with disp/step -> shows if observability COLLAPSES
#    at a runaway (e.g. the terminal stationary endpoint).
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  python3 /root/ros2_ws/ellipselio_traj_sampler.py $OUTDIR/trajectory.csv 1200 /ellipselio_odom \
    > $OUTDIR/sampler.log 2>&1"
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  python3 /root/ros2_ws/ellipselio_diag.py $OUTDIR/analytics.csv 1200 \
    > $OUTDIR/diag.log 2>&1"
sleep 2

# 4. replay ONCE with --clock at rate 1.0
echo "=== replaying (rate 1.0, --clock) — watch in Foxglove ==="
docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
  ros2 bag play $BAG --clock --rate 1.0 > $OUTDIR/bagplay.log 2>&1"

sleep 5
while docker exec "$CTR" bash -c 'ps -eo stat,args | grep "ros2 bag play" | grep -v grep | awk "\$1 !~ /Z/" | grep -q .'; do sleep 3; done
echo "  bag done; settling, flushing sampler..."
sleep 4
docker exec "$CTR" bash -c "pkill -INT -f 'ellipselio_traj_sampler.py|ellipselio_diag.py' 2>/dev/null" || true
for w in $(seq 1 20); do
  docker exec "$CTR" bash -c "grep -q 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null" && break
  sleep 1
done
docker exec "$CTR" bash -c "pkill -9 -f 'ellipselio_traj_sampler.py|ellipselio_diag.py' 2>/dev/null" || true

echo
echo "===== $BAGNAME — EllipseLIO RESULTS ====="
docker exec "$CTR" bash -c "grep -h 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null | tail -1 || echo '  (no summary — check $OUTDIR/ellipselio.log)'"
echo "  outputs (host): $HOSTOUT/   (trajectory.csv: t,x,y,z,yaw,disp_from_start,step)"
echo "  Foxglove bridge + EllipseLIO left running for review."
