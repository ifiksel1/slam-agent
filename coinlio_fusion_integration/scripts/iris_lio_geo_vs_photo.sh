#!/bin/bash
# DISTRIBUTIONAL geo-vs-photo comparison of IRIS-LIO on a field bag (coinlio_fusion build).
# Runs each mode N times (fresh container per run, INTERLEAVED photo/geo to balance thermal/
# time drift), records the frame-invariant trajectory metrics, and reports runaway RATE +
# drift distribution per mode.
#
# WHY distributional: base EllipseLIO is non-deterministic on knife-edge scenes (~25% runaway
# on 3rd_floor) and neither OMP_NUM_THREADS=1 nor a single-threaded executor fixes it (OMP=1
# still diverged 174m; single-threaded starves the node). So we stop chasing a deterministic
# baseline and instead ask the ACTUAL IRIS-LIO question: does the photometric residual change
# the divergence DISTRIBUTION vs geometry-only? geo = photometric.enable:false (pipeline fully
# off ~= base EllipseLIO); photo = enable:true, photo_scale 1e-9 (v1 stable).
#
# Usage: iris_lio_geo_vs_photo.sh <bag_dir_name> [N=6] [runaway_thresh_m=3.0]
set -uo pipefail
BAGNAME="${1:?usage: iris_lio_geo_vs_photo.sh <bag_dir_name> [N] [thresh_m]}"
N="${2:-6}"
RUNAWAY="${3:-3.0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CTR=coinlio_fusion
IMG=coinlio-fusion:humble
RMW=rmw_cyclonedds_cpp
WS="$HOME/coinlio_fusion_ws"
FIELD_HOST="$HOME/superodom_ws/field"
BAG=/root/field/$BAGNAME
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
CFG=/root/ros2_ws/src/ellipselio/config
SAVE="$WS/results/field/${BAGNAME}_geo_vs_photo"

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }
# GUARD: any OTHER SLAM container alive on the same ROS_DOMAIN_ID=0 (the production ellipselio
# container especially) injects duplicate /ellipselio + /ellipselio_container nodes that answer
# param queries and break component loading -> photometric silently looks absent. Stop them so
# the A/B sees ONLY coinlio_fusion's nodes. (Zombie-nodes-poison-everything gotcha.)
echo "=== stopping other domain-0 SLAM containers (clean A/B slate) ==="
docker stop ellipselio slam_gpu_system fast_livo2_slam_autostart 2>/dev/null || true
mkdir -p "$SAVE"; : > "$SAVE/summary.csv"
cp "$HERE/../../ellipselio_integration/scripts/ellipselio_traj_sampler.py" "$WS/ellipselio_traj_sampler.py"
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

run_one() {  # $1=mode(geo|photo)  $2=index
  local MODE=$1 i=$2 EN=true
  [ "$MODE" = geo ] && EN=false
  local OUTDIR=/root/ros2_ws/results/gvp/${MODE}_$i
  local HOSTDIR=$WS/results/gvp/${MODE}_$i
  echo "########## $MODE run $i/$N (photometric.enable=$EN) ##########"
  docker rm -f "$CTR" 2>/dev/null || true
  docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=$RMW \
    -v "$WS:/root/ros2_ws" -v "$FIELD_HOST:/root/field:ro" \
    "$IMG" sleep infinity >/dev/null
  docker exec "$CTR" bash -c "mkdir -p $OUTDIR"
  # per-run config: photometric on/off (scale stays the file's 1e-9 v1 value)
  docker exec "$CTR" bash -lc "sed 's/enable: true/enable: $EN/' $CFG/os1_64_ouster.yaml > $CFG/_gvp_$MODE.yaml"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
    ros2 launch ellipselio ellipselio_standalone.launch.py \
      config_path:=$CFG config_file:=_gvp_$MODE.yaml use_sim_time:=true rviz:=false > $OUTDIR/node.log 2>&1"
  sleep 8
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
    python3 /root/ros2_ws/ellipselio_traj_sampler.py $OUTDIR/trajectory.csv 1200 /ellipselio_odom > $OUTDIR/sampler.log 2>&1"
  sleep 2
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
    ros2 bag play $BAG --clock --rate 1.0 > $OUTDIR/play.log 2>&1"
  sleep 5
  while docker exec "$CTR" bash -c 'ps -eo stat,args | grep "ros2 bag play" | grep -v grep | awk "\$1 !~ /Z/" | grep -q .'; do sleep 3; done
  sleep 4
  docker exec "$CTR" bash -c "pkill -INT -f ellipselio_traj_sampler.py 2>/dev/null" || true
  for w in $(seq 1 20); do docker exec "$CTR" bash -c "grep -q 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null" && break; sleep 1; done
  docker exec "$CTR" bash -c "pkill -9 -f ellipselio_traj_sampler.py 2>/dev/null" || true
  cp "$HOSTDIR/trajectory.csv" "$SAVE/${MODE}_run${i}.csv" 2>/dev/null || true
  local SUM=$(docker exec "$CTR" bash -c "grep -h 'ELLIPSELIO TRAJ SUMMARY' $OUTDIR/sampler.log 2>/dev/null | tail -1")
  local FD=$(echo "$SUM" | grep -oE 'final_disp_from_start=[0-9.]+' | grep -oE '[0-9.]+$')
  local PK=$(echo "$SUM" | grep -oE 'peak_excursion=[0-9.]+' | grep -oE '[0-9.]+$')
  local MS=$(echo "$SUM" | grep -oE 'max_single_step=[0-9.]+' | grep -oE '[0-9.]+$')
  local NS=$(echo "$SUM" | grep -oE 'samples=[0-9]+' | grep -oE '[0-9]+$')
  local PHACT=$(docker exec "$CTR" bash -c "grep -c -iE '\[photometric\]' $OUTDIR/node.log 2>/dev/null" || echo 0)
  echo ">>> $MODE run$i: final_disp=${FD:-?}m peak=${PK:-?}m max_step=${MS:-?}m samples=${NS:-?} photo_log_lines=$PHACT"
  echo "$MODE,$i,${FD:-NA},${PK:-NA},${MS:-NA},${NS:-NA},$PHACT" >> "$SAVE/summary.csv"
}

# interleave photo/geo
for i in $(seq 1 "$N"); do run_one photo "$i"; run_one geo "$i"; done

echo; echo "===== GEO vs PHOTO DISTRIBUTION ($BAGNAME, N=$N each, runaway>${RUNAWAY}m) ====="
python3 - "$SAVE/summary.csv" "$RUNAWAY" <<'PY'
import sys, statistics as st
rows=[l.strip().split(',') for l in open(sys.argv[1]) if l.strip()]
th=float(sys.argv[2])
for mode in ('geo','photo'):
    fds=[float(r[2]) for r in rows if r[0]==mode and r[2]!='NA']
    if not fds: print(f"{mode}: no data"); continue
    ra=[d for d in fds if d>th]
    print(f"{mode:5s} n={len(fds)}  runaway={len(ra)}/{len(fds)} ({100*len(ra)/len(fds):.0f}%)  "
          f"final_disp median={st.median(fds):.2f} max={max(fds):.2f} min={min(fds):.3f}  "
          f"runaway_vals={[round(d,1) for d in ra]}")
PY
echo "  saved: $SAVE/summary.csv + {geo,photo}_run*.csv"
