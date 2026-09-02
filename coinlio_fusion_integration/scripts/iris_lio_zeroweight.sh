#!/bin/bash
# ZERO-WEIGHT DIAGNOSTIC (3-way) — disambiguates WHY the photometric pipeline raises IRIS-LIO's
# runaway rate on 3rd_floor (geo 17% vs photo 67%, unchanged by the timing decouple). Runs THREE
# interleaved modes, fresh container per run, same frame-invariant metrics:
#   geo   = photometric.enable:false                 (pipeline OFF  -> base EllipseLIO, ~17%)
#   zw    = enable:true,  photo_scale 1.0e-15         (pipeline fully ON: image build + features +
#                                                       residual rows COMPUTED, but weight ~0 so the
#                                                       residual contributes ~nothing to the state)
#   photo = enable:true,  photo_scale 1.0e-9          (the live config, ~67%)
#
# READOUT:
#   zw ~= geo  (17%)  => it's the residual WEIGHT/MATH (1e-9 perturbs the knife-edge IEKF)
#                        => FIX = gate/tune the weight (obs_min hard-gate; build itself is harmless)
#   zw ~= photo(67%)  => it's the pipeline COMPUTE LOAD (build steals IEKF headroom regardless of
#                        weight) => FIX = gate the BUILD itself (skip createImages when healthy)
# Directly tests the old GATE-0 "even at zero weight it destabilized -> timing" claim.
#
# Boost the clock first: sudo ~/superodom_ws/set_cpu_clock.sh boost
# Usage: iris_lio_zeroweight.sh <bag_dir_name> [N=6] [runaway_thresh_m=3.0]
set -uo pipefail
BAGNAME="${1:?usage: iris_lio_zeroweight.sh <bag_dir_name> [N] [thresh_m]}"
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
SAVE="$WS/results/field/${BAGNAME}_zeroweight"

[ -f "$FIELD_HOST/$BAGNAME/metadata.yaml" ] || { echo "no bag at $FIELD_HOST/$BAGNAME"; exit 1; }
# stop other domain-0 SLAM containers (zombie-nodes-poison-everything gotcha)
echo "=== stopping other domain-0 SLAM containers (clean slate) ==="
docker stop ellipselio slam_gpu_system fast_livo2_slam_autostart 2>/dev/null || true
mkdir -p "$SAVE"; : > "$SAVE/summary.csv"
cp "$HERE/../../ellipselio_integration/scripts/ellipselio_traj_sampler.py" "$WS/ellipselio_traj_sampler.py"
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

run_one() {  # $1=mode(geo|zw|photo)  $2=index
  local MODE=$1 i=$2 EN=true SCALE=1.0e-9
  case "$MODE" in
    geo)   EN=false ;;
    zw)    EN=true; SCALE=1.0e-15 ;;
    photo) EN=true; SCALE=1.0e-9 ;;
  esac
  local OUTDIR=/root/ros2_ws/results/zw/${MODE}_$i
  local HOSTDIR=$WS/results/zw/${MODE}_$i
  echo "########## $MODE run $i/$N (enable=$EN photo_scale=$SCALE) ##########"
  docker rm -f "$CTR" 2>/dev/null || true
  docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
    -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=$RMW \
    -v "$WS:/root/ros2_ws" -v "$FIELD_HOST:/root/field:ro" \
    "$IMG" sleep infinity >/dev/null
  docker exec "$CTR" bash -c "mkdir -p $OUTDIR"
  # per-run config: toggle enable AND photo_scale
  docker exec "$CTR" bash -lc "sed -e 's/enable: true/enable: $EN/' -e 's/photo_scale: 1.0e-9/photo_scale: $SCALE/' $CFG/os1_64_ouster.yaml > $CFG/_zw_$MODE.yaml"
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && \
    ros2 launch ellipselio ellipselio_standalone.launch.py \
      config_path:=$CFG config_file:=_zw_$MODE.yaml use_sim_time:=true rviz:=false > $OUTDIR/node.log 2>&1"
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
  local NR=$(docker exec "$CTR" bash -c "grep -oE 'n_rows[= ][0-9]+' $OUTDIR/node.log 2>/dev/null | grep -oE '[0-9]+' | tail -1" 2>/dev/null)
  echo ">>> $MODE run$i: final_disp=${FD:-?}m peak=${PK:-?}m max_step=${MS:-?}m samples=${NS:-?} photo_n_rows=${NR:-NA}"
  echo "$MODE,$i,${FD:-NA},${PK:-NA},${MS:-NA},${NS:-NA},${NR:-NA}" >> "$SAVE/summary.csv"
}

# interleave photo / zw / geo to balance thermal+time drift across modes
for i in $(seq 1 "$N"); do run_one photo "$i"; run_one zw "$i"; run_one geo "$i"; done

echo; echo "===== ZERO-WEIGHT DIAGNOSTIC ($BAGNAME, N=$N each, runaway>${RUNAWAY}m) ====="
python3 - "$SAVE/summary.csv" "$RUNAWAY" <<'PY'
import sys, statistics as st
rows=[l.strip().split(',') for l in open(sys.argv[1]) if l.strip()]
th=float(sys.argv[2])
for mode in ('geo','zw','photo'):
    fds=[float(r[2]) for r in rows if r[0]==mode and len(r)>2 and r[2] not in ('NA','')]
    if not fds: print(f"{mode}: no data"); continue
    ra=[d for d in fds if d>th]
    nr=[r[6] for r in rows if r[0]==mode and len(r)>6 and r[6] not in ('NA','')]
    print(f"{mode:5s} n={len(fds)}  runaway={len(ra)}/{len(fds)} ({100*len(ra)/len(fds):.0f}%)  "
          f"final_disp median={st.median(fds):.2f} max={max(fds):.2f} min={min(fds):.3f}  "
          f"photo_n_rows~{nr[-1] if nr else 'NA'}  runaway_vals={[round(d,1) for d in ra]}")
print()
print("READOUT: zw~=geo => residual WEIGHT/math (gate the weight); zw~=photo => pipeline COMPUTE load (gate the build)")
PY
echo "  saved: $SAVE/summary.csv + {geo,zw,photo}_run*.csv"
