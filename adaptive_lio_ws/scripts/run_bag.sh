#!/usr/bin/env bash
# DISTRIBUTIONAL Adaptive-LIO bag-replay battery, containerized. Reports the SAME frame-invariant
# trajectory metrics + runaway RATE as run_coinlio_bag.sh / the EllipseLIO geo-vs-photo battery, so
# Adaptive-LIO is directly comparable to COIN-LIO / EllipseLIO / SuperOdom / FAST-LIO on the same bag.
#
# Adaptive-LIO is ROS1; field bags are ROS2 sqlite3 -> use a pre-converted ROS1 bag
# (rosbags-convert <ros2_dir> --dst <name>.bag). The bag must carry /ouster/points + /ouster/imu.
#
# Boost the clock first:  sudo ~/superodom_ws/set_cpu_clock.sh boost
# Usage: run_bag.sh <ros1_bag_path> [N=6] [runaway_thresh_m=3.0] [rate=1.0]
set -uo pipefail
BAG="${1:?usage: run_bag.sh <ros1_bag_path> [N] [runaway_m] [rate]}"
N="${2:-6}"; RUNAWAY="${3:-3.0}"; RATE="${4:-1.0}"
[ -f "$BAG" ] || { echo "no ROS1 bag at $BAG (convert first: rosbags-convert <ros2_dir> --dst $BAG)"; exit 1; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # adaptive_lio_ws/
IMAGE="${IMAGE:-adaptive-lio:noetic}"
CTR="${CTR:-adaptive_lio}"
BAGDIR="$(cd "$(dirname "$BAG")" && pwd)"; BAGBASE="$(basename "$BAG")"

# start (or reuse) the container: host net for ROS, mount bag dir + this ws (as a pkg) + live config
if ! docker ps --format '{{.Names}}' | grep -qx "$CTR"; then
  docker rm -f "$CTR" >/dev/null 2>&1 || true
  echo "=== starting container $CTR from $IMAGE ==="
  docker run -d --name "$CTR" --network host --privileged \
    -v "$BAGDIR":/bags:ro \
    -v "$HERE":/root/catkin_ws/src/adaptive_lio_ws:ro \
    -v "$HERE/config/mapping_ouster.yaml":/root/catkin_ws/src/adaptive_lio/config/mapping_m.yaml:ro \
    "$IMAGE" sleep infinity >/dev/null
fi

# the whole battery runs INSIDE the container; pass params through the environment.
# -i is REQUIRED: without it docker exec does not attach stdin, so `bash -s <<INNER` reads EOF
# immediately and silently runs nothing (exit 0, no roscore, no results).
docker exec -i -e BAGBASE="$BAGBASE" -e N="$N" -e RUNAWAY="$RUNAWAY" -e RATE="$RATE" \
  -e TAG="$(basename "$BAG" .bag)" "$CTR" bash -s <<'INNER'
set -uo pipefail
# ROS setup.bash is NOT set -u safe (references ROS_MASTER_URI etc. before defining them) — relax
# nounset around sourcing, same as run_coinlio_bag.sh, or the inner script aborts before any work.
set +u
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
set -u
# make the mounted integration package discoverable to roslaunch/$(find) without a rebuild
export ROS_PACKAGE_PATH=/root/catkin_ws/src/adaptive_lio_ws:$ROS_PACKAGE_PATH
BAG="/bags/$BAGBASE"
SAVE="/root/results/adaptive_lio/$TAG"; mkdir -p "$SAVE"; : > "$SAVE/summary.csv"
SAMPLER=/root/catkin_ws/src/adaptive_lio_ws/scripts/adaptive_lio_traj_sampler.py

if ! rostopic list >/dev/null 2>&1; then
  echo "=== starting roscore ==="; roscore >/tmp/alio_roscore.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi
rosparam set /use_sim_time true

cleanup_nodes() {
  pkill -INT -f 'adaptive_lio_traj_sampler.py' 2>/dev/null || true
  pkill -INT -f 'adaptive_lio.launch'          2>/dev/null || true
  pkill -INT -f 'rosbag play'                  2>/dev/null || true
  for w in $(seq 1 40); do
    pgrep -f 'adaptive_lio.launch' >/dev/null 2>&1 || pgrep -f 'type=adaptive_lio' >/dev/null 2>&1 || break
    sleep 0.5
  done
  pkill -9 -f 'adaptive_lio.launch'          2>/dev/null || true
  pkill -9 -f 'rosbag play'                  2>/dev/null || true
  pkill -9 -f 'adaptive_lio_traj_sampler.py' 2>/dev/null || true
  # the estimator binary is literally named "adaptive_lio"; match the ROS-launched instance only
  pkill -9 -f '__name:=adaptive_lio'         2>/dev/null || true
  sleep 1
  yes | timeout 8 rosnode cleanup >/dev/null 2>&1 || true
}

run_one() {
  local i=$1; local OUTDIR="$SAVE/run_$i"; mkdir -p "$OUTDIR"
  echo "########## adaptive_lio run $i/$N ##########"
  cleanup_nodes
  roslaunch adaptive_lio_ws adaptive_lio.launch use_sim_time:=true rviz:=false > "$OUTDIR/node.log" 2>&1 &
  sleep 8
  python3 "$SAMPLER" "$OUTDIR/trajectory.csv" 1200 /odom > "$OUTDIR/sampler.log" 2>&1 &
  local SPID=$!
  sleep 2
  rosbag play --clock --quiet --rate "$RATE" "$BAG" > "$OUTDIR/play.log" 2>&1 &
  local PLAYPID=$!
  sleep 5
  while kill -0 "$PLAYPID" 2>/dev/null; do sleep 3; done
  sleep 4
  kill -INT "$SPID" 2>/dev/null || true
  for w in $(seq 1 20); do grep -q 'ADAPTIVE_LIO TRAJ SUMMARY' "$OUTDIR/sampler.log" 2>/dev/null && break; sleep 1; done
  cleanup_nodes
  local SUM=$(grep -h 'ADAPTIVE_LIO TRAJ SUMMARY' "$OUTDIR/sampler.log" 2>/dev/null | tail -1)
  local FD=$(echo "$SUM" | grep -oE 'final_disp_from_start=[0-9.]+' | grep -oE '[0-9.]+$')
  local PK=$(echo "$SUM" | grep -oE 'peak_excursion=[0-9.]+'       | grep -oE '[0-9.]+$')
  local MS=$(echo "$SUM" | grep -oE 'max_single_step=[0-9.]+'      | grep -oE '[0-9.]+$')
  local NS=$(echo "$SUM" | grep -oE 'samples=[0-9]+'              | grep -oE '[0-9]+$')
  echo ">>> adaptive_lio run$i: final_disp=${FD:-?}m peak=${PK:-?}m max_step=${MS:-?}m samples=${NS:-?}"
  echo "adaptive_lio,$i,${FD:-NA},${PK:-NA},${MS:-NA},${NS:-NA}" >> "$SAVE/summary.csv"
}

trap 'cleanup_nodes 2>/dev/null; exit 130' INT TERM
for i in $(seq 1 "$N"); do run_one "$i"; done

echo; echo "===== ADAPTIVE-LIO DISTRIBUTION ($TAG, N=$N) ====="
python3 - "$SAVE/summary.csv" "$RUNAWAY" <<'PY'
import sys, statistics as st
rows=[l.strip().split(',') for l in open(sys.argv[1]) if l.strip()]
th=float(sys.argv[2]); recs=[]
for r in rows:
    if len(r)<6: continue
    try: fd=float(r[2]); ns=int(r[5])
    except: continue
    recs.append((r[1], fd, ns))
if not recs: print("no data — check run_*/node.log"); sys.exit()
runaway=[(i,fd) for i,fd,ns in recs if fd>=th]
ok=[(i,fd) for i,fd,ns in recs if fd<th]
print(f"adaptive_lio N={len(recs)}")
print(f"  RUNAWAY (final_disp>={th}m): {len(runaway)}/{len(recs)} ({100*len(runaway)/len(recs):.0f}%)  "
      f"{[f'run{i}:{fd:.1f}m' for i,fd in runaway]}")
if ok:
    ds=[fd for _,fd in ok]
    print(f"  BOUNDED: {len(ok)}/{len(recs)}  loop_closure median={st.median(ds):.2f}m "
          f"max={max(ds):.2f}m min={min(ds):.2f}m  vals={[round(d,2) for d in ds]}")
else:
    print("  BOUNDED: 0 — every run ran away")
PY
echo "  saved: $SAVE/summary.csv + run_*/trajectory.csv (inside container: docker cp $CTR:$SAVE .)"
INNER
