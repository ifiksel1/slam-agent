#!/bin/bash
# DISTRIBUTIONAL run of UPSTREAM COIN-LIO (ROS1 Noetic, ~/coinlio_ws) on a field bag, reporting
# the SAME frame-invariant trajectory metrics + runaway RATE as the EllipseLIO geo-vs-photo battery
# (iris_lio_geo_vs_photo.sh) so the numbers are directly comparable. This answers: does COIN-LIO's
# OWN integrated photometric+geometric estimator (where IRIS-LIO's photometric residual came from)
# handle the degenerate 3rd_floor scene better than EllipseLIO-base / IRIS-LIO does?
#
# COIN-LIO is ROS1 and the field bags are ROS2 sqlite3 -> use the pre-converted ROS1 bag
# (rosbags-convert). Converted bag default: ~/coinlio_ws/bags/<name>.bag
#
# Boost the clock first:  sudo ~/superodom_ws/set_cpu_clock.sh boost
# Usage: run_coinlio_bag.sh [ros1_bag_path] [N=6] [runaway_thresh_m=3.0]
set -uo pipefail
BAG="${1:-$HOME/coinlio_ws/bags/3rd_floor_20260613_222459.bag}"
N="${2:-6}"
RUNAWAY="${3:-3.0}"
RATE="${4:-1.0}"   # bag replay rate; lower (e.g. 0.5) if COIN-LIO is compute-bound on the Jetson
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/coinlio_ws"
SAMPLER="$HERE/coinlio_traj_sampler.py"
TAG="$(basename "$BAG" .bag)"
SAVE="$WS/results/coinlio/${TAG}"
[ -f "$BAG" ] || { echo "no ROS1 bag at $BAG (convert first: rosbags-convert <ros2_dir> --dst $BAG)"; exit 1; }
mkdir -p "$SAVE"; : > "$SAVE/summary.csv"

# ROS setup.bash is not `set -u` safe (references unbound vars) — relax nounset around sourcing
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u

F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" || echo "clock: $F — recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

# persistent roscore for the whole battery
if ! rostopic list >/dev/null 2>&1; then
  echo "=== starting roscore ==="; roscore >/tmp/coinlio_roscore.log 2>&1 &
  for i in $(seq 1 20); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
fi
rosparam set /use_sim_time true

cleanup_nodes() {  # reap node + sampler + player; WAIT for full death (safe: patterns not in this script's cmdline)
  # SIGINT roslaunch FIRST so it gracefully deregisters /laserMapping from the master. If we just
  # kill the node binary, a lingering roslaunch shutdown can deregister the NEXT run's same-named
  # node (the run2/run3 samples=0 race). Then poll until the processes are truly gone.
  pkill -INT -f 'coinlio_traj_sampler.py' 2>/dev/null || true
  pkill -INT -f 'mapping_ouster.launch'   2>/dev/null || true
  pkill -INT -f 'rosbag play'             2>/dev/null || true
  for w in $(seq 1 40); do
    pgrep -f 'mapping_ouster.launch' >/dev/null 2>&1 || pgrep -f 'coin_lio_mapping' >/dev/null 2>&1 || break
    sleep 0.5
  done
  # escalate anything still alive, then purge stale master registrations so the next node is clean
  pkill -9 -f 'coin_lio_mapping'        2>/dev/null || true
  pkill -9 -f 'mapping_ouster.launch'   2>/dev/null || true
  pkill -9 -f 'rosbag play'             2>/dev/null || true
  pkill -9 -f 'coinlio_traj_sampler.py' 2>/dev/null || true
  sleep 1
  yes | timeout 8 rosnode cleanup >/dev/null 2>&1 || true
}

run_one() {  # $1=index
  local i=$1
  local OUTDIR="$SAVE/run_$i"; mkdir -p "$OUTDIR"
  echo "########## COIN-LIO run $i/$N ##########"
  cleanup_nodes
  # 1. launch COIN-LIO node ONLY (bag_file empty -> no embedded player); we drive the bag ourselves
  roslaunch coin_lio mapping_ouster.launch rviz:=false > "$OUTDIR/node.log" 2>&1 &
  sleep 8
  # 2. trajectory sampler on /Odometry (bounded so it can never orphan)
  python3 "$SAMPLER" "$OUTDIR/trajectory.csv" 1200 /Odometry > "$OUTDIR/sampler.log" 2>&1 &
  local SPID=$!
  sleep 2
  # 3. replay the converted ROS1 bag (--clock drives sim time) at rate 1.0
  rosbag play --clock --quiet --rate "$RATE" "$BAG" > "$OUTDIR/play.log" 2>&1 &
  local PLAYPID=$!   # NOT 'PPID' — that is a bash readonly special var (parent PID)
  sleep 5
  while kill -0 "$PLAYPID" 2>/dev/null; do sleep 3; done   # wait for replay to finish
  sleep 4                                                # settle last scans
  kill -INT "$SPID" 2>/dev/null || true
  for w in $(seq 1 20); do grep -q 'COINLIO TRAJ SUMMARY' "$OUTDIR/sampler.log" 2>/dev/null && break; sleep 1; done
  cleanup_nodes
  local SUM=$(grep -h 'COINLIO TRAJ SUMMARY' "$OUTDIR/sampler.log" 2>/dev/null | tail -1)
  local FD=$(echo "$SUM" | grep -oE 'final_disp_from_start=[0-9.]+' | grep -oE '[0-9.]+$')
  local PK=$(echo "$SUM" | grep -oE 'peak_excursion=[0-9.]+'       | grep -oE '[0-9.]+$')
  local MS=$(echo "$SUM" | grep -oE 'max_single_step=[0-9.]+'      | grep -oE '[0-9.]+$')
  local NS=$(echo "$SUM" | grep -oE 'samples=[0-9]+'              | grep -oE '[0-9]+$')
  echo ">>> coinlio run$i: final_disp=${FD:-?}m peak=${PK:-?}m max_step=${MS:-?}m samples=${NS:-?}"
  echo "coinlio,$i,${FD:-NA},${PK:-NA},${MS:-NA},${NS:-NA}" >> "$SAVE/summary.csv"
}

# reap our children on TERM/INT (e.g. TaskStop) so we never orphan a node/roslaunch that would
# pollute the master and kill the NEXT battery's nodes ("killing on exit" race)
trap 'cleanup_nodes 2>/dev/null; exit 130' INT TERM

for i in $(seq 1 "$N"); do run_one "$i"; done

echo; echo "===== UPSTREAM COIN-LIO DISTRIBUTION ($TAG, N=$N) ====="
# COIN-LIO's failure mode on this bag is a STALL (node stops emitting odometry mid-bag), NOT a
# runaway like EllipseLIO. A full run has ~7900 samples (7919 clouds @ 20Hz); a stalled run has
# far fewer (it froze partway). So classify by sample count, then report loop-closure drift for
# the runs that actually COMPLETED.
python3 - "$SAVE/summary.csv" "$RUNAWAY" <<'PY'
import sys, statistics as st
rows=[l.strip().split(',') for l in open(sys.argv[1]) if l.strip()]
th=float(sys.argv[2])
COMPLETE=7000   # samples >= this => ran the full bag; below => stalled partway
recs=[]
for r in rows:
    if len(r)<6: continue
    try: fd=float(r[2]); ns=int(r[5])
    except: continue
    recs.append((r[1], fd, ns))
if not recs: print("no data — check run_*/node.log"); sys.exit()
done=[(i,fd) for i,fd,ns in recs if ns>=COMPLETE]
stalled=[(i,fd,ns) for i,fd,ns in recs if ns<COMPLETE]
print(f"coinlio N={len(recs)}")
print(f"  STALLED (incomplete): {len(stalled)}/{len(recs)} ({100*len(stalled)/len(recs):.0f}%)  "
      f"{[f'run{i}:{ns}smp@{fd:.0f}m' for i,fd,ns in stalled]}")
if done:
    ds=[fd for _,fd in done]
    print(f"  COMPLETED: {len(done)}/{len(recs)}  loop_closure median={st.median(ds):.2f}m "
          f"max={max(ds):.2f}m min={min(ds):.2f}m  vals={[round(d,2) for d in ds]}")
else:
    print("  COMPLETED: 0 — every run stalled")
PY
echo "  saved: $SAVE/summary.csv + run_*/trajectory.csv"
