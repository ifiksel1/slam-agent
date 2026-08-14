#!/bin/bash
# Run ONE offline FAST-LIO replay of an input bag and record outputs to an output bag.
# Isolated on ROS master :11312 so it NEVER touches the live pipeline (:11311) or the FC.
# Teardown kills ONLY the PIDs this script started (never pkill by binary name -> would hit live laserMapping).
#
# Usage:  run_one.sh <input_bag> <output_bag> <profile: current|tuned> [rate=1.0]
set -u
IN="$1"; OUT="$2"; PROFILE="${3:-current}"; RATE="${4:-1.0}"
PORT=11312
WORK="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI="http://localhost:${PORT}"
export ROS_HOSTNAME=localhost

# ---- tuning profiles (edit here to change the experiment) ----
if [ "$PROFILE" = "tuned" ]; then
  # Tier 1 (resolution/near-field) + higher max_iteration. Tier 2 EXCLUDED per user:
  # det_range and cube_side_length kept at CURRENT values (100 / 300).
  ARGS="blind:=0.4 det_range:=100 filter_size_surf:=0.25 filter_size_map:=0.25 point_filter_num:=1 cube_side_length:=300 max_iteration:=4"
else
  ARGS="blind:=1.0 det_range:=100 filter_size_surf:=0.5 filter_size_map:=0.5 point_filter_num:=3 cube_side_length:=300 max_iteration:=3"
fi
echo "[run_one] profile=$PROFILE rate=$RATE"
echo "[run_one] params: $ARGS"

[ -f "$IN" ] || { echo "[run_one] ERROR: input bag not found: $IN"; exit 1; }

cleanup() {
  echo "[run_one] teardown..."
  rosnode kill /replay_rec >/dev/null 2>&1 || true   # clean-close the recorded bag
  sleep 2
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true   # SIGINT -> roslaunch shuts its nodes
  sleep 3
  [ -n "${ROSCORE_PID:-}" ] && kill -INT "$ROSCORE_PID" >/dev/null 2>&1 || true
  sleep 1
}
trap cleanup EXIT

# fail fast if something already holds the isolated port (don't clobber it)
if curl -s "http://localhost:${PORT}" >/dev/null 2>&1; then
  echo "[run_one] ERROR: something is already on port ${PORT}. Clean it up first."; exit 1
fi

# isolated master
roscore -p $PORT >/tmp/replay_roscore_${PROFILE}.log 2>&1 &
ROSCORE_PID=$!
sleep 4
rosparam set /use_sim_time true

# FAST-LIO (laserMapping only)
roslaunch "$WORK/fastlio_replay.launch" $ARGS >/tmp/replay_laserMapping_${PROFILE}.log 2>&1 &
LAUNCH_PID=$!
echo "[run_one] waiting for laserMapping to register..."
for i in $(seq 1 20); do
  rosnode list 2>/dev/null | grep -q /laserMapping && break; sleep 1
done

# record outputs (named node so we can clean-close it)
rosbag record -O "$OUT" __name:=replay_rec \
  /Odometry /fastlio_health /path /tf /tf_static >/tmp/replay_rec_${PROFILE}.log 2>&1 &
sleep 2

# play the input bag on sim clock (blocks until done)
echo "[run_one] playing bag..."
rosbag play --clock --rate "$RATE" "$IN"
echo "[run_one] play finished; draining..."
sleep 3   # let laserMapping finish the last scans

cleanup
trap - EXIT
# rosbag record may leave a .active if not fully flushed -> reindex if needed
[ -f "${OUT}.active" ] && rosbag reindex "${OUT}.active" >/dev/null 2>&1 && mv "${OUT}.active" "$OUT" 2>/dev/null || true
echo "[run_one] DONE profile=$PROFILE -> $OUT"
