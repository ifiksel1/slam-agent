#!/bin/bash
# ONE offline FAST-LIO replay with EXPLICIT param args (for the confined-space sweep).
# Isolated on ROS master :11312 -> never touches the live pipeline (:11311) or the FC.
# Records ONLY /fastlio_health + /Odometry (small) for metric extraction.
#
# Usage: sweep_run_one.sh <input_bag> <output_bag> "<roslaunch args>" [rate=1.0]
#   e.g. sweep_run_one.sh in.bag out.bag "filter_size_surf:=0.25 filter_size_map:=0.25 point_filter_num:=2 max_iteration:=4 blind:=0.4 det_range:=100 cube_side_length:=300"
set -u
IN="$1"; OUT="$2"; ARGS="$3"; RATE="${4:-1.0}"
PORT=11312
WORK="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI="http://localhost:${PORT}"
export ROS_HOSTNAME=localhost

echo "[sweep_run] args: $ARGS"
[ -f "$IN" ] || { echo "[sweep_run] ERROR: input bag not found: $IN"; exit 1; }

cleanup() {
  rosnode kill /replay_rec >/dev/null 2>&1 || true
  sleep 2
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  sleep 3
  [ -n "${ROSCORE_PID:-}" ] && kill -INT "$ROSCORE_PID" >/dev/null 2>&1 || true
  sleep 1
}
trap cleanup EXIT

if curl -s "http://localhost:${PORT}" >/dev/null 2>&1; then
  echo "[sweep_run] ERROR: port ${PORT} busy; clean up first."; exit 1
fi

roscore -p $PORT >/tmp/sweep_roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 4
rosparam set /use_sim_time true

roslaunch "$WORK/fastlio_replay.launch" $ARGS >/tmp/sweep_laserMapping.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 25); do rosnode list 2>/dev/null | grep -q /laserMapping && break; sleep 1; done

rosbag record -O "$OUT" __name:=replay_rec /Odometry /fastlio_health >/tmp/sweep_rec.log 2>&1 &
sleep 2

echo "[sweep_run] playing $IN ..."
rosbag play --clock --rate "$RATE" "$IN" >/dev/null 2>&1
sleep 3

cleanup
trap - EXIT
[ -f "${OUT}.active" ] && rosbag reindex "${OUT}.active" >/dev/null 2>&1 && mv "${OUT}.active" "$OUT" 2>/dev/null || true
echo "[sweep_run] DONE -> $OUT"
