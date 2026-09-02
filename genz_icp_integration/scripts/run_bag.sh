#!/bin/bash
# OFFLINE: replay one hangar bag through GenZ-ICP (ROS1 Noetic, arm64) and record
# trajectory + per-scan alpha to CSV.
#
# GenZ-ICP is LiDAR-ONLY: it consumes /ouster/points and nothing else. No IMU, no
# extrinsics, no time-sync. /mavros/local_position/pose in these bags is the onboard
# EKF and is recorded separately as a cross-check reference, NOT fed to the estimator.
#
# Clean slate per run (docker rm -f) follows the project rule: never reuse a container
# across comparison runs.
#
# Usage: run_bag.sh <bag.bag> [config] [tag] [rate] [play_seconds]
#   play_seconds: "" = whole bag; a number = rosbag play -u <n> (smoke tests)
set -uo pipefail
BAG_FILE="${1:?usage: run_bag.sh <bag.bag> [config] [tag] [rate] [play_seconds]}"
CFG="${2:-hangar.yaml}"
TAG="${3:-}"
RATE="${4:-1.0}"
PLAY_SEC="${5:-}"
VIZ="${6:-true}"   # false = no debug clouds (faster, but NO alpha telemetry)
MCR="${MAP_CLEANUP_RADIUS:-300.0}"  # 100 = upstream default (indoor); 300 for the hangar

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G="$(dirname "$HERE")"
CTR=genz_icp
IMAGE="${GENZ_IMAGE:-genz_icp:noetic}"
BAGS_HOST="${BAGS_HOST:-$HOME/all_bag_files}"
TOPIC="${TOPIC:-/ouster/points}"

BASE="$(basename "$BAG_FILE" .bag)"
HOSTOUT="$G/results/${BASE}${TAG}"
[ -f "$BAGS_HOST/$BAG_FILE" ] || { echo "no bag at $BAGS_HOST/$BAG_FILE"; exit 1; }
mkdir -p "$HOSTOUT"

# Bag duration bounds the sampler so it can never orphan.
DUR=$(docker run --rm -v "$BAGS_HOST:/bags:ro" "$IMAGE" \
        bash -lc "rosbag info -y -k duration /bags/$BAG_FILE" 2>/dev/null | tr -d '\r')
[ -z "$DUR" ] && DUR=600
if [ -n "$PLAY_SEC" ]; then WINDOW="$PLAY_SEC"; else WINDOW="$DUR"; fi
# sampler lifetime = play window / rate + startup + drain margin
SAMPLE_SEC=$(awk -v w="$WINDOW" -v r="$RATE" 'BEGIN{printf "%.0f", w/r + 45}')

echo "=== bag=$BAG_FILE dur=${DUR}s rate=$RATE window=${WINDOW}s cfg=$CFG ==="

# clock advisory (project rule: boost for accuracy runs)
F=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null || echo 0)
[ "$F" = "1984000" ] && echo "clock: boosted (good)" \
  || echo "clock: $F -- recommend 'sudo ~/superodom_ws/set_cpu_clock.sh boost'"

echo "=== (re)starting $CTR (clean slate) ==="
docker rm -f "$CTR" >/dev/null 2>&1 || true
# NO --net=host. The host already runs a rosmaster on 11311 (the foxglove_bridge
# stack, ROS_IP=192.168.2.50). Sharing the host netns made our roscore fail to bind
# and our nodes silently register with THAT master, which then could not route
# between them -> "XmlRpcClient::writeRequest: Connection refused" and zero odometry.
# Everything we need (roscore, estimator, sampler, bag player) lives inside this one
# container, so an isolated network namespace is both sufficient and safer: it cannot
# perturb the user's running stack.
docker run -d --name "$CTR" --init --shm-size=4gb \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
  -v "$BAGS_HOST:/bags:ro" \
  -v "$G/config:/root/genz_cfg:ro" \
  -v "$G/launch:/root/genz_launch:ro" \
  -v "$G/scripts:/root/genz_scripts:ro" \
  -v "$HOSTOUT:/root/out" \
  "$IMAGE" sleep infinity >/dev/null

SRC='source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash'

docker exec -d "$CTR" bash -lc "$SRC && roscore > /root/out/roscore.log 2>&1"
for i in $(seq 30); do
  docker exec "$CTR" bash -lc "$SRC && rostopic list" >/dev/null 2>&1 && break
  sleep 1
done

echo "=== launching GenZ-ICP (topic=$TOPIC) ==="
docker exec -d "$CTR" bash -lc "$SRC && \
  roslaunch /root/genz_launch/genz_hangar.launch topic:=$TOPIC config_file:=$CFG visualize:=$VIZ map_cleanup_radius:=$MCR \
    > /root/out/genz.log 2>&1"
sleep 6

docker exec -d "$CTR" bash -lc "$SRC && \
  python3 /root/genz_scripts/genz_traj_sampler.py /root/out/trajectory.csv $SAMPLE_SEC \
    > /root/out/sampler.log 2>&1"
sleep 2

# Reference: onboard EKF pose, recorded straight from the bag (not an estimator input).
docker exec -d "$CTR" bash -lc "$SRC && \
  rostopic echo -p /mavros/local_position/pose > /root/out/mavros_ref.csv 2>/dev/null"

PLAY_ARGS="--clock -q -r $RATE"
[ -n "$PLAY_SEC" ] && PLAY_ARGS="$PLAY_ARGS -u $PLAY_SEC"
# Liveness probe: confirm the estimator is actually consuming and publishing,
# instead of discovering an empty CSV at the end of a long replay.
(
  sleep 25
  echo "--- probe: topics/rates 25s in ---"
  docker exec "$CTR" bash -lc "$SRC && rostopic hz -w 10 /genz/odometry" 2>&1 | head -4
  docker exec "$CTR" bash -lc "$SRC && rostopic hz -w 10 /genz/planar_points" 2>&1 | head -4
  echo "--- end probe ---"
) &
PROBE_PID=$!

echo "=== playing bag ($PLAY_ARGS) ==="
docker exec "$CTR" bash -lc "$SRC && rosbag play $PLAY_ARGS /bags/$BAG_FILE \
  > /root/out/play.log 2>&1"

kill $PROBE_PID 2>/dev/null; wait $PROBE_PID 2>/dev/null
echo "=== draining ==="
sleep 20
docker exec "$CTR" bash -lc "pkill -INT -f genz_traj_sampler" >/dev/null 2>&1 || true
sleep 8
docker exec "$CTR" bash -lc "pkill -INT -f 'rostopic echo'" >/dev/null 2>&1 || true
sleep 2
docker rm -f "$CTR" >/dev/null 2>&1 || true

echo "=== results -> $HOSTOUT ==="
if [ -s "$HOSTOUT/trajectory.csv" ]; then
  python3 "$HERE/genz_report.py" "$HOSTOUT/trajectory.csv"
else
  echo "NO TRAJECTORY RECORDED -- inspect $HOSTOUT/genz.log"
  tail -20 "$HOSTOUT/genz.log" 2>/dev/null
fi
