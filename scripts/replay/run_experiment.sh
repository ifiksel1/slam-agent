#!/bin/bash
# Host orchestrator: replay ONE input bag through FAST-LIO twice (current vs tuned config),
# record each run's outputs to its own bag, then compare. Runs offline & isolated in the
# slam container (ROS master :11312) -- the live pipeline and FC are untouched. NO ARMING.
#
# Usage:  scripts/replay/run_experiment.sh <input_bag_host_path> [output_dir=./replay_out] [rate=1.0]
#
# Input bag MUST contain the FAST-LIO inputs /lidar_points and /lidar_imu.
set -eu
IN_HOST="${1:?usage: run_experiment.sh <input_bag> [out_dir] [rate]}"
OUT_DIR="${2:-./replay_out}"
RATE="${3:-1.0}"
C=slam-hesai-fastlio
WORK=/root/replay
HERE="$(cd "$(dirname "$0")" && pwd)"

[ -f "$IN_HOST" ] || { echo "ERROR: input bag not found: $IN_HOST"; exit 1; }
docker inspect -f '{{.State.Running}}' "$C" 2>/dev/null | grep -q true || { echo "ERROR: container $C not running"; exit 1; }
mkdir -p "$OUT_DIR"

echo "== staging scripts + input bag into container =="
docker exec "$C" mkdir -p "$WORK"
docker cp "$HERE/fastlio_replay.launch" "$C:$WORK/"
docker cp "$HERE/run_one.sh"            "$C:$WORK/"
docker cp "$HERE/compare_bags.py"       "$C:$WORK/"
docker exec "$C" chmod +x "$WORK/run_one.sh"
docker cp "$IN_HOST" "$C:$WORK/input.bag"

echo "== RUN 1/2: current config =="
docker exec "$C" bash "$WORK/run_one.sh" "$WORK/input.bag" "$WORK/out_current.bag" current "$RATE"

echo "== RUN 2/2: tuned config =="
docker exec "$C" bash "$WORK/run_one.sh" "$WORK/input.bag" "$WORK/out_tuned.bag" tuned "$RATE"

echo "== COMPARE =="
docker exec "$C" bash -lc "source /opt/ros/noetic/setup.bash; python3 $WORK/compare_bags.py $WORK/out_current.bag $WORK/out_tuned.bag" | tee "$OUT_DIR/comparison.txt"

echo "== copying results to $OUT_DIR =="
docker cp "$C:$WORK/out_current.bag" "$OUT_DIR/" 2>/dev/null || echo "  (out_current.bag missing)"
docker cp "$C:$WORK/out_tuned.bag"   "$OUT_DIR/" 2>/dev/null || echo "  (out_tuned.bag missing)"
docker cp "$C:$WORK/out_tuned_compare.png" "$OUT_DIR/" 2>/dev/null || true
echo "Done. Results in: $OUT_DIR  (out_current.bag, out_tuned.bag, comparison.txt[, .png])"
