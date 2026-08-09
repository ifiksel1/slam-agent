#!/bin/bash
# Replay a calibration bag through LI-Init and report the estimated LiDAR<->IMU
# extrinsic + time offset.
#
# Runs entirely inside the li-init image, on its OWN ROS master, with no network
# path to the flight controller and no contact with the live pipeline.
#
# Usage: run_li_init.sh <bag> [rate]
set -euo pipefail

BAG="${1:?usage: run_li_init.sh <bag> [rate]}"
RATE="${2:-1.0}"
IMAGE="li-init:latest"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"

[ -f "$BAG" ] || { echo "ERROR: bag not found: $BAG" >&2; exit 1; }
BAG_ABS="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ERROR: image $IMAGE not built. Run:" >&2
  echo "  docker build -t $IMAGE $REPO/docker/li_init/" >&2
  exit 1
fi

OUTDIR="$REPO/li_init_result"
mkdir -p "$OUTDIR"

echo "bag    : $BAG_ABS"
echo "rate   : $RATE"
echo "config : $REPO/config/li_init_jt128.yaml"
echo "output : $OUTDIR/"
echo

# --network none: hard guarantee this cannot reach the FC or the live ROS master.
docker run --rm --network none \
  -v "$BAG_ABS:/data/calib.bag:ro" \
  -v "$REPO/config/li_init_jt128.yaml:/root/li_ws/config/li_init_jt128.yaml:ro" \
  -v "$REPO/config/li_init_jt128.launch:/root/li_ws/config/li_init_jt128.launch:ro" \
  -v "$OUTDIR:/root/li_ws/src/LiDAR_IMU_Init/result" \
  "$IMAGE" bash -lc '
set -e
source /opt/ros/noetic/setup.bash
source /root/li_ws/devel/setup.bash

# REQUIRED with --network none: the container has only loopback and its hostname
# does not resolve, so ROS XMLRPC fails with endless "Connection refused" unless
# both the master URI and the advertised hostname are pinned to localhost.
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

roscore >/tmp/roscore.log 2>&1 &
for i in $(seq 1 30); do
  rostopic list >/dev/null 2>&1 && break
  sleep 1
done
rostopic list >/dev/null 2>&1 || { echo "ERROR: roscore never came up"; cat /tmp/roscore.log; exit 1; }
echo "--- roscore up ---"

roslaunch /root/li_ws/config/li_init_jt128.launch >/tmp/li_init.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 30); do
  rosnode list 2>/dev/null | grep -q laserMapping && break
  sleep 1
done
rosnode list 2>/dev/null | grep -q laserMapping \
  || { echo "ERROR: laserMapping node never started"; tail -40 /tmp/li_init.log; exit 1; }
echo "--- laserMapping up ---"

echo "--- replaying bag ---"
rosbag play --clock -r '"$RATE"' /data/calib.bag

# LI-Init keeps refining after the data ends; give it time to write the result.
echo "--- waiting for refinement to finish ---"
sleep 40
kill $LAUNCH_PID 2>/dev/null || true
pkill -f li_init 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
sleep 3

echo "=================== NODE LOG (tail) =================="
tail -60 /tmp/li_init.log

echo
echo "=================== BUILD VERSIONS ==================="
cat /root/li_ws/BUILD_VERSIONS.txt
echo "=================== EXCITATION ======================="
# LI-Init prints a live per-axis progress meter. The FINAL reading tells you whether
# the bag ever made the extrinsic observable -- this is the first thing to check when
# no result is produced.
# The meter is colourised and redrawn with cursor-control escapes, so strip ANSI
# before matching: the codes sit BETWEEN the colon and the percentage.
EXC=$(sed -e "s/\x1b\[[0-9;]*[a-zA-Z]//g" /tmp/li_init.log \
      | grep -ao "Rotation around Lidar [XYZ] Axis: *[0-9]*%" | tail -3)
if [ -n "$EXC" ]; then
  echo "$EXC"
else
  echo "(no excitation meter found -- did the node process any data?)"
fi

echo "=================== RESULT ==========================="
RES=/root/li_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
if [ -s "$RES" ]; then
  cat "$RES"
else
  echo "NO CALIBRATION PRODUCED."
  echo
  echo "The result file is $( [ -f "$RES" ] && echo empty || echo missing ). If the excitation"
  echo "percentages above are not at/near 100% on ALL THREE axes, the BAG is the"
  echo "problem, not the extrinsic -- re-record with more rotation. See"
  echo "docs/LI_INIT_CALIBRATION.md for the motion profile."
fi
echo "======================================================"
' 2>&1 | tee "$OUTDIR/run.log"

echo
echo "Result + log in $OUTDIR/"
echo
cat <<'NEXT'
--------------------------------------------------------------------------------
Interpreting the result

LI-Init reports the extrinsic AND the LiDAR/IMU time offset. Compare against the
incumbent values in config/hesai_jt128.yaml:
    extrinsic_T: [ 0.006622, -0.009142, -0.039155 ]
    extrinsic_R: 3.21 deg total  (rpy -1.16 / -2.31 / +1.92 deg)

  * Converges near IDENTITY rotation -> the incumbent 3.21 deg was likely a bad
    fit, plausibly absorbing the old timing error. Consider replacing it.
  * Reproduces ~3.21 deg -> the incumbent is real. Leave it alone.
  * Reports a large TIME OFFSET -> that is the finding; a timing error was being
    absorbed into the rotation. Fix the timing first, then re-run.
  * Refuses to converge / reports insufficient excitation -> the BAG is the
    problem, not the extrinsic. Re-record with more rotation on all three axes.

Do NOT write a new extrinsic into the live config on one run. Validate it first
by replaying a separate closed-loop bag through scripts/replay/run_one.sh with
old vs new values and comparing end-to-end position closure.
--------------------------------------------------------------------------------
NEXT
