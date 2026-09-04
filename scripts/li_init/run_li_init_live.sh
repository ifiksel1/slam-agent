#!/bin/bash
# LIVE LI-Init calibration session with real-time excitation guidance.
#
# Runs LI-Init against the sensor directly so you can watch the three per-axis
# excitation meters fill while you move the vehicle -- instead of recording blind
# and only finding out after a replay that the bag was unusable.
#
# ############################################################################
# #  SAFETY: HANDHELD, MOTORS DISARMED, PROPS OFF.                           #
# #  Static data collection only. Do not arm. Do not fly.                    #
# ############################################################################
#
# WHY THE FLIGHT PIPELINE MUST BE STOPPED FIRST
#   LI-Init runs a full FAST-LIO front end. Run concurrently with the live
#   pipeline, the CPU contention can starve laserMapping, which trips
#   drift_monitor -- configured with enable_auto_restart=true AND
#   enable_source_failover=true, so it would command an EKF source failover on
#   the live FC (MAV_CMD_DO_AUX_FUNCTION 218) and restart nodes.
#
#   Stopping slam-fastlio.service removes that hazard entirely: it also stops
#   mavros, so during the session there is NO ROS->FC bridge running at all.
#
# This script never starts mavros and never commands the FC.
#
# Usage: run_li_init_live.sh [bag_name]
set -uo pipefail

BAG="${1:-calib_live_$(date +%m_%d_%Y_%H_%M_%S).bag}"
CONTAINER="slam-hesai-fastlio"
IMAGE="li-init:latest"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
OUTDIR="$REPO/li_init_result"
SERVICE="slam-fastlio.service"

cin() { docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash 2>/dev/null; $1"; }

echo "=============================================================================="
echo "LI-Init LIVE calibration session"
echo "=============================================================================="

# ---------------------------------------------------------------------------
# 1. Verify DISARMED *before* anything else -- this is the last moment mavros is
#    alive to ask. If the pipeline is already stopped we cannot check, so the
#    operator must confirm.
# ---------------------------------------------------------------------------
if cin 'rosnode list 2>/dev/null | grep -q "^/mavros$"'; then
  armed=$(cin 'timeout 10 rostopic echo -n1 /mavros/state 2>/dev/null | grep "^armed:" | awk "{print \$2}"')
  echo "FC armed state: ${armed:-<unavailable>}"
  if [ "$armed" = "True" ]; then
    echo "REFUSING TO PROCEED: vehicle reports ARMED. Disarm first." >&2
    exit 1
  fi
  [ "$armed" = "False" ] || { echo "Could not read armed state. Aborting." >&2; exit 1; }
  echo "  -> disarmed, OK"
else
  echo "mavros not running (pipeline already stopped) -- cannot verify armed state."
fi

read -r -p "Props OFF and vehicle handheld? Type DISARMED to continue: " ack
[ "$ack" = "DISARMED" ] || { echo "Aborted." >&2; exit 1; }

# ---------------------------------------------------------------------------
# 2. Require the flight pipeline to be stopped. Deliberately NOT done for you:
#    it needs sudo, and stopping a live pipeline is the operator's call.
# ---------------------------------------------------------------------------
if cin 'rosnode list 2>/dev/null | grep -qE "^/(laserMapping|drift_monitor|mavros)$"'; then
  cat <<EOF

The flight pipeline is still running. Stop it first, in another terminal:

    sudo systemctl stop ${SERVICE}

Then re-run this script. Restore it afterwards with:

    sudo systemctl start ${SERVICE}

EOF
  exit 1
fi
echo "  -> flight pipeline stopped (no mavros, no drift_monitor): no FC path exists"

# ---------------------------------------------------------------------------
# 3. Bring up a bare master + the LiDAR driver only. No mapping, no bridge.
# ---------------------------------------------------------------------------
cleanup() {
  echo
  echo "--- tearing down calibration session ---"
  docker exec "$CONTAINER" pkill -f "rosbag record" 2>/dev/null
  docker exec "$CONTAINER" pkill -f hesai_ros_driver_node 2>/dev/null
  docker exec "$CONTAINER" pkill -f roscore 2>/dev/null
  docker exec "$CONTAINER" pkill -f rosmaster 2>/dev/null
  sleep 2
  cat <<EOF

--------------------------------------------------------------------------------
RESTORE THE FLIGHT PIPELINE when you are done:

    sudo systemctl start ${SERVICE}

--------------------------------------------------------------------------------
EOF
}
trap cleanup EXIT INT TERM

echo "--- starting roscore + Hesai driver (driver only) ---"
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash; roscore'
for i in $(seq 1 30); do cin 'rostopic list >/dev/null 2>&1' && break; sleep 1; done
cin 'rostopic list >/dev/null 2>&1' || { echo "roscore failed to start" >&2; exit 1; }

docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash; rosrun hesai_ros_driver hesai_ros_driver_node'
echo "--- waiting for /lidar_points ---"
for i in $(seq 1 40); do
  cin 'timeout 3 rostopic hz /lidar_points 2>/dev/null | grep -q average' && break
  sleep 1
done
cin 'timeout 6 rostopic hz /lidar_points 2>/dev/null | grep -m1 average' \
  || { echo "ERROR: /lidar_points not publishing -- driver failed to start" >&2; exit 1; }

# Archive the session so the calibration can be re-run offline and audited later.
echo "--- recording archive bag: ${BAG} ---"
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; cd /root && rosbag record -O '/root/${BAG}' /lidar_points /lidar_imu"

mkdir -p "$OUTDIR"

cat <<'MOTION'

================================================================================
NOW MOVE THE VEHICLE. Watch the three meters below fill to 100%.
================================================================================
  1. STAY COMPLETELY STILL for the first ~10 s (needed to build the initial map).
  2. Then, smoothly and continuously:
       roll  +/- 30-45 deg      pitch +/- 30-45 deg      yaw +/- 45-90 deg
       translate ~0.5-1 m fore/aft, left/right, up/down
       finish with slow figure-eights combining rotation and translation
  3. Keep motions SMOOTH -- jerks saturate the IMU, too slow is unobservable.

  Whichever axis lags, rotate about THAT axis more. LI-Init writes the result
  once all three are satisfied and the refinement window completes.

  Stand where there is planar structure at several ORIENTATIONS (walls + floor +
  ceiling), a few metres out. Max return on this sensor is ~22 m.

  Ctrl+C to stop early (no result will be written).
================================================================================

MOTION

# --network host so the container reaches the master on localhost:11311. This is
# NOT the offline runner's --network none, but mavros is not running, so there is
# no ROS->FC path regardless. LI-Init only subscribes to lidar + imu.
TTY_FLAGS="-i"
[ -t 1 ] && TTY_FLAGS="-it"

docker run --rm $TTY_FLAGS --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e ROS_HOSTNAME=localhost \
  -v "$REPO/config/li_init_jt128.yaml:/root/li_ws/config/li_init_jt128.yaml:ro" \
  -v "$REPO/config/li_init_jt128_live.launch:/root/li_ws/config/li_init_jt128_live.launch:ro" \
  -v "$OUTDIR:/root/li_ws/src/LiDAR_IMU_Init/result" \
  "$IMAGE" bash -lc '
source /opt/ros/noetic/setup.bash
source /root/li_ws/devel/setup.bash
roslaunch /root/li_ws/config/li_init_jt128_live.launch'

# Stop the recorder and wait for rosbag to rename "<bag>.active" -> "<bag>".
# This MUST happen before the docker cp below: while recording is live the file
# on disk is still "<bag>.active", so a copy of "<bag>" silently finds nothing
# and the archive is left stranded inside the container.
echo
echo "--- closing archive bag ---"
docker exec "$CONTAINER" pkill -INT -f "rosbag record" 2>/dev/null
for i in $(seq 1 60); do
  docker exec "$CONTAINER" test -f "/root/${BAG}" 2>/dev/null && break
  sleep 1
done
docker exec "$CONTAINER" test -f "/root/${BAG}" 2>/dev/null \
  || echo "WARNING: /root/${BAG} did not finalize; check for /root/${BAG}.active" >&2

echo
echo "=================== BUILD VERSIONS ==================="
docker run --rm --network none "$IMAGE" cat /root/li_ws/BUILD_VERSIONS.txt
echo "=================== RESULT ==========================="
if [ -s "$OUTDIR/Initialization_result.txt" ]; then
  cat "$OUTDIR/Initialization_result.txt"
else
  echo "NO CALIBRATION PRODUCED -- excitation was insufficient or the run was"
  echo "interrupted. The archive bag is still saved; you can retry the motion."
fi
echo "======================================================"

# Pull the archive bag out of the container.
docker cp "${CONTAINER}:/root/${BAG}" "${REPO}/${BAG}" 2>/dev/null \
  && docker exec "$CONTAINER" rm -f "/root/${BAG}" \
  && echo "Archive bag: ${REPO}/${BAG}"

cat <<'NEXT'

Compare against the incumbent in config/hesai_jt128.yaml:
    extrinsic_T: [ 0.006622, -0.009142, -0.039155 ]
    extrinsic_R: 3.21 deg total  (rpy -1.16 / -2.31 / +1.92 deg)

  near IDENTITY rotation -> incumbent likely a bad fit; candidate for replacement
  reproduces ~3.21 deg  -> incumbent is real; leave it alone
  large TIME OFFSET     -> timing was being absorbed into the rotation

Do NOT write a new extrinsic on one run. Validate with a separate closed-loop bag
through scripts/replay/run_one.sh, old vs new, comparing position closure.
NEXT
