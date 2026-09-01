#!/bin/bash
# Record a RAW pitch-excitation bag (/lidar_points + /lidar_imu) for pitch->lateral
# drift analysis, with NO ROS->FC path in existence for the duration.
#
# ############################################################################
# #  SAFETY: HANDHELD, MOTORS DISARMED, PROPS OFF.                           #
# #  Static data collection only. Do not arm. Do not fly.                    #
# ############################################################################
#
# WHY THIS EXISTS instead of record_calib_bag.sh:
#   record_calib_bag.sh only runs `rosbag record` -- it assumes a master and the
#   Hesai driver are already up, which normally comes from slam-fastlio.service.
#   But that service also starts mavros and drift_monitor, and drift_monitor is
#   configured with enable_auto_restart=true AND enable_source_failover=true, so
#   with it running a CPU stall can command an EKF source failover on the live FC
#   (MAV_CMD_DO_AUX_FUNCTION 218). Hand-pitching the vehicle with that path alive
#   is exactly the hazard run_li_init_live.sh is written to avoid.
#
#   So this script brings up a bare roscore + the LiDAR driver ONLY. No mapping,
#   no bridge, no mavros. It never starts mavros and never commands the FC.
#
# The bag it produces feeds:
#   scripts/replay/run_one.sh            -> replay offline to get /Odometry + /fastlio_health
#   scripts/replay/analyze_pitch_drift.py -> diagnose the lateral drift
#
# Usage: record_pitch_bag.sh [duration_s] [bag_name]
set -uo pipefail

DURATION="${1:-150}"
BAG="${2:-pitch_$(date +%m_%d_%Y_%H_%M_%S).bag}"
CONTAINER="slam-hesai-fastlio"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SERVICE="slam-fastlio.service"

cin() { docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash 2>/dev/null; $1"; }

echo "=============================================================================="
echo "Pitch-excitation bag  (${DURATION}s -> ${BAG})"
echo "=============================================================================="

# --- 1. Disarm check, while mavros may still be alive to answer ---------------
if cin 'rosnode list 2>/dev/null | grep -q "^/mavros$"'; then
  armed=$(cin 'timeout 10 rostopic echo -n1 /mavros/state 2>/dev/null | grep "^armed:" | awk "{print \$2}"')
  echo "FC armed state: ${armed:-<unavailable>}"
  if [ "$armed" = "True" ]; then
    echo "REFUSING TO PROCEED: vehicle reports ARMED. Disarm first." >&2
    exit 1
  fi
else
  echo "mavros not running (pipeline stopped) -- cannot verify armed state."
fi

read -r -p "Props OFF and vehicle handheld? Type DISARMED to continue: " ack
[ "$ack" = "DISARMED" ] || { echo "Aborted." >&2; exit 1; }

# --- 2. Require the flight pipeline stopped (operator's call, needs sudo) -----
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

# --- 3. Bare master + LiDAR driver only --------------------------------------
cleanup() {
  echo
  echo "--- tearing down recording session ---"
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

# Confirm the IMU is live too -- a bag without /lidar_imu is useless to the analyzer.
cin 'timeout 6 rostopic hz /lidar_imu 2>/dev/null | grep -m1 average' \
  || { echo "ERROR: /lidar_imu not publishing -- analyzer needs it" >&2; exit 1; }

cat <<'MOTION'

================================================================================
PITCH PROTOCOL -- this is not the LI-Init motion. Isolation matters more here.
================================================================================
  1. STAY COMPLETELY STILL ~15 s. This sets the gravity reference that defines
     the "lateral" axis. Rest the vehicle on something solid if you can.
  2. Then 5-6 ISOLATED PITCH CYCLES, +/- 30-40 deg, slow and smooth,
     with a 2-3 s PAUSE between each. The pauses are the measurement: they show
     whether drift accumulates per cycle or recovers.
  3. Actively suppress roll and yaw. Whatever leaks in is signal, not a problem
     -- the analyzer regresses it out -- but the less, the sharper the result.
  4. RETURN TO THE EXACT START POSE and hold still ~10 s. Without this the
     closure number is meaningless.

  Stand where structure is visible at several ORIENTATIONS. If you pitch down
  into bare floor, degeneracy is guaranteed and will mask everything else.
================================================================================

MOTION

read -r -p "Ready? Press Enter to start the ${DURATION}s recording... " _

echo "--- recording ${DURATION}s -> ${BAG} ---"
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; cd /root && rosbag record --duration=${DURATION} -O '/root/${BAG}' /lidar_points /lidar_imu"

# rosbag renames "<bag>.active" -> "<bag>" only once it has flushed. Copying before
# that silently finds nothing and strands the bag inside the container.
echo "--- closing bag ---"
for i in $(seq 1 60); do
  docker exec "$CONTAINER" test -f "/root/${BAG}" 2>/dev/null && break
  sleep 1
done
docker exec "$CONTAINER" test -f "/root/${BAG}" 2>/dev/null \
  || { echo "WARNING: /root/${BAG} did not finalize; check for /root/${BAG}.active" >&2; exit 1; }

# Archive a host-side copy, but KEEP the container-side bag: scripts/replay/ is not
# mounted into the container and the host has no `rosbag` module, so both the replay
# and the analysis have to run inside the container against /root/${BAG}. Copying a
# multi-GB bag back in later would be pure waste.
docker cp "${CONTAINER}:/root/${BAG}" "${REPO}/${BAG}" \
  && echo "Bag (host archive): ${REPO}/${BAG}"
echo "Bag (container, kept for replay): /root/${BAG}"

cat <<NEXT

Next, with the flight pipeline still STOPPED (the replay runs a full FAST-LIO and
is CPU-heavy; starting the service now would put drift_monitor back in the loop):

  1. Copy the replay/analysis scripts into the container (not a mounted path):
       docker cp scripts/replay ${CONTAINER}:/root/replay

  2. Replay to generate /Odometry + /fastlio_health (isolated master :11312):
       docker exec -it ${CONTAINER} bash -lc \\
         "/root/replay/run_one.sh /root/${BAG} /root/${BAG%.bag}_replay.bag current"

  3. Analyze (must run in-container: the host has no rosbag module):
       docker exec -it ${CONTAINER} bash -lc \\
         "source /opt/ros/noetic/setup.bash; python3 /root/replay/analyze_pitch_drift.py \\
            /root/${BAG} /root/${BAG%.bag}_replay.bag --json"

     NOTE: --png needs matplotlib, which is NOT installed in this container.
     Text + --json carry the full result; add plots with
       docker exec ${CONTAINER} pip3 install matplotlib
     if you want them.

  4. Free the container-side bags when done:
       docker exec ${CONTAINER} rm -f /root/${BAG} /root/${BAG%.bag}_replay.bag

NEXT
