#!/bin/bash
# Record an excitation bag for LI-Init LiDAR<->IMU calibration.
#
# READ-ONLY: records /lidar_points + /lidar_imu from the live master. Sends nothing
# to the flight controller.
#
# ############################################################################
# #  SAFETY: HANDHELD, MOTORS DISARMED, PROPS OFF.                           #
# #  This is a static data recording, NOT a flight test. Do not arm.         #
# ############################################################################
#
# Usage: record_calib_bag.sh [output.bag] [duration_seconds]
set -euo pipefail

OUT="${1:-calib_$(date +%m_%d_%Y_%H_%M_%S).bag}"
DURATION="${2:-120}"
CONTAINER="slam-hesai-fastlio"

cat <<'BANNER'
================================================================================
LI-Init excitation bag
================================================================================
Motors DISARMED, props OFF, vehicle HANDHELD. Do not arm. Do not fly.

LI-Init needs the extrinsic to be OBSERVABLE. That requires excitation on all
three rotation axes plus translation -- a slow walk does NOT determine it, and a
degenerate bag yields a confidently WRONG extrinsic. That is the most likely way
the current 3.21 deg value came to exist, so do not repeat it.

Procedure:
  1. STAY COMPLETELY STILL for the first ~10 s (upstream requires >5 s to
     accumulate the initial map). Do not skip this.
  2. Then, smoothly and continuously for the rest of the recording:
       - rotate about ROLL   (+/- 30-45 deg, back and forth)
       - rotate about PITCH  (+/- 30-45 deg)
       - rotate about YAW    (+/- 45-90 deg)
       - translate fore/aft, left/right, and up/down (~0.5-1 m each)
       - finish with slow figure-eights combining rotation and translation
  3. Keep motions SMOOTH. Jerks saturate the IMU; too slow is unobservable.

Environment matters as much as motion. Stand somewhere with planar structure at
several ORIENTATIONS (walls + floor + ceiling, furniture), a few metres away.
This sensor is short range (median return 2.4 m, max ~22 m) -- an empty corridor
or a wide-open space will not constrain the solution.
================================================================================
BANNER

read -r -p "Motors disarmed and props off? Type DISARMED to continue: " ack
if [ "$ack" != "DISARMED" ]; then
  echo "Aborted." >&2
  exit 1
fi

# Confirm with the FC directly rather than trusting the typed answer.
echo "Verifying armed state from /mavros/state ..."
armed=$(docker exec "$CONTAINER" bash -lc \
  'source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash 2>/dev/null;
   timeout 10 rostopic echo -n1 /mavros/state 2>/dev/null | grep "^armed:" | awk "{print \$2}"' || true)
echo "  /mavros/state armed = ${armed:-<unavailable>}"
if [ "$armed" = "True" ]; then
  echo "REFUSING TO RECORD: the vehicle reports ARMED. Disarm first." >&2
  exit 1
fi

echo
echo "Recording ${DURATION}s to ${OUT} ..."
echo "Stay still for the first 10 s, THEN begin excitation."
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash 2>/dev/null;
   cd /root && rosbag record --duration=${DURATION} -O '/root/${OUT}' /lidar_points /lidar_imu"

docker cp "${CONTAINER}:/root/${OUT}" "./${OUT}"
docker exec "$CONTAINER" rm -f "/root/${OUT}"

echo
echo "Wrote ./${OUT}"
docker run --rm -v "$(pwd):/d" -w /d ros:noetic-ros-core \
  bash -lc "source /opt/ros/noetic/setup.bash && rosbag info '${OUT}'" 2>/dev/null || true
echo
echo "Next: scripts/li_init/run_li_init.sh ./${OUT}"
