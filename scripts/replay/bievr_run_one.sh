#!/bin/bash
# Run ONE offline BIEVR-LIO replay of a flight bag (bievr_lio_ros/process_bag), inside the
# slam-system:bievr image. Run it in a throwaway container with no network and no devices, so it
# cannot reach the live ROS master (:11311) or the FC:
#
#   docker run --rm --network none -v /mnt/thumbdrive:/mnt/usb:ro \
#     -v $PWD/config:/cfg:ro -v $PWD/scripts/replay:/replay:ro -v <outdir>:/out \
#     --entrypoint bash slam-system:bievr /replay/bievr_run_one.sh <bag> /out/<name> [imu_offset_s]
#
# NEVER run on the NUC while the live stack is up: CPU contention trips drift_monitor.
#
# process_bag reads the bag directly as fast as it can (no rosbag play, no sim clock), so the run
# is deterministic and the per-scan "step" timing is pure compute on this machine.
# Outputs: <prefix>.tum (t x y z qx qy qz qw, every scan, lossless), <prefix>_odom.bag
# (/bievr_lio/odom + biases, best effort: the publisher queue is 10 and process_bag runs faster
# than real time, so messages can drop), <prefix>.log (stdout incl. per-scan timings).
#
# imu_offset_s (optional, default 0): shift every /lidar_imu stamp by this many seconds first, to
# test the lidar/IMU time offset BIEVR has no parameter for. FAST-LIO's equivalent is
# stamp - time_offset_lidar_to_imu = stamp + 0.014996.
set -u
IN="$1"; PREFIX="$2"; IMU_OFFSET="${3:-0}"
PORT=11312

export ROS_MASTER_URI="http://localhost:${PORT}"
export ROS_HOSTNAME=localhost
set +u   # ROS setup scripts reference unset variables
source /opt/ros/noetic/setup.bash
source /root/slam_ws/devel/setup.bash
set -u

for f in ${IN//,/ }; do [ -f "$f" ] || { echo "[bievr_run_one] input bag not found: $f"; exit 1; }; done

# <bag> may be a comma-separated list of split parts of one flight; they are merged first.
BAG="$IN"
if [ "$IMU_OFFSET" != "0" ] || [[ "$IN" == *,* ]]; then
  BAG="${PREFIX}_input.bag"
  echo "[bievr_run_one] merging / shifting /lidar_imu by ${IMU_OFFSET} s -> $BAG"
  python3 /replay/bievr_shift_imu.py "$IN" "$BAG" "$IMU_OFFSET" || exit 1
fi

# Replay params = the deployed bievr_params.yaml with the TUM log and per-scan timing switched on.
PARAMS="${PREFIX}_params.yaml"
sed -e "s|^\(\s*trajectory_path:\).*|\1 \"${PREFIX}.tum\"|" \
    -e "s|^\(\s*timing:\).*|\1 True|" /cfg/bievr_params.yaml > "$PARAMS"

roscore -p $PORT >"${PREFIX}_roscore.log" 2>&1 &
ROSCORE_PID=$!
sleep 4

rosbag record -O "${PREFIX}_odom.bag" --buffsize=1024 __name:=bievr_rec \
  /bievr_lio/odom /bievr_lio/bias/acc /bievr_lio/bias/gyro >"${PREFIX}_rec.log" 2>&1 &
sleep 2

echo "[bievr_run_one] processing $BAG"
SECONDS=0
rosrun bievr_lio_ros process_bag --sensor_config_file /cfg/bievr_jt128.yaml \
  --params_file "$PARAMS" --bag "$BAG" >"${PREFIX}.log" 2>&1
RC=$?
echo "[bievr_run_one] process_bag exit $RC, wall ${SECONDS} s"

sleep 2
rosnode kill /bievr_rec >/dev/null 2>&1 || true
sleep 2
kill -INT "$ROSCORE_PID" >/dev/null 2>&1 || true
sleep 1
[ -f "${PREFIX}_odom.bag.active" ] && rosbag reindex "${PREFIX}_odom.bag.active" >/dev/null 2>&1 \
  && mv "${PREFIX}_odom.bag.active" "${PREFIX}_odom.bag"
[ "$BAG" != "$IN" ] && rm -f "$BAG"
echo "[bievr_run_one] DONE -> ${PREFIX}.tum ($(wc -l < "${PREFIX}.tum" 2>/dev/null || echo 0) poses)"
exit $RC
