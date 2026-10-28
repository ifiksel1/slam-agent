#!/usr/bin/env bash
# L2 closed-loop sim on Shannon: QuadRover SITL + MAVROS + the robot's SLAM-to-FC bridge + the render
# node, all in one private network namespace per run (the ROS container has --network none; SITL's
# container joins it), so nothing reaches the host network, the revolute-gcs SITL on 5760-5763, or a
# real flight controller.
#
#   sitl_harness.sh up <name> <map.pcd> [start_x start_y start_yaw_deg]   (env LIDAR_UP, LATENCY_S, FAULT, FAULT_START_S, FAULT_RATE, HW_DROP, HW_KEEP, HOME_YAW, EXTRA_PARMS)
#   sitl_harness.sh down <name>
#   sitl_harness.sh status <name>
#   sitl_harness.sh exec <name> <command...>        run inside the ROS container (ROS env sourced)
#
# Paths (Shannon): SLAM repo copy ~/sim/l2/slam (config/, overrides/), this folder ~/sim/l2/planner_sim,
# SITL build ~/sim/ardupilot, raycaster ~/sim/marsim, runs ~/sim/l2/runs/<name>.
set -uo pipefail
cmd=${1:?up|down|status|exec}; name=${2:?run name}
ROS=l2-ros-$name; SITL=l2-sitl-$name
BASE=~/sim/l2; RUN=$BASE/runs/$name
AP=~/sim/ardupilot

down() { docker rm -f $SITL $ROS >/dev/null 2>&1 || true; }

case $cmd in
up)
  map=${3:?map.pcd}; sx=${4:-0}; sy=${5:-0}; syaw=${6:-0}
  down
  rm -rf $RUN; mkdir -p $RUN/logs $RUN/sitl
  # the robot's hardware params without what belongs to that board or its props: IMU/compass/baro IDs
  # and calibration (SITL: "3D Accel calibration needed"), the battery monitor's wiring, the motor
  # outputs (the robot's SERVO1-4 are motors 4,1,3,2; SITL's model expects 1-4: it flips on takeoff),
  # the motor/thrust model (Q_M_*: DShot, hover 0.37 for the real props) and the attitude controller
  # tuned for the real airframe (rate/angle gains 3-4x the SITL model's, input TC 0.35: with them the
  # model never finishes its takeoff and swings +-2 m in Guided). The position/velocity loops, speeds,
  # limits, land/RTL and the EKF stay the robot's.
  grep -v -E "^(INS_|COMPASS_|BARO|BATT_(MONITOR|VOLT_|AMP_|CURR_|SERIAL_NUM)|BATT_VLT_OFFSET|SERVO[0-9]+_|Q_M_|Q_A_RAT_|Q_A_ANG_(RLL|PIT|YAW)_P|Q_A_ACC_|Q_A_INPUT_TC|Q_A_ANG_LIM_TC|Q_A_RATE_)" $AP/quadrover_from_pixracer.param \
    | grep -v -E "${HW_DROP:-^\$}" | grep -E "${HW_KEEP:-.}" > $RUN/sitl/hw.parm   # bisecting: HW_DROP / HW_KEEP regexes
  # sim-only experiments on top of everything: EXTRA_PARMS="Q_WP_SPD 1.0;NAME VALUE"
  echo "${EXTRA_PARMS:-}" | tr ';' '\n' | sed '/^ *$/d' > $RUN/sitl/extra.parm
  # ONE defaults file, in load order (a later line wins). ArduPilot (AP_Param::read_param_defaults_file)
  # sets done_all_default_params from the LAST file of a --defaults list only; when that file holds no
  # param that is unknown on the first pass (an empty extra.parm), the dynamically registered tables
  # (Q_*, MAVn_*) never get their defaults re-applied and come up at firmware defaults. cl8 flew that
  # way: MAV1_POSITION 1 (LOCAL_POSITION_NED 1 Hz, the planner bridge capped against a stale pose),
  # Q_WP_SPD 10 and stock Q_A_/Q_P_ gains. A single file makes the flag cover every line.
  cat $AP/Tools/autotest/default_params/rover.parm $AP/Tools/autotest/default_params/quadrover.parm \
    $RUN/sitl/hw.parm $BASE/planner_sim/sitl_overrides.parm $RUN/sitl/extra.parm > $RUN/sitl/defaults.parm
  docker run -d --name $ROS --network none --cpus ${CPUS:-6} \
    -e ROS_HOSTNAME=localhost -e ROS_MASTER_URI=http://localhost:11311 \
    -e MAP_PCD=/map/$(basename $map) -e START_X=$sx -e START_Y=$sy -e START_YAW_DEG=$syaw \
    -e LIDAR_UP=${LIDAR_UP:-0} -e LATENCY_S=${LATENCY_S:-0.07} -e FAULT=${FAULT:-none} -e FAULT_START_S=${FAULT_START_S:-1e9} -e FAULT_RATE=${FAULT_RATE:-0.05} \
    -v $(realpath $map):/map/$(basename $map):ro -v $BASE/slam:/slam:ro -v $BASE/planner_sim:/planner_sim:ro \
    -v ~/sim/marsim:/marsim:ro -v $RUN:/run \
    --entrypoint bash l4-sim-l2:latest -c "sleep infinity" >/dev/null
  # SITL needs glibc 2.34 (built on Shannon's 22.04): its own container, in the ROS container's network
  docker run -d --name $SITL --network container:$ROS --cpus 2 -w /run/sitl \
    -v $AP:/ap:ro -v $BASE/planner_sim:/planner_sim:ro -v $RUN:/run ubuntu:22.04 \
    /ap/build/sitl/bin/ardurover --model quadrover --home 42.56335,12.64329,163,${HOME_YAW:-0} -I0 -w \
    --defaults /run/sitl/defaults.parm \
    >/dev/null
  sleep 2
  docker exec -d $ROS bash /planner_sim/run_ros.sh
  echo "up: $name (logs $RUN/logs, SITL $RUN/sitl)"
  ;;
down) down; echo "down: $name" ;;
status) docker ps --filter name=l2-.*-$name --format '{{.Names}} {{.Status}}'; tail -n 2 $RUN/logs/*.log 2>/dev/null ;;
exec) shift 2; docker exec $ROS bash -c "source /opt/ros/noetic/setup.bash; source /super_ws/devel/setup.bash; $*" ;;
*) echo "unknown command $cmd"; exit 2 ;;
esac
