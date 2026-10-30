#!/usr/bin/env bash
# L2 closed loop on Shannon: one mission flown and driven by SUPER, the track planner, the planner bridge
# and the mission executive against QuadRover SITL, in an L2 run (sitl_harness.sh). SITL only: nothing
# reaches a real FC.
#
#   closed_loop.sh <name> <map.pcd> <start_x> <start_y> <start_yaw_deg> <mission_map_frame.yaml> <super.yaml> [timeout_s]
#
# Needs ~/sim/l2/planner (a copy of slam-agent planner/). Writes ~/sim/l2/runs/<name>/:
#   logs/super.log (each line prefixed with its wall time, for the replan rate), logs/planner_bridge.log,
#   logs/landing_check.log, logs/executive.log, logs/fly_mission.log, closed_loop.bag (rosbag of the
#   run), mission_fc.yaml, logs/track_planner.log. A mission's land legs are checked by the landing_check node
#   (l4_landing, /planner/landing_*); its drive legs go to the track_planner node (l4_track, /planning/drive_goal), whose carrot
#   (/track_planner/pos_cmd) the bridge streams in its drive mode (/planner_bridge/set_mode, set by the
#   executive per leg).
#
# Env: DRIVE_PARMS (default below): Rover driving params appended to the run's EXTRA_PARMS, the nuc-04
#   robot's own (ardupilot quadrover_overrides_nuc-04.param; the generic hardware file SITL loads has
#   none, so SITL had rover.parm's WP_RADIUS 3, which counts a 0.5 m carrot as reached). With them the
#   SITL rover tracks the track planner's path to ~5 cm (cl33, cl36); rover.parm's own calibration
#   (CRUISE_SPEED 5, CRUISE_THROTTLE 30, ATC_STR_RAT_FF 0.75) drove 1 m arcs off the path (cl38). The
#   robot's CRUISE values over-drive SITL's motor model: speed peaks ~1 m/s at WP_SPEED 0.5.
#   SITL's motor model (SimRover's generic: 20 m/s at full throttle, 2 s lag) is not the robot's: ~5 % throttle
#   per 1 m/s, so the robot's CRUISE_SPEED 0.5 / CRUISE_THROTTLE 50 is a 10x throttle feed-forward in SITL
#   (0.8-1.9 m/s against a 0.2-0.5 m/s demand, g-runs). SITL's own calibration for the same meaning:
#   CRUISE_SPEED 1.0;CRUISE_THROTTLE 5 (d6-d13). The track planner gets the run's WP_SPEED as its drive_speed
#   (its run-out). PLANNER_DIR: another planner/ copy to run (default ~/sim/l2/planner).
#   L4_WS: the C++ planner nodes (planner/ros: l4_track, l4_landing), a catkin install space built in
#   l4-sim-l2 with CMAKE_INSTALL_PREFIX=/ws/install (default ~/sim/cpp_ws/install); copied to /ws/install.
#   L4_BRIDGE_WS: the C++ planner bridge and mission executive (planner/ros: l4_common, l4_bridge,
#   l4_executive), an install space built the same way with CMAKE_INSTALL_PREFIX=/ws/install (default
#   ~/sim/cpp_ws_bx/install, ~/sim/cpp_ws_bx/build.sh install); copied to /ws_bx/install. The nodes run
#   as /ws_bx/install/lib/l4_bridge/planner_bridge and /ws_bx/install/lib/l4_executive/mission_executive
#   (fly_mission.py starts the executive; the fault cases pkill those paths). The same install space may
#   hold all five packages: then point both at it.
#   BRIDGE_IMPL (default cpp): py starts the former Python bridge and executive from PLANNER_DIR instead
#   (A/B against a planner/ copy that still has them, e.g. ~/sim/l2/planner_bx_py: the baseline b1).
#   DRIVE_PARMS="" drops them. LIDAR_UP (default 0.25 m here): the render node's lidar above SITL's ground, out of the
#   maps' floor smear (render_node.py). Any other env of sitl_harness.sh up (EXTRA_PARMS, CPUS, ...)
#   passes through. TRACK_Z_LO (default 0.30 m here, the node's own default 0.15): the track planner's
#   obstacle band floor. The render node raycasts a merged map whose floor is a 0.1-0.3 m smear that
#   also steps 0.15-0.25 m between areas (map drift baked in); at 0.15 the live window's evidence memory
#   held most of the floor beyond 1.5 m as walls (cl31). 0.30 is what the track planner (l4_track) uses on whole maps
#   for the same smear. Sim only.
# Failure tests (faults/cases.sh): FAULT_HOOK, a command started in the ROS container just before
#   fly_mission.py (its output in logs/fault.log); END_HOLD_S, seconds fly_mission.py waits after the
#   mission ends before it lands SITL (default 0); RECORD_EXTRA, more topics for the bag.
set -uo pipefail
name=${1:?name}; map=${2:?map}; sx=${3:?}; sy=${4:?}; syaw=${5:?}; mission=${6:?mission}; cfg=${7:?super config}
timeout=${8:-900}
BASE=${L2_BASE:-$HOME/sim/l2}   # L2_BASE: a private copy of ~/sim/l2 (sitl_harness.sh)
H=$BASE/planner_sim/sitl_harness.sh
RUN=$BASE/runs/$name; ROS=l2-ros-$name
DRIVE_PARMS=${DRIVE_PARMS-WP_SPEED 0.5;WP_RADIUS 0.3;WP_PIVOT_ANGLE 60;CRUISE_SPEED 0.5;CRUISE_THROTTLE 50;ATC_STR_RAT_FF 0.3}
export EXTRA_PARMS="${DRIVE_PARMS};${EXTRA_PARMS:-}"
export LIDAR_UP=${LIDAR_UP-0.25}
TRACK_Z_LO=${TRACK_Z_LO-0.30}
# the track planner's drive_speed (its run-out, l4_track::run_out; capped at 1.0 m/s): the run's WP_SPEED
TRACK_SPEED=$(echo "$DRIVE_PARMS" | tr ';' '\n' | awk '$1=="WP_SPEED" {print $2}')

$H up $name $map $sx $sy $syaw
docker cp ${PLANNER_DIR:-$BASE/planner} $ROS:/planner   # PLANNER_DIR: another planner/ copy (a private test)
docker exec $ROS mkdir -p /ws && docker cp ${L4_WS:-$HOME/sim/cpp_ws/install} $ROS:/ws/install
# the C++ nodes (rosrun in their install space; the node's name and params follow as $0 and $@)
LANDING_CMD="bash -c 'source /ws/install/setup.bash; exec rosrun l4_landing landing_check \$0 \$@'"
TRACK_CMD="bash -c 'source /ws/install/setup.bash; exec rosrun l4_track track_planner \$0 \$@'"
BRIDGE_IMPL=${BRIDGE_IMPL:-cpp}
if [ "$BRIDGE_IMPL" = py ]; then
  BRIDGE_CMD="python3 -u /planner/planner_bridge.py"
else
  docker exec $ROS mkdir -p /ws_bx && docker cp ${L4_BRIDGE_WS:-$HOME/sim/cpp_ws_bx/install} $ROS:/ws_bx/install
  BRIDGE_CMD=/ws_bx/install/lib/l4_bridge/planner_bridge
fi
docker cp $cfg $ROS:/super_ws/src/SUPER/super_planner/config/super_closed_loop.yaml
docker cp $mission $ROS:/run/mission_map.yaml

# wait for the robot's bridge to feed vision and the EKF to settle on it (l2_check.py waits the same way)
for _ in $(seq 1 90); do
  $H exec $name "timeout 3 rostopic echo -n 1 /mavros/vision_pose/pose" >/dev/null 2>&1 && break
  sleep 2
done
sleep 20

$H exec $name "rosparam load /planner/config/planner_bridge.yaml /planner_bridge; \
  rosparam set /planner_bridge/speed_cap 1.0; rosparam set /planner_bridge/fc_mount_yaw_deg -0.45; \
  nohup $BRIDGE_CMD __name:=planner_bridge > /run/logs/planner_bridge.log 2>&1 &
  nohup $LANDING_CMD __name:=landing_check > /run/logs/landing_check.log 2>&1 &
  nohup $TRACK_CMD __name:=track_planner _z_lo:=$TRACK_Z_LO _drive_speed:=${TRACK_SPEED:-1.0} > /run/logs/track_planner.log 2>&1 &
  nohup bash -c 'stdbuf -oL rosrun super_planner fsm_node _config_name:=super_closed_loop.yaml 2>&1 | python3 -u /planner_sim/stamp_lines.py' > /run/logs/super.log 2>&1 &
  nohup rosbag record -O /run/closed_loop.bag --lz4 \
    /planner/odom /planner/cloud /Odometry /mavros/local_position/pose /mavros/vision_pose/pose \
    /mavros/setpoint_raw/local /mavros/state /planning/pos_cmd /planning/click_goal \
    /planner_bridge/status /planner_bridge/anchor /mission_executive/status /mission_executive/leg \
    /planner/landing_spot /planner/landing_clear /planner/landing_status /planner/landing_markers \
    /planner_bridge/set_mode /planner_bridge/mode /planner_bridge/mode_code /planner_bridge/enable /planner_bridge/yaw_cmd \
    /planning/drive_goal /track_planner/pos_cmd /track_planner/path /track_planner/status \
    /mavros/statustext/recv /mavros/local_position/velocity_local \
    /fsm_node/visualization/exp_traj /fsm_node/visualization/backup_traj /fsm_node/visualization/committed_traj \
    /fsm_node/visualization/exp_sfc /fsm_node/visualization/backup_sfc /fsm_node/visualization/goal \
    ${RECORD_EXTRA:-} \
    > /run/logs/record.log 2>&1 &
  sleep 1"
sleep 15
# failure tests (faults/cases.sh): a fault runner started just before the mission
[ -n "${FAULT_HOOK:-}" ] && $H exec $name "nohup $FAULT_HOOK > /run/logs/fault.log 2>&1 &
sleep 1"
$H exec $name "END_HOLD_S=${END_HOLD_S:-0} BRIDGE_IMPL=$BRIDGE_IMPL python3 -u /planner_sim/fly_mission.py /run/mission_map.yaml $timeout" > $RUN/logs/fly_mission.log 2>&1
$H exec $name "pkill -INT -f 'rosbag record'; sleep 5; true"
grep -c "ReplanOnce succeed" $RUN/logs/super.log
tail -n 3 $RUN/logs/fly_mission.log
docker logs l2-sitl-$name > $RUN/logs/sitl_stdout.log 2>&1   # SITL's console (watchdog: "GOT SIGALRM")
$H down $name
echo "closed loop $name done: $RUN"
