#!/usr/bin/env bash
# L2 failure tests on Shannon: one closed-loop run (closed_loop.sh) per fault case, SITL only.
#
#   faults/cases.sh <case> <run name>      e.g.  ~/sim/l2/planner_sim/faults/cases.sh coast_fly_super f2
#   faults/cases.sh list
#
# Every case flies and drives the Hensel Phelps drive-and-fly mission (missions/example_hp_drive_fly.yaml,
# clean run cl39) or a variant under faults/missions/, with:
#   - faults/render_fault.py in place of render_node.py (RENDER_SCRIPT): /sim/truth, live SLAM faults, boxes;
#   - faults/mav_watch.py on SITL's SERIAL3 (SITL_ARGS --serial3=tcp:4, SERIAL3_PROTOCOL 2): the FC's
#     modes and texts as SITL reports them (logs/mav_watch.jsonl); in the link cases also the GCS heartbeat;
#   - faults/inject.py: waits for the trigger, injects, observes, writes fault_result.json;
#   - fly_mission.py waits END_HOLD_S after the executive's DONE/STOPPED before it lands SITL, so what the
#     FC does after the stop is seen.
# SITL-only params (EXTRA_PARMS) are listed per case below and in the run's sitl/extra.parm.
# Env HOOK_PRE: a command for the ROS container before the runner starts (e.g. an A/B parameter:
#   HOOK_PRE="rosparam set /mission_executive/slam_frozen_flight_mode QRLOITER").
# Results: ~/sim/l2/runs/<run>/fault_result.json; the table: faults/results.py.
set -uo pipefail
case_=${1:?case (or list)}; run=${2:-}
BASE=${L2_BASE:-$HOME/sim/l2}; PS=$BASE/planner_sim; F=$PS/faults   # L2_BASE: a private copy (sitl_harness.sh)
MAP=~/sim/marsim/maps/hp_1015_carved.pcd
CFG=$PS/super_closed_loop_hp.yaml
MISSION=$BASE/planner/missions/example_hp_drive_fly.yaml

# triggers: mid fly leg 3 (0.11,-4.46 -> -0.28,-7.61, 3.2 m straight south) at 1 m/s; mid drive leg (6)
FLY="leg_type=fly,leg=3,min_speed=0.9,max_wait=6"
DRV="leg_type=drive,leg=6,min_speed=0.5,max_wait=10"
GCS=""             # "--gcs": mav_watch is the GCS link
PARMS="SERIAL3_PROTOCOL 2"
HOLD=15            # END_HOLD_S
MAXS=150
ACT=()
case $case_ in
  list) grep -E "^  [a-z_]+\)" $0 | sed 's/).*//'; exit 0 ;;
  baseline)            TRIG="start"; ACT=(none) ;;
  # no false frozen-pose trigger: the drive-and-fly mission with a 30 s hover at a waypoint and 20 s parked
  # after the drive (SITL's pose repeats exactly while parked; the frozen-pose check needs evidence of motion)
  hover_park)          MISSION=$F/missions/hover_park.yaml; TRIG="start"; MAXS=320; ACT=(none) ;;
  # 1 / coast (D4): the planner killed at speed; the bridge killed at speed (no setpoints at all)
  coast_fly_super)     TRIG=$FLY; ACT=("pkill:-9 -x fsm_node") ;;
  coast_fly_bridge)    TRIG=$FLY; ACT=("pkill:-9 -f '^/ws[^ ]*/lib/l4_bridge/planner_bridge'") ;;
  coast_drive_track)   TRIG=$DRV; ACT=("pkill:-9 -f '^/ws[^ ]*/lib/l4_track/track_planner'") ;;
  coast_drive_bridge)  TRIG=$DRV; ACT=("pkill:-9 -f '^/ws[^ ]*/lib/l4_bridge/planner_bridge'") ;;
  # 2 SLAM stale: /Odometry and /cloud_registered stop (FAST-LIO dead) or freeze (stuck), 8 s, then back
  stale_fly_stop)      TRIG=$FLY; ACT=('render:{"stale": "stop"}' "sleep:8" 'render:{"stale": "none"}') ;;
  stale_fly_freeze)    TRIG=$FLY; ACT=('render:{"stale": "freeze"}' "sleep:8" 'render:{"stale": "none"}') ;;
  stale_drive_stop)    TRIG=$DRV; ACT=('render:{"stale": "stop"}' "sleep:8" 'render:{"stale": "none"}') ;;
  stale_drive_freeze)  TRIG=$DRV; ACT=('render:{"stale": "freeze"}' "sleep:8" 'render:{"stale": "none"}') ;;
  # 3 SLAM reset: laserMapping restart (2 s gap, new origin) or an instant pose jump (new origin, no gap)
  reset_fly)           TRIG=$FLY; ACT=('render:{"reset": {"gap_s": 2.0}}') ;;
  jump_fly)            TRIG=$FLY; ACT=('render:{"reset": {"gap_s": 0.0}}') ;;
  reset_drive)         TRIG=$DRV; ACT=('render:{"reset": {"gap_s": 2.0}}') ;;
  # 4 goal inside an obstacle (the 0.9 m box at map (-2.6, -7.6)); on_unreachable: next on that leg
  goal_obst_fly)       MISSION=$F/missions/goal_in_box_fly.yaml; TRIG="leg_type=fly,leg=4"; ACT=(none) ;;
  goal_obst_drive)     MISSION=$F/missions/goal_in_box_drive.yaml; TRIG="leg_type=drive,leg=6"; ACT=(none) ;;
  # 5 an obstacle appears on the path: a 0.6 x 0.6 x 3 m pillar 2.5 m ahead in flight; a 0.5 x 0.5 x 1.2 m
  #   box (a person's legs, a crate) 1.5 m ahead while driving
  obst_fly)            TRIG=$FLY; ACT=("box_ahead:2.5:0.6,0.6,3.0:1.5") ;;
  obst_drive)          TRIG=$DRV; ACT=("box_ahead:1.5:0.5,0.5,1.2:0.6") ;;
  # 6 landing spot occupied: a 0.6 m crate on the spot (-6, -9) as the land leg starts its fly-over
  land_occupied)       TRIG="leg_type=fly,leg=5"; ACT=("box_at:-6.0,-9.0,0.3:0.6,0.6,0.6") ;;
  # 7 link loss: the GCS heartbeat (robot's FS_GCS_ENABLE 1, FS_GCS_TIMEOUT 2, FS_ACTION 3) stops;
  #   MAVROS dies (setpoints and the FC's vision feed both stop)
  link_gcs_fly)        TRIG=$FLY; GCS="--gcs"; PARMS="$PARMS;FS_GCS_ENABLE 1;FS_GCS_TIMEOUT 2"; HOLD=60
                       ACT=("pkill:-9 -f '^python3 -u /planner_sim/faults/mav_watch.py'") ;;
  link_gcs_drive)      TRIG=$DRV; GCS="--gcs"; PARMS="$PARMS;FS_GCS_ENABLE 1;FS_GCS_TIMEOUT 2"; HOLD=60
                       ACT=("pkill:-9 -f '^python3 -u /planner_sim/faults/mav_watch.py'") ;;
  link_mavros_fly)     TRIG=$FLY; HOLD=40; MAXS=90; ACT=("pkill:-9 -x mavros_node") ;;
  link_mavros_drive)   TRIG=$DRV; HOLD=40; MAXS=90; ACT=("pkill:-9 -x mavros_node") ;;
  *) echo "unknown case $case_ ($0 list)"; exit 2 ;;
esac
[ -n "$run" ] || { echo "usage: $0 <case> <run name>"; exit 2; }

# the fault runner's command, as a script the ROS container reads from /planner_sim (mounted read-only)
mkdir -p $F/hooks
hook=$F/hooks/$run.sh
{
  echo "#!/usr/bin/env bash"
  echo "# $case_ ($run), written by faults/cases.sh"
  [ -n "${HOOK_PRE:-}" ] && echo "$HOOK_PRE   # HOOK_PRE"
  printf 'nohup python3 -u /planner_sim/faults/mav_watch.py %s > /dev/null 2>&1 &   # its record: logs/mav_watch.jsonl\n' "$GCS"
  printf 'exec python3 -u /planner_sim/faults/inject.py --case %q --trigger %q --end_hold %q --max_s %q' "$case_" "$TRIG" "$HOLD" "$MAXS"
  for a in "${ACT[@]}"; do printf ' --action %q' "$a"; done
  echo
} > $hook

cd $BASE
RENDER_SCRIPT=/planner_sim/faults/render_fault.py SITL_ARGS="--serial3=tcp:4" EXTRA_PARMS="$PARMS" \
  END_HOLD_S=$HOLD FAULT_HOOK="bash /planner_sim/faults/hooks/$run.sh" \
  RECORD_EXTRA="/sim/truth /sim_fault/state /sim_fault/cmd /fault/events" \
  $PS/closed_loop.sh $run $MAP 0 0 -90.7 $MISSION $CFG 400
echo "case $case_ in $run:"
grep -h "SIGALRM" $BASE/runs/$run/logs/sitl_stdout.log 2>/dev/null | head -3 && echo "SITL WATCHDOG RESET: rerun this case"
tail -n 3 $BASE/runs/$run/logs/fault.log
