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
GONE=10            # observe this long after the executive exits (inject.py --gone_hold)
ACT=()
# low battery cases: the robot's battery failsafe params, and SITL's simulated pack voltage set through MAVROS
BATT="SIM_BATT_VOLTAGE 12.6;SIM_BATT_CAP_AH 0;BATT_LOW_TIMER 10;BATT_FS_LOW_ACT 2;BATT_FS_CRT_ACT 2"
batt_v() { echo "shell:rosservice call /mavros/param/set \"{param_id: SIM_BATT_VOLTAGE, value: {integer: 0, real: $1}}\""; }
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
  # 2d with a 30 s freeze (2026-11-03, the ground vision cut: the EKF dead-reckoning that long, any failsafe)
  stale_drive_freeze_long) TRIG=$DRV; HOLD=45; MAXS=200; ACT=('render:{"stale": "freeze"}' "sleep:30" 'render:{"stale": "none"}') ;;
  # 2b manual: a frozen pose in a manual flight (no executive, no planner bridge): mid fly leg 3 the executive and
  #   the planner bridge are killed, QRLOITER set (the pilot's mode), and 0.3 s later the pose freezes while
  #   QRLoiter brakes from ~1 m/s (MAV_CMD_DO_SET_MODE 19: MAVROS's set_mode does not know the QR names). Only the SLAM-to-FC bridge's own detection can act (vision cut, owner 2026-11-02)
  manual_fly_freeze)   TRIG=$FLY; HOLD=34; MAXS=30   # MAXS: inject.py's pgrep sees its own --action args as the executive
                       ACT=("pkill:-9 -f '^/ws[^ ]*/lib/l4_executive/mission_executive'"
                            "shell:rosservice call /mavros/cmd/command \"{command: 176, param1: 1, param2: 19}\""
                            "pkill:-9 -f '^/ws[^ ]*/lib/l4_bridge/planner_bridge'" "sleep:0.3"
                            'render:{"stale": "freeze"}' "sleep:8" 'render:{"stale": "none"}') ;;
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
  # 5a near: the pillar 1.8 m ahead (its face 1.5 m), mid fly leg 3: SUPER's committed trajectory runs into it
  #   before its map has it and its replans fail (the f25r path); the bridge's path_blocked must hold
  obst_fly_near)       TRIG=$FLY; ACT=("box_ahead:1.8:0.6,0.6,3.0:1.5") ;;
  # no fault: the Gate L2 obstacle course (planner_sim/course/hp_gate_l2.pcd, missions/gate_l2_hp_course.yaml;
  #   d12/d13/d18): no false path_blocked, the drive stops; clearance: course/clearance.py
  gate_l2)             MAP=$PS/course/hp_gate_l2.pcd; MISSION=$BASE/planner/missions/gate_l2_hp_course.yaml
                       TRIG="start"; MAXS=300; ACT=(none) ;;
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
  # 8 low battery (L2 task list: "low battery (QRLand)"). SITL's pack is constant at SIM_BATT_VOLTAGE (SIM_BATT_CAP_AH 0);
  #   the injection sets it through MAVROS's param service. Thresholds as sitl_overrides.parm scales the robot's 6S pack
  #   (low 10.8 / critical 10.5 V; robot 21.6 / 21.0); the robot's BATT_LOW_TIMER 10, BATT_FS_LOW_ACT 2, BATT_FS_CRT_ACT 2
  #   (Hold: QRLand in copter flight, Rover Hold otherwise; Rover/failsafe.cpp handle_battery_failsafe), set again
  #   here. Each stage fires BATT_LOW_TIMER s after the voltage goes under it. Staged: 10.65 V (low), 20 s later 10.35 V
  #   (critical). batt_fly: mid fly leg 3, the low failsafe ~10 s later on leg 4. batt_drive: at the land leg's NAV_LAND
  #   (the drive leg lasts ~7 s, shorter than the timer), the low failsafe ~10 s later mid drive leg 6.
  #   *_crit: straight to 10.35 V (a sudden sag: only the critical stage fires)
  #   batt_fly's failsafe comes as the vehicle turns at leg 4's end (bf02, bf03: 0.1-0.2 m/s); batt_fly_cruise injects at
  #   leg 4's start so that it comes in transit on leg 5 (the land leg's fly-over)
  batt_fly)            TRIG=$FLY; HOLD=40; PARMS="$PARMS;$BATT"; ACT=("$(batt_v 10.65)" "sleep:20" "$(batt_v 10.35)") ;;
  batt_fly_cruise)     TRIG="leg_type=fly,leg=4"; HOLD=40; PARMS="$PARMS;$BATT"; ACT=("$(batt_v 10.65)" "sleep:20" "$(batt_v 10.35)") ;;
  batt_drive)          TRIG="phase=LAND,leg=5"; HOLD=40; PARMS="$PARMS;$BATT"; ACT=("$(batt_v 10.65)" "sleep:20" "$(batt_v 10.35)") ;;
  batt_fly_crit)       TRIG=$FLY; HOLD=30; PARMS="$PARMS;$BATT"; ACT=("$(batt_v 10.35)") ;;
  batt_drive_crit)     TRIG="phase=LAND,leg=5"; HOLD=30; PARMS="$PARMS;$BATT"; ACT=("$(batt_v 10.35)") ;;
  *) echo "unknown case $case_ ($0 list)"; exit 2 ;;
esac
[ -n "$run" ] || { echo "usage: $0 <case> <run name>"; exit 2; }
# Env CASE_PARMS: more SITL-only params for this run ("NAME VALUE;NAME VALUE"), e.g. an A/B of a Rover setting
[ -n "${CASE_PARMS:-}" ] && PARMS="$PARMS;$CASE_PARMS"

# the fault runner's command, as a script the ROS container reads from /planner_sim (mounted read-only)
mkdir -p $F/hooks
hook=$F/hooks/$run.sh
{
  echo "#!/usr/bin/env bash"
  echo "# $case_ ($run), written by faults/cases.sh"
  [ -n "${HOOK_PRE:-}" ] && echo "$HOOK_PRE   # HOOK_PRE"
  printf 'nohup python3 -u /planner_sim/faults/mav_watch.py %s > /dev/null 2>&1 &   # its record: logs/mav_watch.jsonl\n' "$GCS"
  printf 'exec python3 -u /planner_sim/faults/inject.py --case %q --trigger %q --end_hold %q --max_s %q --gone_hold %q' "$case_" "$TRIG" "$HOLD" "$MAXS" "$GONE"
  for a in "${ACT[@]}"; do printf ' --action %q' "$a"; done
  echo
} > $hook

cd $BASE
RENDER_SCRIPT=/planner_sim/faults/render_fault.py SITL_ARGS="--serial3=tcp:4" EXTRA_PARMS="$PARMS" \
  END_HOLD_S=$HOLD FAULT_HOOK="bash /planner_sim/faults/hooks/$run.sh" \
  RECORD_EXTRA="/sim/truth /sim_fault/state /sim_fault/cmd /fault/events /planning_cmd/poly_traj /fastlio_mavros_bridge/frozen_status /mavros/rc/out" \
  $PS/closed_loop.sh $run $MAP 0 0 -90.7 $MISSION $CFG 400
echo "case $case_ in $run:"
grep -h "SIGALRM" $BASE/runs/$run/logs/sitl_stdout.log 2>/dev/null | head -3 && echo "SITL WATCHDOG RESET: rerun this case"
tail -n 3 $BASE/runs/$run/logs/fault.log
