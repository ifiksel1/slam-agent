#!/bin/bash
# closeups.sh -- record a close-up video of every flight in the library: <flight>_closeup.mp4 in its folder.
#
# For each flight in a fixed order: open the bag in a new Foxglove tab on :0 (open_bag.sh), wait until it is
# loaded, record Foxglove playing it with record.sh for the bag's length + 3 s, then close that tab (ctrl+w).
# Foxglove must already be running on :0 with layouts/closeup_layout.json imported; the script selects it in
# each new tab (layout menu -> search -> pick). Its 3D panel follows the "drone" TF frame. Skips flights whose close-up video already exists, so it can be rerun.
# When all are done it runs gen.py to regenerate README.md / index.html / notes.md with the close-up links.
#
# Safety for the owner's Foxglove tabs (7 open; closing the last tab quits Foxglove):
#  - the tab this script opens is always the 8th; ctrl+w is sent only after clicking tab 8 and checking on screen
#    (tabs.py) that tab 8 is the active one and that Foxglove is the focused window;
#  - if tab 8 is already open when a flight starts (not ours to know), the script stops.
# Log: tools/closeups.log.  Run in tmux:  tmux new -d -s closeups 'bash ~/sim/foxglove/tools/closeups.sh'
set -u
F=~/sim/foxglove; T=$F/tools; LOG=$T/closeups.log
PY=~/sim/venv/bin/python
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
OUR_SLOT=8                       # the owner has 7 tabs; ours opens as the 8th
TAB8_X=1790; TAB_Y=46            # centre of tab 8 (tabs are 200 px wide from x=290)

FLIGHTS=(
  closed_loop/closed_loop_hp_real_route
  planner_only/canusa_A_inside
  planner_only/canusa_B_climb
  planner_only/canusa_C_out_the_gap
  planner_only/hensel_phelps_1_real_route
  planner_only/hensel_phelps_1_real_route_1.3m
  planner_only/hensel_phelps_1_real_route_floor_lowered
  planner_only/hensel_phelps_1_real_route_robot
  planner_only/hensel_phelps_2_tight_loop
  planner_only/hensel_phelps_3_north
  closed_loop/closed_loop_canusa_A
  closed_loop/closed_loop_canusa_A_corridors_fixed
  drive_fly/closed_loop_drive_fly
  drive_fly/closed_loop_drive_fly_cl33
  gate_l2/gate_l2_course
  gate_l2/gate_l2_course_pass
  faults/fault_slam_freeze_fly
  faults/fault_obstacle_appears_fly
  faults/fault_mavros_link_loss_fly
)

log() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
slot() { $PY $T/tabs.py; }
fg_alive() { pgrep -f "foxglove-portable/opt/Foxglove/foxglove-studio" >/dev/null; }

focus_fg() {   # make Foxglove the active window; wait (up to 10 min) if it cannot be
  local i
  for i in $(seq 1 120); do
    python3 $T/xwin.py activate_fg >/dev/null 2>&1 && return 0
    [ $i = 1 ] && log "  waiting: cannot focus Foxglove ($(python3 $T/xwin.py active 2>&1 | cut -f2-))"
    sleep 5
  done
  return 1
}

close_our_tab() {   # close tab 8, only after checking it is ours and active
  local i
  for i in 1 2 3; do
    focus_fg || return 1
    python3 $T/xin.py click $TAB8_X $TAB_Y; sleep 1
    python3 $T/xin.py move 2600 $TAB_Y
    if [ "$(slot)" = "$OUR_SLOT" ] && python3 $T/xwin.py fg_active >/dev/null 2>&1; then
      python3 $T/xin.py key ctrl+w; sleep 3
      if [ "$(slot)" != "$OUR_SLOT" ]; then log "  closed tab $OUR_SLOT"; return 0; fi
    fi
    sleep 2
  done
  log "  ERROR: could not close tab $OUR_SLOT safely"
  return 1
}

select_layout() {   # make closeup_layout the layout of the active tab (our tab 8): layout menu, search, pick
  local i
  for i in 1 2 3; do
    focus_fg || return 1
    [ "$(slot)" = "$OUR_SLOT" ] || return 1
    python3 $T/xin.py click 3630 86; sleep 1.5
    if $PY $T/menu_open.py; then
      python3 $T/xin.py type closeup_layout; sleep 1
      python3 $T/xin.py click 3485 163; sleep 3     # the single search result
      python3 $T/xin.py move 2600 $TAB_Y
      $PY $T/menu_open.py || return 0
      python3 $T/xin.py key Escape
    fi
    sleep 2
  done
  return 1
}

bag_len() { $PY -c "
from rosbags.rosbag1 import Reader
with Reader('$1') as r: print(int((r.end_time - r.start_time) / 1e9 + 0.999))"; }

vid_len() { $PY -c "
import imageio_ffmpeg, re, subprocess
e = subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-i', '$1'], capture_output=True, text=True).stderr
m = re.search(r'Duration: (\d+):(\d+):([\d.]+)', e)
print(int(int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3])) if m else 0)"; }

record_one() {   # record_one CAT/NAME -> 0 ok, 1 failed (retry later), 2 fatal (stop)
  local rel=$1 name=${1#*/} dir=$F/$1 bag out tmp len mb wait_s i mon bad
  bag=$dir/$name.bag; out=$dir/${name}_closeup.mp4; tmp=$dir/.${name}_closeup.part.mp4
  [ -f "$bag" ] || { log "  ERROR: no bag $bag"; return 1; }
  len=$(bag_len "$bag"); mb=$(( $(stat -c %s "$bag") / 1000000 ))
  log "$rel: bag ${len} s, ${mb} MB"
  fg_alive || { log "  FATAL: Foxglove is not running"; return 2; }
  focus_fg || { log "  FATAL: Foxglove cannot be focused"; return 2; }
  # tab 8 must not exist yet: click where it would be and see whether it becomes active
  python3 $T/xin.py click $TAB8_X $TAB_Y; sleep 1; python3 $T/xin.py move 2600 $TAB_Y
  if [ "$(slot)" = "$OUR_SLOT" ]; then
    log "  FATAL: a tab $OUR_SLOT is already open (left over?). Close it by hand, then rerun."; return 2
  fi
  bash $T/open_bag.sh "$bag"
  for i in $(seq 1 30); do sleep 2; [ "$(slot)" = "$OUR_SLOT" ] && break; done
  [ "$(slot)" = "$OUR_SLOT" ] || { log "  ERROR: the bag's tab did not appear as tab $OUR_SLOT"; return 1; }
  wait_s=$(( 8 + mb / 40 )); sleep $wait_s
  select_layout || { log "  ERROR: could not select closeup_layout"; close_our_tab || return 2; return 1; }
  sleep 3
  log "  opened as tab $OUR_SLOT, waited ${wait_s} s to load; recording $((len + 3)) s"
  focus_fg || return 2
  rm -f "$tmp" "$tmp.bad"
  ( while sleep 3; do python3 $T/xwin.py fg_active >/dev/null 2>&1 || echo "$(date +%T) $(python3 $T/xwin.py active | cut -f2-)" >> "$tmp.bad"; done ) &
  mon=$!
  bash $T/record.sh "$tmp" $((len + 3)) >/dev/null 2>>"$LOG"
  kill $mon 2>/dev/null; wait $mon 2>/dev/null
  bad=""; [ -s "$tmp.bad" ] && bad="Foxglove lost focus during recording: $(head -c 300 "$tmp.bad" | tr '\n' ';')"
  rm -f "$tmp.bad"
  close_our_tab || return 2
  if [ -n "$bad" ]; then log "  RETRY LATER: $bad"; rm -f "$tmp"; return 1; fi
  i=$(vid_len "$tmp")
  if [ "$i" -lt "$len" ]; then log "  RETRY LATER: video is ${i} s, bag ${len} s"; rm -f "$tmp"; return 1; fi
  mv "$tmp" "$out"
  log "  wrote ${out#$F/} (${i} s, $(( $(stat -c %s "$out") / 1000000 )) MB)"
  return 0
}

log "=== closeups.sh start (pid $$)"
todo=()
for rel in "${FLIGHTS[@]}"; do
  name=${rel#*/}
  if [ -s "$F/$rel/${name}_closeup.mp4" ]; then log "$rel: close-up exists, skip"; else todo+=("$rel"); fi
done
for pass in 1 2; do
  [ ${#todo[@]} = 0 ] && break
  [ $pass = 2 ] && log "--- pass 2: retrying ${todo[*]}"
  failed=()
  for rel in "${todo[@]}"; do
    record_one "$rel"; rc=$?
    [ $rc = 2 ] && { log "=== STOPPED (fatal). Fix and rerun; finished videos are kept."; exit 1; }
    [ $rc = 1 ] && failed+=("$rel")
  done
  todo=("${failed[@]+"${failed[@]}"}")
done
[ ${#todo[@]} -gt 0 ] && log "NOT DONE after 2 passes: ${todo[*]}"
log "regenerating README.md, index.html, notes.md (gen.py)"
$PY $T/gen.py >> "$LOG" 2>&1 && log "gen.py ok" || log "gen.py FAILED"
log "=== closeups.sh done"
