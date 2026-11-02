#!/usr/bin/env bash
# Run failure-test cases one after another (one SITL run at a time), rerunning a case once under a new
# name ("<run>r") when SITL's watchdog reset it (GOT SIGALRM under host load).
#
#   faults/batch.sh case:run [case:run ...]     e.g. batch.sh coast_fly_super:f3 coast_fly_bridge:f4
# Progress lines go to ~/sim/l2/faults_batch.log.
# Env WAIT_CMD: a shell command; before each case, wait (60 s steps) while it succeeds and log its output, e.g.
#   another agent's SITL: WAIT_CMD="docker ps --format '{{.Names}}' | grep -E '^superleak|^l2-sitl-' | grep -vE 'mr4f|click|^l2-sitl-lf'"
set -uo pipefail
B=${L2_BASE:-$HOME/sim/l2}; LOG=$B/faults_batch.log   # L2_BASE: a private copy (sitl_harness.sh)
for pair in "$@"; do
  c=${pair%%:*}; r=${pair##*:}
  for name in $r ${r}r; do
    if [ -n "${WAIT_CMD:-}" ]; then
      while w=$(bash -c "$WAIT_CMD" 2>/dev/null) && [ -n "$w" ]; do
        echo "$(date +%T) waiting for: $(echo $w)" >> $LOG; sleep 60
      done
    fi
    echo "$(date +%T) start $c $name (load $(cut -d' ' -f1 /proc/loadavg))" >> $LOG
    $B/planner_sim/faults/cases.sh $c $name > $B/$name.out 2>&1
    if grep -q SIGALRM $B/runs/$name/logs/sitl_stdout.log 2>/dev/null; then
      echo "$(date +%T) $c $name: SITL watchdog reset" >> $LOG
      continue
    fi
    echo "$(date +%T) done $c $name: $(grep -o '"end": {[^}]*}' $B/runs/$name/fault_result.json 2>/dev/null | head -1 | cut -c1-220)" >> $LOG
    break
  done
done
echo "$(date +%T) batch finished" >> $LOG
