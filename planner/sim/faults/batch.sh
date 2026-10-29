#!/usr/bin/env bash
# Run failure-test cases one after another (one SITL run at a time), rerunning a case once under a new
# name ("<run>r") when SITL's watchdog reset it (GOT SIGALRM under host load).
#
#   faults/batch.sh case:run [case:run ...]     e.g. batch.sh coast_fly_super:f3 coast_fly_bridge:f4
# Progress lines go to ~/sim/l2/faults_batch.log.
set -uo pipefail
B=~/sim/l2; LOG=$B/faults_batch.log
for pair in "$@"; do
  c=${pair%%:*}; r=${pair##*:}
  for name in $r ${r}r; do
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
