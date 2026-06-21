#!/bin/bash
# Repeatedly replay a bag through EllipseLIO (with /analytics logged, via run_bag_foxglove.sh)
# until a TERMINAL RUNAWAY is caught (final displacement from origin > threshold) or max runs.
# Each run's analytics.csv + trajectory.csv are saved under <bag>_runaway_hunt/runN_*. Purpose:
# capture the analytics of a RUNAWAY run so its obs_score/num_feats can be compared to a clean
# run — i.e. test whether EllipseLIO's native obs_score is a usable flyaway gate.
#
# EllipseLIO on 3rd_floor is a ~coin-flip: clean runs close to <0.2m, runaways end >5m. So a
# final_disp > 3m threshold cleanly separates them. (Catches the TERMINAL runaway; the
# operator returns the sensor to origin so a large final_disp == failed closure == runaway.)
#
# Usage: loop_until_runaway.sh <bag_dir_name> [max_runs=6] [thresh_m=3.0]
set -uo pipefail
BAG="${1:?usage: loop_until_runaway.sh <bag_dir_name> [max_runs] [thresh_m]}"
MAX="${2:-6}"
THRESH="${3:-3.0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HOME/ellipselio_ws/results/field/$BAG"
SAVE="$HOME/ellipselio_ws/results/field/${BAG}_runaway_hunt"
mkdir -p "$SAVE"

for i in $(seq 1 "$MAX"); do
  echo "########################## RUN $i / $MAX ##########################"
  "$HERE/run_bag_foxglove.sh" "$BAG" || true
  # preserve this run's data before the next iteration overwrites OUTDIR
  cp "$OUT/analytics.csv"  "$SAVE/run${i}_analytics.csv"  2>/dev/null || true
  cp "$OUT/trajectory.csv" "$SAVE/run${i}_trajectory.csv" 2>/dev/null || true
  FD=$(grep -oE 'final_disp_from_start=[0-9.]+' "$OUT/sampler.log" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  MS=$(grep -oE 'max_single_step=[0-9.]+'      "$OUT/sampler.log" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  PK=$(grep -oE 'peak_excursion=[0-9.]+'       "$OUT/sampler.log" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  echo ">>> RUN $i RESULT: final_disp=${FD:-?}m  peak=${PK:-?}m  max_step=${MS:-?}m   (saved run${i}_*.csv)"
  if awk -v fd="${FD:-0}" -v th="$THRESH" 'BEGIN{exit !(fd+0 > th+0)}'; then
    echo "######## RUNAWAY CAUGHT on run $i: final_disp=${FD}m > ${THRESH}m ########"
    echo "runaway analytics: $SAVE/run${i}_analytics.csv"
    echo "RUNAWAY_RUN=$i"
    exit 0
  fi
done
echo "######## no runaway in $MAX runs (all closed < ${THRESH}m) — try more runs ########"
exit 0
