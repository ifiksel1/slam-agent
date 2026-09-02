#!/bin/bash
# Full 12-bag hangar sweep. Sequential, clean container per bag.
# Rate is 1.0 by default = the REAL-TIME condition (GenZ-ICP captures ~59% of a 20 Hz
# stream on this box), which is what the estimator would see in flight. Pass 0.5 for the
# best-case accuracy condition (~89% capture). Capture rate is logged per bag either way.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RATE="${1:-1.0}"
TAG="${2:-_sweep}"
CFG="${3:-hangar.yaml}"   # was hardcoded to hangar.yaml -- a silent way to sweep the wrong config
BAGS_HOST="${BAGS_HOST:-$HOME/all_bag_files}"
SUMMARY="$(dirname "$HERE")/results/sweep${TAG}_rate${RATE}.txt"
[ -f "$(dirname "$HERE")/config/$CFG" ] || { echo "no config: $CFG"; exit 1; }
: > "$SUMMARY"
mapfile -t BAGS < <(cd "$BAGS_HOST" && ls *.bag | sort)
echo "sweep: ${#BAGS[@]} bags @ rate $RATE cfg=$CFG -> $SUMMARY"
i=0
for b in "${BAGS[@]}"; do
  i=$((i+1))
  echo "===== [$i/${#BAGS[@]}] $b ====="
  { echo "===== $b ====="; } >> "$SUMMARY"
  BAGS_HOST="$BAGS_HOST" MAP_CLEANUP_RADIUS=300.0 \
    "$HERE/run_bag.sh" "$b" "$CFG" "$TAG" "$RATE" "" true 2>&1 | tee -a "$SUMMARY" | tail -9
done
echo; echo "===== SWEEP COMPLETE -> $SUMMARY ====="
