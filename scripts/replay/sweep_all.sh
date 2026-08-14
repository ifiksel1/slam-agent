#!/bin/bash
# Confined-space FAST-LIO param sweep: 7 configs x 5 newest non-empty flight bags = 35 replays.
# S0 = TRUE live baseline (matches config/fastlio_to_fc.launch effective values).
# Each subsequent step keeps MORE points / MORE compute. Constants: blind 0.4, det_range 100, cube 300.
# Resumable: skips runs whose output bag already exists. Sequential (shared isolated port + fair timing).
set -u
WORK="$(cd "$(dirname "$0")" && pwd)"
OUTROOT="/mnt/usb/replay_sweep"
CONST="blind:=0.4 det_range:=100 cube_side_length:=300"
mkdir -p "$OUTROOT"

# config_id : escalating args (surf/map, point_filter_num, max_iteration)
declare -A CFG
CFG[S0_baseline]="filter_size_surf:=0.35 filter_size_map:=0.35 point_filter_num:=3 max_iteration:=3"
CFG[S1]="filter_size_surf:=0.30 filter_size_map:=0.30 point_filter_num:=3 max_iteration:=3"
CFG[S2]="filter_size_surf:=0.28 filter_size_map:=0.28 point_filter_num:=2 max_iteration:=3"
CFG[S3]="filter_size_surf:=0.25 filter_size_map:=0.25 point_filter_num:=2 max_iteration:=4"
CFG[S4]="filter_size_surf:=0.22 filter_size_map:=0.22 point_filter_num:=2 max_iteration:=4"
CFG[S5]="filter_size_surf:=0.20 filter_size_map:=0.20 point_filter_num:=1 max_iteration:=4"
CFG[S6]="filter_size_surf:=0.15 filter_size_map:=0.15 point_filter_num:=1 max_iteration:=4"
ORDER="S0_baseline S1 S2 S3 S4 S5 S6"

# 5 newest non-empty flight bags
mapfile -t BAGS < <(for d in $(ls -1dt /mnt/usb/*_flight/ 2>/dev/null); do
  b=$(ls -S "$d"*.bag 2>/dev/null | head -1); [ -z "$b" ] && continue
  [ "$(stat -c %s "$b")" -lt 50000000 ] && continue
  echo "$b"; done | head -5)

echo "=== SWEEP: ${#BAGS[@]} bags x $(echo $ORDER | wc -w) configs ==="
printf '%s\n' "${BAGS[@]}"
TOTAL=0; DONE=0
for cfg in $ORDER; do for b in "${BAGS[@]}"; do TOTAL=$((TOTAL+1)); done; done

for cfg in $ORDER; do
  mkdir -p "$OUTROOT/$cfg"
  for b in "${BAGS[@]}"; do
    tag=$(basename "$b" .bag)
    out="$OUTROOT/$cfg/${tag}.bag"
    DONE=$((DONE+1))
    if [ -f "$out" ]; then echo "[$DONE/$TOTAL] skip (exists): $cfg/$tag"; continue; fi
    echo "[$DONE/$TOTAL] RUN $cfg <- $tag"
    bash "$WORK/sweep_run_one.sh" "$b" "$out" "${CFG[$cfg]} $CONST" 1.0 2>&1 | tail -2
  done
done
echo "=== SWEEP COMPLETE -> $OUTROOT ==="
