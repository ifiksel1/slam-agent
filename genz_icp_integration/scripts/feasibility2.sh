#!/bin/bash
# Step 1b: push past the iters=15 operating point toward 20 Hz real-time.
# Two axes: iteration cap (10, 5) and correspondence-search cost (fewer points / coarser voxel).
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG=737_700_FRONT_2025-05-17-04-39-04.bag
for cfg in hangar_it10 hangar_it5 hangar_it15_p2000 hangar_it15_v05; do
  echo "########## $cfg @ rate 1.0 ##########"
  MAP_CLEANUP_RADIUS=300.0 "$HERE/run_bag.sh" "$BAG" "$cfg.yaml" "_feas_$cfg" 1.0 "" true 2>&1 | tail -8
done
