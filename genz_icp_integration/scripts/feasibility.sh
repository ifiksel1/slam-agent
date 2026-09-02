#!/bin/bash
# Real-time feasibility: does an operating point exist where GenZ-ICP sustains the sensor
# rate on this Orin NX? Sweeps max_num_iterations (the suspected bottleneck -- stock 100 is
# very high and only binds in degenerate scenes, i.e. exactly the hangar) at replay rate 1.0
# = a live 20 Hz OS1. Reports THROUGHPUT (wall-clock) and accuracy together; either alone
# is misleading.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG=737_700_FRONT_2025-05-17-04-39-04.bag
for cfg in hangar_it30 hangar_it15; do
  echo "########## $cfg @ rate 1.0 ##########"
  MAP_CLEANUP_RADIUS=300.0 "$HERE/run_bag.sh" "$BAG" "$cfg.yaml" "_feas_$cfg" 1.0 "" true 2>&1 | tail -9
done
