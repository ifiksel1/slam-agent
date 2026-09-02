#!/bin/bash
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for cfg in hangar_it10_v05 hangar_it5_v05; do
  echo "########## $cfg @ rate 1.0 ##########"
  MAP_CLEANUP_RADIUS=300.0 "$HERE/run_bag.sh" 737_700_FRONT_2025-05-17-04-39-04.bag "$cfg.yaml" "_feas_$cfg" 1.0 "" true 2>&1 | tail -8
done
