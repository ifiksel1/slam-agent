#!/bin/bash
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for i in 1 2 3; do
  echo "########## RT determinism run $i ##########"
  MAP_CLEANUP_RADIUS=300.0 "$HERE/run_bag.sh" 737_700_FRONT_2025-05-17-04-39-04.bag \
    hangar_rt.yaml "_det$i" 1.0 "" true 2>&1 | tail -7
done
