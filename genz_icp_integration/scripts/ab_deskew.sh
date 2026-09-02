#!/bin/bash
# A/B: deskew off (hangar_rt) vs on (hangar_rt_deskew), rate 1.0.
# Bags chosen = the 3 with closed_loop:True in icp_gt_v2 (unimpeached ground truth),
# plus FRONT which is our most-characterised bag.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS=(737_MAX_8_LIO_SAM_3_2025-05-15-02-11-55.bag
      cmem3qaqb005q02mnaysmacre_2025-08-22-02-03-32.bag
      cmem9gs5j006202mngllj1it1_2025-08-22-03-18-39.bag
      737_700_FRONT_2025-05-17-04-39-04.bag)
for b in "${BAGS[@]}"; do
  echo "########## $b :: deskew ON ##########"
  MAP_CLEANUP_RADIUS=300.0 "$HERE/run_bag.sh" "$b" hangar_rt_deskew.yaml "_dsk" 1.0 "" true 2>&1 | tail -6
done
