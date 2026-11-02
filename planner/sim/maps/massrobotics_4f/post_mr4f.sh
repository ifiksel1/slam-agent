#!/usr/bin/env bash
# After build_mr4f_live.sh: the SC-PGO report (loops, correction) and before/after top-down views on Shannon.
#   before: the same keyframes on the live FAST-LIO odometry poses; after: SC-PGO's optimized poses (both uncleaned)
set -uo pipefail
W=~/sim/mr4f/live; B0=~/sim/recordings/massrobotics_20261030/10_30_2026_22_01_05_flight/10_30_2026_22_01_05_0.bag; M=~/sim/maps/massrobotics_4f; T=~/sim/mr4f/map
PY="taskset -c 20-27 nice -n 5 $HOME/sim/venv/bin/python -I"
D="Mass Robotics 4th floor 2026-10-30"
mkdir -p $M/pgo_compare
$PY $T/build_sim_map.py $W/out $B0 $M/pgo_compare before_odom "$D, FAST-LIO odometry poses (before PGO)" \
  --poses odom --no_clean > $M/pgo_compare/before_odom.log 2>&1
$PY $T/build_sim_map.py $W/out $B0 $M/pgo_compare after_pgo "$D, SC-PGO optimized poses (after PGO)" \
  --poses optimized --no_clean > $M/pgo_compare/after_pgo.log 2>&1
rm -f $M/pgo_compare/*_raw.pcd
$PY $T/pgo_report.py $W/out $W/pgo.log $M/pgo_compare/pgo_paths.png $M/mr4f_1030.meta.json | tee $M/pgo_compare/pgo_report.txt
echo POST-DONE
