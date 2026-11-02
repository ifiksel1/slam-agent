#!/usr/bin/env bash
# After build_mr4f_live.sh: the SC-PGO report (loops, correction) and before/after top-down views on Shannon.
#   before: the same keyframes on the live FAST-LIO odometry poses; after: SC-PGO's optimized poses (both uncleaned)
set -uo pipefail
# Same env as build_mr4f_live.sh (REC, STAMP, MAPNAME, W) for another recording; output in $M/pgo_compare_<MAPNAME>
# (the 10-30 map keeps $M/pgo_compare).
STAMP=${STAMP:-10_30_2026_22_01_05}
B=${REC:-$HOME/sim/recordings/massrobotics_20261030/${STAMP}_flight}
W=${W:-$HOME/sim/mr4f/live}; B0=$B/${STAMP}_0.bag; M=~/sim/maps/massrobotics_4f; T=~/sim/mr4f/map
MAPNAME=${MAPNAME:-mr4f_1030}
PC=$M/pgo_compare; [ $MAPNAME = mr4f_1030 ] || PC=$M/pgo_compare_$MAPNAME
PY="taskset -c 20-27 nice -n 5 $HOME/sim/venv/bin/python -I"
D="Mass Robotics 4th floor ${STAMP}"
mkdir -p $PC
$PY $T/build_sim_map.py $W/out $B0 $PC before_odom "$D, FAST-LIO odometry poses (before PGO)" \
  --poses odom --no_clean > $PC/before_odom.log 2>&1
$PY $T/build_sim_map.py $W/out $B0 $PC after_pgo "$D, SC-PGO optimized poses (after PGO)" \
  --poses optimized --no_clean > $PC/after_pgo.log 2>&1
rm -f $PC/*_raw.pcd
$PY $T/pgo_report.py $W/out $W/pgo.log $PC/pgo_paths.png $M/$MAPNAME.meta.json | tee $PC/pgo_report.txt
echo POST-DONE
