#!/usr/bin/env bash
# Mass Robotics 4F sim map from the LIVE FAST-LIO odometry + SC-PGO (Shannon, detached):
#   live_to_pgo_bag.py (live /Odometry + scans in the IMU frame) -> SC-PGO alone (run_pgo_live.sh in slam-pgo:tuned)
#   -> build_sim_map.py. Replaces build_mr4f.sh's FAST-LIO replay, which started cold and drifted (see live_to_pgo_bag.py).
#
#   nohup build_mr4f_live.sh > ~/sim/mr4f/live/build.log 2>&1 &      (finished steps are skipped on a re-run)
#
# Map settings (BUILD_ARGS, chosen 2026-10-30 on this recording):
#   --ground_level: the live odometry's slow vertical drift (up to 1.06 m at 440-474 s) left the floor warped by
#     0.69 m (p5-p95 about a plane); levelling each keyframe on the floor it sees brings that to 0.18 m, while the
#     floor seen on different passes still agrees to 1-2 cm (floor_consistency.py).
#   far low points (< 0.6 m, seen from > 6 m) dropped and a flat floor at z = 0 filled in (grazing-floor smear).
#   carving --span_max 4 --free_ratio 3 --free_min 4: the defaults also thinned walls (glass partitions).
#
# Another recording (env): REC (folder of split bags), STAMP (bag name prefix), MAPNAME, DESC, W (work dir), e.g. the
# 2026-11-02 re-drive:  REC=~/sim/recordings/massrobotics_20261102/11_02_2026_15_56_13_flight STAMP=11_02_2026_15_56_13 \
#   MAPNAME=mr4f_1102 W=~/sim/mr4f/live_1102 DESC="..." build_mr4f_live.sh
set -uo pipefail
STAMP=${STAMP:-10_30_2026_22_01_05}
B=${REC:-$HOME/sim/recordings/massrobotics_20261030/${STAMP}_flight}
W=${W:-$HOME/sim/mr4f/live}; M=~/sim/maps/massrobotics_4f; T=~/sim/mr4f/map
MAPNAME=${MAPNAME:-mr4f_1030}
DESC=${DESC:-"Mass Robotics 4th floor, drive+flights 2026-10-30 22:01:05 UTC (live FAST-LIO + SC-PGO); z up, floor at 0 m, 6 cm grid, cleaned"}
PY="taskset -c 20-27 nice -n 5 $HOME/sim/venv/bin/python -I"
BAGS=$(ls $B/${STAMP}_?.bag | sort)
mkdir -p $W/out $M
step() { echo "=== $(date +%T) $*"; }
if [ ! -s $W/pgo_in.bag ]; then
  step "live_to_pgo_bag"; $PY $T/live_to_pgo_bag.py $W/pgo_in.bag $BAGS || { echo FAILED; exit 1; }
fi
if ! grep -q PGO-OK $W/pgo.log 2>/dev/null; then
  step "SC-PGO on the live odometry (slam-pgo:tuned, rate ${RATE:-0.5})"
  docker run --rm -v $W:/out alpine sh -c 'rm -rf /out/out' >/dev/null 2>&1; mkdir -p $W/out
  docker rm -f pgo-$MAPNAME >/dev/null 2>&1
  docker run --rm --name pgo-$MAPNAME --network none -e ROS_HOSTNAME=localhost -e ROS_MASTER_URI=http://localhost:11311 \
    --cpus 4 --cpu-shares 512 \
    --mount type=bind,src=$W/pgo_in.bag,dst=/bag.bag,readonly --mount type=bind,src=$W/out,dst=/out \
    --mount type=bind,src=$T/run_pgo_live.sh,dst=/run_pgo_live.sh,readonly \
    slam-pgo:tuned bash /run_pgo_live.sh /bag.bag /out ${RATE:-0.5} 20.0 0.5 0.15 3.0 > $W/pgo.log 2>&1
  tr "\r" "\n" < $W/pgo.log | grep -E "PGO-SUMMARY|PGO-ERROR|PGO-OK" | head -3
  echo "loops accepted: $(tr '\r' '\n' < $W/pgo.log | grep -c 'Add this SC loop')"
fi
step "build_sim_map.py"
$PY $T/build_sim_map.py $W/out $(echo $BAGS | cut -d" " -f1) $M $MAPNAME "$DESC" \
  --workers 8 ${BUILD_ARGS---ground_level --span_max 4 --free_ratio 3 --free_min 4} 2>&1 | tee $W/build_map.log
step "MR4F-LIVE-BUILD-DONE"
