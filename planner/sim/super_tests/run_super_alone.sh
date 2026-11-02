#!/usr/bin/env bash
# SUPER fsm_node alone (no SITL, no FC) on Shannon, in its own container (own roscore, --network none,
# --memory 8g, 4 cores), fed by a test script from this folder; fsm_node's RSS sampled every 5 s to rss.txt.
#
#   run_super_alone.sh <name> <image> <duration_s> <script args...>
#
# FEEDER (default stuck_from_rest.py): the test script, run as  <script> /map.pcd <script args...>
#   stuck_from_rest.py  rx ry rz gx gy gz [goal_every_s]   a static hover (planner frame) and one goal (the
#       2026-11-02 leak: 13.97 -32.54 1.53 14.95 -32.95 1.35 keeps PlanFromRest failing; goal_every_s 0.02 re-sends
#       the goal, which resets the from-rest backoff)
#   pillar_test.py  sx sy sz gx gy gz D [trig_v] [duration_s]   a perfect-tracking vehicle and a pillar appearing
#       D m ahead (faults case 5a, f25r): -29 -20 1.1 -29 -12 1.1 1.5 0.6 45
# MAP (mr4f_1102.pcd), CFG (super_closed_loop_mr4f.yaml), OUT_BASE (~/sim/superleak/runs), CPUSET (20-23),
# WRAP (e.g. heaptrack, a rosrun --prefix). Example (2026-11-02 before/after: l4-sim-l2:latest vs :leakfix):
#   CPUSET=20-23 ./run_super_alone.sh stuck_after l4-sim-l2:leakfix 300 13.97 -32.54 1.53 14.95 -32.95 1.35
set -uo pipefail
name=$1; img=$2; dur=$3; shift 3; args="$*"
HERE=$(cd "$(dirname "$0")" && pwd)
MAP=${MAP:-$HOME/sim/maps/massrobotics_4f/mr4f_1102.pcd}
CFG=${CFG:-$HOME/sim/mr4f/planner_sim/super_closed_loop_mr4f.yaml}
OUT=${OUT_BASE:-$HOME/sim/superleak/runs}/$name; mkdir -p $OUT
docker rm -f superleak-$name >/dev/null 2>&1
docker run -d --name superleak-$name --network none --cpus 4 --cpuset-cpus ${CPUSET:-20-23} --memory 8g --memory-swap 8g \
  -v $HERE:/repro:ro -v $(realpath $MAP):/map.pcd:ro -v $(realpath $CFG):/cfg.yaml:ro -v $OUT:/out \
  --entrypoint /entrypoint.sh $img sleep infinity >/dev/null
docker exec superleak-$name bash -c 'cp /cfg.yaml /super_ws/src/SUPER/super_planner/config/super_closed_loop.yaml'
docker exec -d superleak-$name bash -c 'source /opt/ros/noetic/setup.bash; export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311; roscore > /out/roscore.log 2>&1'
sleep 4
docker exec -d superleak-$name bash -c "source /opt/ros/noetic/setup.bash; export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311; source /super_ws/devel/setup.bash; cd /out; stdbuf -oL rosrun ${WRAP:+--prefix \"$WRAP\"} super_planner fsm_node _config_name:=super_closed_loop.yaml > /out/super.log 2>&1"
docker exec -d superleak-$name bash -c "source /opt/ros/noetic/setup.bash; export ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311; source /super_ws/devel/setup.bash; python3 -u /repro/${FEEDER:-stuck_from_rest.py} /map.pcd $args > /out/feeder.log 2>&1"
echo "t_s rss_kb fails" > $OUT/rss.txt
t0=$(date +%s)
while [ $(( $(date +%s) - t0 )) -lt $dur ]; do
  rss=$(docker exec superleak-$name bash -c 'ps -C fsm_node -o rss= | head -1' 2>/dev/null | tr -d ' ')
  f=$(grep -c "PlanFromRest failed" $OUT/super.log 2>/dev/null)
  echo "$(( $(date +%s) - t0 )) ${rss:-0} ${f:-0}" >> $OUT/rss.txt
  sleep 5
done
docker stats --no-stream superleak-$name >> $OUT/rss.txt
docker exec superleak-$name bash -c 'pkill -INT -f fsm_node; sleep 3; true'
docker rm -f superleak-$name >/dev/null
wc -l $OUT/super.log; cat $OUT/feeder.log; tail -4 $OUT/rss.txt
