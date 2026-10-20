#!/bin/bash
# Run the real drift_monitor.py, with the /drift_monitor params that config/fastlio_to_fc.launch
# resolves for slam_backend:=bievr, over a recorded BIEVR health stream (<prefix>_chain.bag from
# bievr_live_replay.sh). Records /slam/drift_risk to <prefix>_monitor.bag.
# Inside slam-system:bievr with --network none: there is no mavros, so every FC action it attempts
# (source switch, LAND, restart) fails on a missing service. That is the point: verdicts only.
# Usage: bievr_monitor_replay.sh <prefix> <params_bievr.yaml (roslaunch --dump-params output)>
set -u
PREFIX="$1"; PARAMS="$2"; PORT=11313
export ROS_MASTER_URI="http://localhost:${PORT}"; export ROS_HOSTNAME=localhost
set +u; source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash; set -u
roscore -p $PORT >"${PREFIX}_mon_roscore.log" 2>&1 & RC=$!; sleep 4
rosparam set /use_sim_time true
python3 - "$PARAMS" <<'PY'
import sys, yaml, rosparam
d = yaml.safe_load(open(sys.argv[1]))
for k, v in d.items():
    if k.startswith('/drift_monitor/'):
        rosparam.set_param_raw(k, v)
PY
rosrun fast_lio drift_monitor.py >"${PREFIX}_monitor.log" 2>&1 & MP=$!; sleep 3
rosbag record -O "${PREFIX}_monitor.bag" __name:=mon_rec /slam/drift_risk /slam/arm_interlock >/dev/null 2>&1 & sleep 2
rosbag play --clock --quiet "${PREFIX}_chain.bag" --topics /bievr_health /bievr_lio/odom
sleep 2; rosnode kill /mon_rec >/dev/null 2>&1; sleep 2; kill -INT $MP; sleep 2; kill -INT $RC; sleep 1
[ -f "${PREFIX}_monitor.bag.active" ] && rosbag reindex "${PREFIX}_monitor.bag.active" >/dev/null 2>&1 && mv "${PREFIX}_monitor.bag.active" "${PREFIX}_monitor.bag"
python3 - "${PREFIX}_monitor.bag" "${PREFIX}_chain.bag" <<'PY'
import sys, rosbag, collections
t0 = next(rosbag.Bag(sys.argv[2]).read_messages(topics=['/bievr_health']))[2].to_sec()
lv = collections.Counter(); first = {}
prev = None
for _, m, t in rosbag.Bag(sys.argv[1]).read_messages(topics=['/slam/drift_risk']):
    level = m.data.split()[0].split(':')[0].strip(',;|') if m.data else '?'
    lv[level] += 1
    first.setdefault(level, (t.to_sec() - t0, m.data[:110]))
    if level != prev: print(f"  t={t.to_sec()-t0:7.2f}s  -> {m.data[:110]}"); prev = level
print("  counts:", dict(lv))
PY
grep -E 'CRITICAL|-> ' "${PREFIX}_monitor.log" | head -8
