#!/usr/bin/env bash
# SC-PGO (the stack's FAST_LIO_SLAM/SC-PGO, alaserPGO) alone on a recording's LIVE FAST-LIO odometry
# (live_to_pgo_bag.py output: /Odometry + /cloud_registered_body). Offline only; runs INSIDE slam-pgo:tuned,
# mounted read-only, like revolute-gui/deploy/pgo/run_pgo.sh, whose SC-PGO settings it keeps.
#
#   run_pgo_live.sh <bag> <out-dir under /out> [rate] [sc_max_radius] [keyframe_meter_gap] [sc_dist_thres] [max_odom_gap]
set -uo pipefail
BAG="${1:?bag}"; OUT="${2:?out dir}"; RATE="${3:-0.5}"; SC_MAX_RADIUS="${4:-20.0}"; KF_GAP="${5:-0.5}"
SC_DIST_THRES="${6:-0.15}"; MAX_ODOM_GAP="${7:-3.0}"
case "$OUT" in /out|/out/*) ;; *) echo "PGO-ERROR refusing $OUT" >&2; exit 2 ;; esac
mkdir -p "$OUT/Scans"; SAVE_DIR="${OUT%/}/"
source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash
roscore >/dev/null 2>&1 &
for _ in $(seq 1 30); do rosparam list >/dev/null 2>&1 && break; sleep 1; done
rosparam set /save_directory "$SAVE_DIR"
rosparam set /keyframe_meter_gap "$KF_GAP"
rosparam set /keyframe_deg_gap 10.0
rosparam set /sc_dist_thres "$SC_DIST_THRES"
rosparam set /loop_icp_max_corr_dist 5.0
rosparam set /loop_fitness_thres 0.15
rosparam set /loop_max_odom_gap "$MAX_ODOM_GAP"
rosparam set /sc_max_radius "$SC_MAX_RADIUS"
rosparam set /mapviz_filter_size 0.05
echo "PGO-START (live odometry) bag=$BAG rate=$RATE sc_max_radius=$SC_MAX_RADIUS keyframe_gap=$KF_GAP sc_dist_thres=$SC_DIST_THRES max_odom_gap=$MAX_ODOM_GAP"
rosrun aloam_velodyne alaserPGO /aft_mapped_to_init:=/Odometry /velodyne_cloud_registered_local:=/cloud_registered_body &
PGO=$!
sleep 3
rosbag play --rate "$RATE" "$BAG"
echo "PGO-PLAYBACK-DONE"
sleep 10
kill -INT "$PGO" 2>/dev/null; sleep 3; kill -INT %1 2>/dev/null; wait "$PGO" 2>/dev/null
scans=$(ls "$OUT/Scans" | wc -l); poses=$(wc -l < "$SAVE_DIR/optimized_poses.txt" 2>/dev/null || echo 0)
echo "PGO-SUMMARY keyframes=$scans optimized_poses=$poses odom_poses=$(wc -l < "$SAVE_DIR/odom_poses.txt" 2>/dev/null || echo 0)"
[ "$scans" -ge 2 ] && [ "$poses" -ge 2 ] && echo PGO-OK || echo "PGO-ERROR too few keyframes"
