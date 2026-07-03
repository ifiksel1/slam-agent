#!/usr/bin/env bash
# Adaptive-LIO hardcodes its published TF/odometry frames as "world", "map", "base_link"
# (src/apps/main_ada.cpp). On the Dragunfly rig those exact names are already owned by MAVROS
# ("map") and robot_state_publisher ("base_link"), so a raw run makes Adaptive-LIO's TF fight
# the flight stack. This rewrites ONLY the frame-string LITERALS into an "alio_*" namespace:
#     world      -> alio_world
#     map        -> alio_odom     (Adaptive-LIO's "map" is really its odom/world frame)
#     base_link  -> alio_body     (its body frame ~= os_imu, upside-down on this rig)
# master.launch then bridges alio_odom/alio_body into the real map/base_link with static TFs.
#
# It deliberately does NOT touch the pose_pub_func routing keys `topic_name == "world"` /
# `== "laser"` (those are internal dispatch strings, not frames). Idempotent.
#
# Usage: namespace_frames.sh <path-to-adaptive_lio-source-root>
set -euo pipefail
SRC="${1:?usage: namespace_frames.sh <adaptive_lio_src_root>}"
F="$SRC/src/apps/main_ada.cpp"
[ -f "$F" ] || { echo "ERROR: $F not found"; exit 1; }

if grep -q 'alio_odom' "$F"; then
  echo "namespace_frames: already applied to $F — skipping"
  exit 0
fi

sed -i \
  -e 's/header\.frame_id = "map";/header.frame_id = "alio_odom";/g' \
  -e 's/header\.frame_id = "\/map";/header.frame_id = "alio_odom";/g' \
  -e 's/child_frame_id = "base_link";/child_frame_id = "alio_body";/g' \
  -e 's/"map", "base_link"))/"alio_odom", "alio_body"))/g' \
  -e 's/"world", "map"))/"alio_world", "alio_odom"))/g' \
  "$F"

# Verify all six sites landed (2 TF sends + odom header + odom child + cloud header + path header).
n=$(grep -cE 'alio_odom|alio_body|alio_world' "$F" || true)
echo "namespace_frames: applied ($n alio_* frame references now in main_ada.cpp)"
[ "$n" -ge 6 ] || { echo "ERROR: expected >=6 alio_* refs, got $n — upstream frame strings may have changed"; exit 1; }
