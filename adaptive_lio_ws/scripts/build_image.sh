#!/usr/bin/env bash
# Build the Adaptive-LIO ROS1 Noetic image. Run from anywhere.
#   ./build_image.sh              # SLAM core only (bag replay)
#   ./build_image.sh --live       # also builds the Ouster ROS1 driver (live sensor)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # adaptive_lio_ws/
IMAGE="${IMAGE:-adaptive-lio:noetic}"
OUSTER=0
[ "${1:-}" = "--live" ] && OUSTER=1

echo "=== building $IMAGE (BUILD_OUSTER_DRIVER=$OUSTER) from $HERE ==="
docker build -f "$HERE/Dockerfile" -t "$IMAGE" \
  --build-arg BUILD_OUSTER_DRIVER="$OUSTER" \
  "$HERE"
echo "=== done: $IMAGE ==="
