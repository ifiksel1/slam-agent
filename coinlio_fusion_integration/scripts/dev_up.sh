#!/bin/bash
# Build (if needed) and start the coinlio_fusion DEV container for the
# COIN-LIO photometric -> EllipseLIO IKFoM port.
#
# Isolated from the production `ellipselio` container: separate image, separate
# container name, separate workspace mount (~/coinlio_fusion_ws). Does NOT stop or
# touch the running `ellipselio` container.
#
#   ./dev_up.sh            # build image if missing + (re)create & start container
#   ./dev_up.sh --rebuild  # force rebuild the image first
set -euo pipefail

IMAGE=coinlio-fusion:humble
CTR=coinlio_fusion
WS="$HOME/coinlio_fusion_ws"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--rebuild" ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[dev_up] building $IMAGE from $HERE/.."
  docker build -t "$IMAGE" -f "$HERE/../Dockerfile" "$HERE/.."
fi

if [[ ! -d "$WS/src/ellipselio" ]]; then
  echo "[dev_up] ERROR: $WS/src/ellipselio missing — create the dev workspace first." >&2
  exit 1
fi

echo "[dev_up] (re)creating container $CTR (mount $WS -> /root/ros2_ws)"
docker rm -f "$CTR" 2>/dev/null || true
# host net + privileged so this same container can later run the Ouster driver +
# foxglove_bridge for GATE-0 / live testing, exactly like run_ellipselio.sh.
docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v "$WS:/root/ros2_ws" \
  -v /home/dev/slam-gpu/bags:/root/bags:ro \
  "$IMAGE" sleep infinity

echo "[dev_up] up. Useful:"
echo "  docker exec -it $CTR bash                       # shell in"
echo "  bash $HERE/build_ws.sh                           # colcon build (baseline or after edits)"
echo "  docker exec $CTR bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg list | grep ellipselio'"
