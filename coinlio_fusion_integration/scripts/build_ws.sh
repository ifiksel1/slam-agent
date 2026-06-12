#!/bin/bash
# colcon build inside the coinlio_fusion dev container.
#   ./build_ws.sh                 # build all (ellipselio + ouster_ros)
#   ./build_ws.sh ellipselio      # build just one package
#   PKG=ellipselio ./build_ws.sh  # same via env
set -uo pipefail
CTR=coinlio_fusion
PKG="${1:-${PKG:-}}"
SEL=""
[[ -n "$PKG" ]] && SEL="--packages-select $PKG"

docker exec "$CTR" bash -lc "
  set -e
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash
  cd /root/ros2_ws
  colcon build --symlink-install $SEL \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
"
rc=$?
echo "[build_ws] colcon exit=$rc"
exit $rc
