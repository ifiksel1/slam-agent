#!/bin/bash
# Build the EllipseLIO workspace (ellipselio + ouster-ros) inside the ellipselio:humble image.
# Mounts the host workspace so build/ + install/ persist. Run once after `docker build`.
#
# ARM64 / Orin NX 8GB notes (from frameworks_ellipse_super.md):
#   - EllipseLIO forces -Ofast -fopenmp -fPIC, C++17. PCL_NO_PRECOMPILE makes builds
#     slow + RAM-hungry -> cap parallelism. We use -j4 (6-core module, 8GB RAM).
#   - If the EKF diverges on aarch64 ONLY, rebuild ellipselio with -O3 (override -Ofast).
set -e
CTR=ellipselio_build
docker rm -f "$CTR" 2>/dev/null || true

docker run --rm --name "$CTR" \
  -v "$HOME/ellipselio_ws:/root/ros2_ws" \
  ellipselio:humble bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    cd /root/ros2_ws
    echo "=== rosdep (best-effort) ==="
    rosdep update --rosdistro humble 2>/dev/null || true
    echo "=== colcon build (ouster_ros + ellipselio) -j4 ==="
    # --packages-up-to pulls in-workspace deps (ouster_ros needs ouster_sensor_msgs + vendored sophus).
    MAKEFLAGS="-j4" colcon build \
      --packages-up-to ouster_ros ellipselio \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 \
      --symlink-install --executor sequential
    echo "=== build complete ==="
    ls install/
  '
echo "Workspace built at ~/ellipselio_ws/install (persisted on host)."
