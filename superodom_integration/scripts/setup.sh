#!/bin/bash
# Turnkey SuperOdom setup — takes a FRESH slam-agent clone to a runnable SuperOdom stack
# with no manual workspace prep. Idempotent: safe to re-run.
#
# Does everything the README "prereqs" used to ask you to do by hand:
#   1. create the host workspace (mounted into the container at /root/ros2_ws)
#   2. clone the EXACT sources+branches that work on this rig (pinned below)
#   3. stage the integration configs where the SuperOdom launch + driver look
#   4. build the Docker image (source-builds GTSAM/Sophus/Ceres/Livox-SDK2 — slow first time)
#   5. unified colcon build of the workspace
#
# After this, run_superodom_bench.sh works directly.
#
# Override anything via env, e.g.:
#   SUPERODOM_WS=~/my_ws SUPERODOM_SRC_URL=https://github.com/you/SuperOdom.git ./setup.sh
#
# All default sources are PUBLIC (superxslam/SuperOdom@ros2, Livox-SDK, teamspatzenhirn,
# ouster-lidar) — no private-repo access needed.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(cd "$HERE/.." && pwd)"

# ---- knobs (env-overridable; defaults = the validated rig recipe) -------------------
WS="${SUPERODOM_WS:-$HOME/superodom_ws}"
SUPERODOM_SRC_URL="${SUPERODOM_SRC_URL:-https://github.com/superxslam/SuperOdom.git}"
SUPERODOM_BRANCH="${SUPERODOM_BRANCH:-ros2}"
OUSTER_ROS_URL="${OUSTER_ROS_URL:-https://github.com/ouster-lidar/ouster-ros.git}"
OUSTER_ROS_BRANCH="${OUSTER_ROS_BRANCH:-ros2}"
LIVOX_URL="${LIVOX_URL:-https://github.com/Livox-SDK/livox_ros_driver2.git}"
LIVOX_BRANCH="${LIVOX_BRANCH:-master}"
OVERLAY_URL="${OVERLAY_URL:-https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git}"
OVERLAY_BRANCH="${OVERLAY_BRANCH:-main}"
IMAGE="${SUPERODOM_IMAGE:-superodom:humble}"
DO_BUILD="${SUPERODOM_BUILD:-1}"   # set 0 to skip docker build + colcon (clone/stage only)

echo "=== SuperOdom setup ==="
echo "  workspace : $WS"
echo "  SuperOdom : $SUPERODOM_SRC_URL @ $SUPERODOM_BRANCH"
echo "  ouster-ros: $OUSTER_ROS_URL @ $OUSTER_ROS_BRANCH"
echo "  livox     : $LIVOX_URL @ $LIVOX_BRANCH"
echo "  overlay   : $OVERLAY_URL @ $OVERLAY_BRANCH"
echo "  image     : $IMAGE  (build=$DO_BUILD)"

clone_or_update() {
  local url="$1" branch="$2" name="$3" dest="$WS/src/$3"
  if [ -d "$dest/.git" ]; then
    echo "[=] $name present — fetch + checkout $branch"
    git -C "$dest" fetch --depth 1 origin "$branch"
    git -C "$dest" checkout -B "$branch" FETCH_HEAD
  else
    echo "[+] cloning $name ($branch)"
    git clone --depth 1 --branch "$branch" "$url" "$dest"
  fi
}

mkdir -p "$WS/src"
clone_or_update "$SUPERODOM_SRC_URL" "$SUPERODOM_BRANCH" SuperOdom
clone_or_update "$OUSTER_ROS_URL"    "$OUSTER_ROS_BRANCH" ouster-ros
clone_or_update "$LIVOX_URL"         "$LIVOX_BRANCH"      livox_ros_driver2
clone_or_update "$OVERLAY_URL"       "$OVERLAY_BRANCH"    rviz_2d_overlay_plugins

# ---- stage integration configs ------------------------------------------------------
# SuperOdom params + calibration live inside the package; the Ouster driver params sit at
# the TOP of src/ (run_superodom_bench.sh passes src/ouster_os1_64_driver.yaml).
echo "[+] staging configs (edit ouster_os1_64_driver.yaml for YOUR sensor/host IP + lidar_mode;"
echo "    CALIBRATE os1_64_calibration.yaml before flight)"
mkdir -p "$WS/src/SuperOdom/super_odometry/config/ouster"
cp "$INTEG/config/os1_64.yaml"                  "$WS/src/SuperOdom/super_odometry/config/os1_64.yaml"
cp "$INTEG/config/ouster/os1_64_calibration.yaml" "$WS/src/SuperOdom/super_odometry/config/ouster/os1_64_calibration.yaml"
cp "$INTEG/config/ouster_os1_64_driver.yaml"    "$WS/src/ouster_os1_64_driver.yaml"

if [ "$DO_BUILD" != "1" ]; then
  echo "=== sources + configs staged (build skipped). Build with: SUPERODOM_BUILD=1 $0 ==="
  exit 0
fi

# ---- build image --------------------------------------------------------------------
echo "=== docker build $IMAGE  (source-builds GTSAM/Sophus/Ceres — slow on first run) ==="
docker build -t "$IMAGE" "$INTEG"

# ---- unified colcon build (recipe from README "Build notes") ------------------------
# 8GB/6-core Orin NX: ONE unified build (do NOT --packages-skip livox — super_odometry
# build-depends on it). Cap parallelism; -DROS_EDITION=ROS2 -DDISTRO_ROS=humble.
echo "=== colcon build workspace (unified, -j3, parallel-workers 1) ==="
CTR=superodom_build
docker rm -f "$CTR" 2>/dev/null || true
docker run --rm --name "$CTR" \
  -v "$WS:/root/ros2_ws" \
  "$IMAGE" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    cd /root/ros2_ws
    rosdep update --rosdistro humble 2>/dev/null || true
    MAKEFLAGS="-j3" colcon build \
      --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
      --symlink-install
    echo "=== build complete ==="
    ls install/
  '

echo
echo "=== SuperOdom ready ==="
echo "  Edit IPs:  $WS/src/ouster_os1_64_driver.yaml  (sensor_hostname / udp_dest / lidar_mode)"
echo "  Calibrate: $WS/src/SuperOdom/super_odometry/config/ouster/os1_64_calibration.yaml"
echo "  Live:      $HERE/run_superodom_bench.sh"
