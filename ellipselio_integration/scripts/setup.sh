#!/bin/bash
# Turnkey EllipseLIO setup — takes a FRESH slam-agent clone to a runnable EllipseLIO
# stack with no manual workspace prep. Idempotent: safe to re-run (clones are pulled,
# configs re-staged, patch re-checked).
#
# Does everything the README "prereqs" used to ask you to do by hand:
#   1. create the host workspace (mounted into the container at /root/ros2_ws)
#   2. clone the EXACT sources+branches that work on this rig (pinned below)
#   3. apply the container_executable launch patch (needed by run_bag_foxglove.sh)
#   4. stage the integration configs into the workspace where the launch files look
#   5. build the Docker image, then colcon-build the workspace
#
# After this, run_ellipselio.sh / run_bag.sh / run_bag_foxglove.sh work directly.
#
# Override anything via env, e.g.:
#   ELLIPSELIO_WS=~/my_ws ELLIPSELIO_SRC_URL=https://github.com/you/ellipselio.git ./setup.sh
#
# NOTE on source access: the working EllipseLIO branch (container_executable + the
# leading-slash frame-name fix) lives on a FORK, not on v4rl-ucy upstream. Default URL
# points at that fork; set ELLIPSELIO_SRC_URL if you mirror it elsewhere. Cloning a
# private fork over HTTPS needs `gh auth setup-git` (or swap to an SSH URL).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEG="$(cd "$HERE/.." && pwd)"

# ---- knobs (env-overridable; defaults = the validated rig recipe) -------------------
WS="${ELLIPSELIO_WS:-$HOME/ellipselio_ws}"
ELLIPSELIO_SRC_URL="${ELLIPSELIO_SRC_URL:-https://github.com/ifiksel1/ellipselio.git}"
ELLIPSELIO_BRANCH="${ELLIPSELIO_BRANCH:-fix/frame-names-no-leading-slash}"
OUSTER_ROS_URL="${OUSTER_ROS_URL:-https://github.com/ouster-lidar/ouster-ros.git}"
OUSTER_ROS_BRANCH="${OUSTER_ROS_BRANCH:-ros2}"
IMAGE="${ELLIPSELIO_IMAGE:-ellipselio:humble}"
DO_BUILD="${ELLIPSELIO_BUILD:-1}"   # set 0 to skip docker build + colcon (clone/stage only)

echo "=== EllipseLIO setup ==="
echo "  workspace : $WS"
echo "  ellipselio: $ELLIPSELIO_SRC_URL @ $ELLIPSELIO_BRANCH"
echo "  ouster-ros: $OUSTER_ROS_URL @ $OUSTER_ROS_BRANCH"
echo "  image     : $IMAGE  (build=$DO_BUILD)"

# clone-or-update a repo at a branch into $WS/src/<name>
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
clone_or_update "$ELLIPSELIO_SRC_URL" "$ELLIPSELIO_BRANCH" ellipselio
clone_or_update "$OUSTER_ROS_URL"     "$OUSTER_ROS_BRANCH" ouster-ros

# ---- apply the container_executable launch patch (idempotent) -----------------------
# run_bag_foxglove.sh passes container_executable:=... — without this arg the launch
# aborts on an unknown argument. The patch is shipped in the repo (not on the fork).
LAUNCH="$WS/src/ellipselio/launch/ellipselio_standalone.launch.py"
if grep -q "container_executable" "$LAUNCH" 2>/dev/null; then
  echo "[=] launch patch already applied"
else
  echo "[+] applying container_executable launch patch"
  git -C "$WS/src/ellipselio" apply "$INTEG/patches/0001-container-executable-launch-arg.patch"
fi

# ---- stage integration configs into the workspace ----------------------------------
# os1_64_ouster.yaml lives where the EllipseLIO launch looks (src/ellipselio/config/).
# The Ouster driver params sit at the TOP of src/ (run scripts pass src/ouster_os1_64_driver.yaml).
echo "[+] staging configs (edit ouster_os1_64_driver.yaml for YOUR sensor/host IP + lidar_mode)"
cp "$INTEG/config/os1_64_ouster.yaml"        "$WS/src/ellipselio/config/os1_64_ouster.yaml"
cp "$INTEG/config/ouster_os1_64_driver.yaml" "$WS/src/ouster_os1_64_driver.yaml"

if [ "$DO_BUILD" != "1" ]; then
  echo "=== sources + configs staged (build skipped). Build with: ELLIPSELIO_BUILD=1 $0 ==="
  exit 0
fi

# ---- build image + workspace --------------------------------------------------------
echo "=== docker build $IMAGE ==="
docker build -t "$IMAGE" "$INTEG"

echo "=== colcon build workspace ==="
ELLIPSELIO_WS="$WS" ELLIPSELIO_IMAGE="$IMAGE" "$HERE/build_ws.sh"

echo
echo "=== EllipseLIO ready ==="
echo "  Edit IPs:  $WS/src/ouster_os1_64_driver.yaml  (sensor_hostname / udp_dest / lidar_mode)"
echo "  Live:      $HERE/run_ellipselio.sh"
echo "  Bag eval:  $HERE/run_bag.sh <bag_dir_name>"
echo "  Watch:     $HERE/run_bag_foxglove.sh <bag_dir_name>"
