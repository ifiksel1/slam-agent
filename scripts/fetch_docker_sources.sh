#!/bin/bash
# Vendor the ROS source packages needed by the Docker image into ./docker_src/.
#
# Why: the Dockerfile COPYs these in instead of cloning inside the build, because
# RevoluteRobotics/FAST_LIO_SLAM is PRIVATE and the build has no git credentials.
# Cloning here (on the host) uses your existing git/gh credentials.
#
# Usage:
#   gh auth login           # or have HTTPS git credentials for the private repo
#   gh auth setup-git       # makes git use the gh token for github.com
#   scripts/fetch_docker_sources.sh
#   docker build -t slam-system:latest .
#
# docker_src/ is .gitignored and excluded (histories/media) via .dockerignore.

set -euo pipefail

# Repo root = parent of this script's dir
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
DEST="$ROOT/docker_src"

# repo dir name -> clone URL  (FAST_LIO_SLAM bundles fast_lio + aloam_velodyne + hesai_ros_driver)
REPOS=(
    "FAST_LIO_SLAM|https://github.com/RevoluteRobotics/FAST_LIO_SLAM.git"
    "livox_ros_driver|https://github.com/Livox-SDK/livox_ros_driver.git"
    "vision_to_mavros|https://github.com/thien94/vision_to_mavros.git"
)

mkdir -p "$DEST"
for entry in "${REPOS[@]}"; do
    name="${entry%%|*}"
    url="${entry##*|}"
    target="$DEST/$name"
    if [[ -d "$target/.git" ]]; then
        echo "[=] $name already present — pulling latest"
        git -C "$target" pull --ff-only || echo "[!] could not fast-forward $name; leaving as-is"
    else
        echo "[+] cloning $name"
        rm -rf "$target"
        git clone --depth 1 "$url" "$target"
    fi
done

echo "[✓] Sources vendored into $DEST"
du -sh "$DEST"
