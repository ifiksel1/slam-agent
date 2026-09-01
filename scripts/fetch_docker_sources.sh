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

# repo dir name -> clone URL -> pinned sha  (FAST_LIO_SLAM bundles fast_lio + aloam_velodyne + hesai_ros_driver)
# SHAs are pinned so a fresh machine reproduces the system that actually flew.
# They are duplicated in overrides/UPSTREAM.yaml, which is the documented copy — keep both in step.
REPOS=(
    "FAST_LIO_SLAM|https://github.com/RevoluteRobotics/FAST_LIO_SLAM.git|0e5ffbd6e0b11b5882301c7fdde596488445ffe3"
    "livox_ros_driver|https://github.com/Livox-SDK/livox_ros_driver.git|3d240d5666129e1a3052e78ee8487a04b08fdda3"
    "vision_to_mavros|https://github.com/thien94/vision_to_mavros.git|139f199fcc82199e357279b11b94e651b07c284e"
)

mkdir -p "$DEST"
for entry in "${REPOS[@]}"; do
    name="${entry%%|*}"
    rest="${entry#*|}"
    url="${rest%%|*}"
    sha="${rest##*|}"
    target="$DEST/$name"

    if [[ -d "$target/.git" ]]; then
        # Never touch a dirty tree: our FAST-LIO modifications live uncommitted in
        # docker_src/FAST_LIO_SLAM, and discarding them is the failure this guard exists for.
        if [[ -n "$(git -C "$target" status --porcelain)" ]]; then
            echo "[=] $name present with local changes — leaving untouched"
            continue
        fi
        echo "[=] $name present and clean — checking out pinned $sha"
        git -C "$target" fetch --depth 1 origin "$sha" 2>/dev/null || git -C "$target" fetch origin
        git -C "$target" checkout --quiet "$sha" || echo "[!] could not check out $sha in $name"
    else
        echo "[+] cloning $name at $sha"
        rm -rf "$target"
        git clone "$url" "$target"
        git -C "$target" checkout --quiet "$sha"
    fi
done

echo "[✓] Sources vendored into $DEST"
du -sh "$DEST"

# docker_src/ is .gitignored, so the clones above carry NONE of our modifications.
# overrides/ is the tracked copy; lay it over the top or the build is plain upstream.
echo
echo "[+] applying tracked overrides"
"$ROOT/scripts/apply_overrides.sh" apply
