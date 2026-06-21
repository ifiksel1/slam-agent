#!/bin/bash
# Vendor the ROS source packages needed by Dockerfile.fastlivo2 into ./docker_src_fastlivo2/.
#
# Why: the Dockerfile COPYs these in instead of cloning inside the build, because
# the build container has no git credentials.  Cloning here (on the host) uses
# your existing git / gh credentials.
#
# FAST-LIVO2 dependencies (per README):
#   - FAST-LIVO2   : https://github.com/hku-mars/FAST-LIVO2.git
#   - rpg_vikit    : https://github.com/xuankuzcr/rpg_vikit.git  (FAST-LIVO2-specific fork)
#   - Sophus       : https://github.com/strasdat/Sophus.git  @ commit a621ff
#                    (non-templated double-only Sophus — FAST-LIVO2 README requirement)
#   - ouster-ros   : ARM64-safe driver COPYed from ultraviewdev_ws (not cloned — avoids exit-255 crash)
#   - vision_to_mavros  : COPYed from ultraviewdev_ws onboard_utilities
#   - fast_livo2_integration  : COPYed from this repo (our generated config/launch package)
#
# Usage:
#   scripts/fetch_docker_sources_fastlivo2.sh
#   docker build -f Dockerfile.fastlivo2 -t fast_livo2_slam:latest .
#
# docker_src_fastlivo2/ is .gitignored and excluded via .dockerignore.

set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
DEST="$ROOT/docker_src_fastlivo2"

ULTRAVIEW_WS="/home/dev/ultraviewdev_ws/src/Ultra-onboard"

# ---------------------------------------------------------------------------
# Cloned repos
# ---------------------------------------------------------------------------
REPOS=(
    "FAST-LIVO2|https://github.com/hku-mars/FAST-LIVO2.git|"
    "rpg_vikit|https://github.com/xuankuzcr/rpg_vikit.git|"
)

mkdir -p "$DEST"

for entry in "${REPOS[@]}"; do
    name="${entry%%|*}"
    rest="${entry#*|}"
    url="${rest%%|*}"
    target="$DEST/$name"

    if [[ -d "$target/.git" ]]; then
        echo "[=] $name already present — pulling latest"
        git -C "$target" pull --ff-only || echo "[!] could not fast-forward $name; leaving as-is"
    else
        echo "[+] cloning $name"
        git clone --depth 1 "$url" "$target" 2>&1 | tail -3
    fi
done

# Sophus: use the local copy at /home/dev/Sophus which is already at commit a621ff (correct
# non-templated double-only Sophus required by FAST-LIVO2 README).  Cloning from GitHub and
# checking out that old commit requires --unshallow which is slow and bandwidth-heavy.
LOCAL_SOPHUS="/home/dev/Sophus"
SOPHUS_DEST="$DEST/Sophus"
if [[ -d "$LOCAL_SOPHUS/.git" ]]; then
    LOCAL_COMMIT=$(git -C "$LOCAL_SOPHUS" rev-parse --short HEAD 2>/dev/null || echo "unknown")
    echo "[C] copying Sophus from local $LOCAL_SOPHUS (commit $LOCAL_COMMIT) — non-templated a621ff build"
    rm -rf "$SOPHUS_DEST"
    cp -a "$LOCAL_SOPHUS" "$SOPHUS_DEST"
else
    echo "[!] WARNING: local Sophus not found at $LOCAL_SOPHUS; falling back to full GitHub clone"
    git clone https://github.com/strasdat/Sophus.git "$SOPHUS_DEST"
    git -C "$SOPHUS_DEST" fetch --unshallow 2>/dev/null || true
    git -C "$SOPHUS_DEST" checkout a621ff2187ddc6bbeed8f09b48e6c7bcf5d2e5b6
fi

# ---------------------------------------------------------------------------
# Copied local sources (use the ARM64-safe pre-built drivers)
# ---------------------------------------------------------------------------
copy_local() {
    local src="$1"
    local name="$2"
    local dst="$DEST/$name"
    if [[ ! -d "$src" ]]; then
        echo "[!] WARNING: local source not found: $src"
        return 1
    fi
    echo "[C] copying $name from $src"
    rm -rf "$dst"
    cp -a "$src" "$dst"
}

copy_local \
    "$ULTRAVIEW_WS/onboard_flight_ops/sen_drv/ouster_ros_onb/ouster-ros" \
    "ouster-ros"

copy_local \
    "$ULTRAVIEW_WS/onboard_utilities/vision_to_mavros_onb" \
    "vision_to_mavros"

copy_local \
    "$ROOT/fast_livo2_integration" \
    "fast_livo2_integration"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "[✓] Sources vendored into $DEST"
echo ""
echo "Directories:"
for d in "$DEST"/*/; do
    count=$(find "$d" -type f | wc -l)
    printf "  %-35s  %d files\n" "$(basename "$d")" "$count"
done
echo ""
du -sh "$DEST"
