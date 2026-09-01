#!/bin/bash
# Manage the tracked overrides that sit on top of the vendored sources in ./docker_src/.
#
# Why this exists: docker_src/ is .gitignored and is re-created by
# scripts/fetch_docker_sources.sh, which clones upstream. Our modifications to
# FAST-LIO live only in that ignored working tree, so a fresh clone + fetch
# produces a build with none of them. ./overrides/ is the tracked copy of every
# file we own; this script moves files between the two.
#
# Usage:
#   scripts/apply_overrides.sh check     # do overrides/ and docker_src/ agree?  (default)
#   scripts/apply_overrides.sh capture   # docker_src -> overrides   (after editing live)
#   scripts/apply_overrides.sh apply     # overrides -> docker_src   (after a fresh fetch)
#
# check exits non-zero on any drift, so it is safe to call from CI or a pre-commit hook.

set -euo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
SRC="$ROOT/docker_src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO"
OVR="$ROOT/overrides/FAST-LIO"

# Every file we have modified or added relative to upstream FAST_LIO_SLAM.
# Regenerate this list with:
#   git -C docker_src/FAST_LIO_SLAM status --porcelain
FILES=(
    "CMakeLists.txt"
    "config/hesai_jt128.yaml"
    "src/laserMapping.cpp"
    "src/fastlio_mavros_bridge.py"
    "src/drift_monitor.py"
    "src/ch9_logger.py"
    "src/flight_recorder.py"
    "src/viz_frames.py"
)

MODE="${1:-check}"

case "$MODE" in
  capture|apply|check) ;;
  *) echo "usage: $(basename "$0") [check|capture|apply]" >&2; exit 2 ;;
esac

if [[ "$MODE" != "capture" && ! -d "$OVR" ]]; then
    echo "[!] $OVR does not exist — run 'capture' first" >&2
    exit 1
fi
if [[ "$MODE" != "check" && ! -d "$SRC" ]]; then
    echo "[!] $SRC does not exist — run scripts/fetch_docker_sources.sh first" >&2
    exit 1
fi

drift=0
missing=0

for f in "${FILES[@]}"; do
    s="$SRC/$f"
    o="$OVR/$f"
    case "$MODE" in
      capture)
        if [[ ! -f "$s" ]]; then echo "[!] missing in docker_src: $f" >&2; missing=1; continue; fi
        mkdir -p "$(dirname "$o")"
        if [[ -f "$o" ]] && cmp -s "$s" "$o"; then
            printf '  =  %s\n' "$f"
        else
            cp -p "$s" "$o"
            printf '  ->  captured %s\n' "$f"
        fi
        ;;
      apply)
        if [[ ! -f "$o" ]]; then echo "[!] missing in overrides: $f" >&2; missing=1; continue; fi
        mkdir -p "$(dirname "$s")"
        if cmp -s "$o" "$s" 2>/dev/null; then
            printf '  =  %s\n' "$f"
        else
            cp -p "$o" "$s"
            printf '  ->  applied %s\n' "$f"
        fi
        ;;
      check)
        if [[ ! -f "$o" ]]; then printf '  MISSING-OVERRIDE  %s\n' "$f"; missing=1; continue; fi
        if [[ ! -f "$s" ]]; then printf '  MISSING-IN-SRC    %s\n' "$f"; missing=1; continue; fi
        if cmp -s "$s" "$o"; then
            printf '  ok       %s\n' "$f"
        else
            printf '  DRIFTED  %s\n' "$f"
            drift=1
        fi
        ;;
    esac
done

if [[ "$MODE" == "check" ]]; then
    if (( drift || missing )); then
        echo
        echo "[!] overrides/ and docker_src/ disagree."
        echo "    'capture' to take docker_src as truth, 'apply' to take overrides as truth."
        exit 1
    fi
    echo
    echo "[✓] overrides/ matches docker_src/ (${#FILES[@]} files)"
fi

(( missing )) && exit 1
exit 0
