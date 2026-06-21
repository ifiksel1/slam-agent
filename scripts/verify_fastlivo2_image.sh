#!/bin/bash
# Stage 6 verification for fast_livo2_slam:latest
# Checks: package availability, launch dry-run, entrypoint health

set -euo pipefail

IMAGE="fast_livo2_slam:latest"
PASS=0
FAIL=0

ok()  { echo "  [PASS] $*"; PASS=$((PASS+1)); }
fail(){ echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
hdr() { echo ""; echo "=== $* ==="; }

hdr "Image existence"
if docker image inspect "$IMAGE" &>/dev/null; then
    SIZE=$(docker image inspect "$IMAGE" --format '{{.Size}}' | awk '{printf "%.1f GB", $1/1073741824}')
    ok "Image $IMAGE exists (size: $SIZE)"
else
    fail "Image $IMAGE not found — build must have failed"
    exit 1
fi

# Run checks inside the container
RUN="docker run --rm --network host $IMAGE bash -c"

hdr "ROS package availability"
for pkg in fast_livo ouster_ros vision_to_mavros fast_livo2_integration; do
    if $RUN "source /root/slam_ws/devel/setup.bash && rospack find $pkg" &>/dev/null; then
        ok "rospack find $pkg"
    else
        fail "rospack find $pkg — package not found in built workspace"
    fi
done

hdr "Binary existence"
if $RUN "source /root/slam_ws/devel/setup.bash && which fastlivo_mapping || ls /root/slam_ws/devel/lib/fast_livo/fastlivo_mapping" &>/dev/null; then
    ok "fastlivo_mapping binary present"
else
    fail "fastlivo_mapping binary not found"
fi

hdr "Config files"
for cfg in fast_livo2_ouster64.yaml camera_minimal.yaml; do
    if $RUN "ls /root/slam_ws/src/fast_livo2_integration/config/$cfg" &>/dev/null; then
        ok "config/$cfg present in container"
    else
        fail "config/$cfg missing in container"
    fi
done

hdr "Sophus installation"
if $RUN "ls /usr/local/lib/libSophus.so" &>/dev/null; then
    ok "Sophus installed at /usr/local/lib/libSophus.so"
else
    fail "Sophus library not found at /usr/local/lib/libSophus.so"
fi

hdr "Launch file XML check"
# roslaunch --dump-params requires a running rosmaster; use python ElementTree
# (always present) for a well-formedness parse of each launch file.
# Note: feed the path via argv (sys.argv[1]) to avoid nested-quote breakage,
# and grep for a unique token (PARSE_OK) that the entrypoint banner can't emit.
for lf in slam.launch master.launch; do
    if $RUN "python3 -c 'import sys,xml.etree.ElementTree as ET; ET.parse(sys.argv[1]); print(\"PARSE_OK\")' /root/slam_ws/src/fast_livo2_integration/launch/$lf" 2>/dev/null | grep -q "PARSE_OK"; then
        ok "launch/$lf is valid XML"
    else
        fail "launch/$lf XML parse error"
    fi
done

hdr "roslaunch dependency resolution (offline node/pkg check)"
# roslaunch --nodes prints the node list without launching; this validates that
# every pkg/type referenced in the launch tree resolves in the built workspace.
if $RUN "source /root/slam_ws/devel/setup.bash && roslaunch --nodes fast_livo2_integration master.launch" &>/dev/null; then
    ok "roslaunch --nodes master.launch resolves all packages/types"
else
    fail "roslaunch --nodes master.launch failed to resolve (non-fatal: may need params)"
fi

echo ""
echo "================================================"
echo "  PASS: $PASS   FAIL: $FAIL"
echo "================================================"
[ $FAIL -eq 0 ]
