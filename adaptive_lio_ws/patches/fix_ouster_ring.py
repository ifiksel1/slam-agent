#!/usr/bin/env python3
# Adaptive-LIO's vendored ouster_ros::Point (src/tools/point_types.h) declares `ring` as uint8_t
# and registers it as std::uint8_t. Modern Ouster drivers (and the field bags on this rig, and
# COIN-LIO's struct) publish `ring` as UINT16. The type mismatch makes PCL fromROSMsg reject the
# field ("Failed to find match for field 'ring'"), leaving ring garbage -> the scan-line index
# goes out of bounds -> SIGSEGV on the first cloud (node dies, zero /odom).
#
# This rewrites ONLY the ouster_ros::Point ring type to uint16_t (struct member + registration),
# leaving zjloc::FullPointType (which also has a uint8_t ring) untouched. Idempotent.
#
# Usage: fix_ouster_ring.py <adaptive_lio_src_root>
import sys, re
root = sys.argv[1]
path = f"{root}/src/tools/point_types.h"
src = open(path).read()

if "uint16_t ring;" in src and "(std::uint16_t, ring, ring)" in src.split("ouster_ros::Point,", 1)[-1]:
    print("fix_ouster_ring: already applied")
    sys.exit(0)

orig = src
# 1) struct member — `uint8_t ring;` is unique to the ouster struct (FullPointType uses `ring = 0;`)
src = src.replace("        uint8_t ring;", "        uint16_t ring;")

# 2) registration — only the occurrence inside the ouster_ros::Point register block
marker = "POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,"
head, tail = src.split(marker, 1)
tail = tail.replace("(std::uint8_t, ring, ring)", "(std::uint16_t, ring, ring)", 1)
src = head + marker + tail

if src == orig:
    print("ERROR: no substitutions made — upstream point_types.h changed", file=sys.stderr)
    sys.exit(1)

open(path, "w").write(src)
# verify: ouster now uint16 (struct + registration); FullPointType (before the marker) still uint8
assert "(std::uint16_t, ring, ring)" in src.split(marker, 1)[1], "ouster registration not patched"
assert "uint16_t ring;" in src, "ouster struct member not patched"
assert "uint8_t ring = 0;" in src, "FullPointType ring member was unexpectedly modified"
print("fix_ouster_ring: applied (ouster_ros::Point.ring uint8 -> uint16)")
