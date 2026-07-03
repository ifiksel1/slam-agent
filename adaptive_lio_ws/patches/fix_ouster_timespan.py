#!/usr/bin/env python3
# Two upstream bugs in Adaptive-LIO's Oust64Handler (src/preprocess/cloud_convert/cloud_convert2.cc)
# that SIGSEGV on this rig's ORGANIZED Ouster clouds (64x1024 with no-return points):
#
#  1) timespan_ = pl_orig.points.back().t / tm_scale
#     For an organized/destaggered cloud the LAST element is NOT the max-timestamp point
#     (measured: back().t=145430 vs max_t=49370610). So timespan_ (hence delta_time) comes out
#     ~300x too small.
#  2) int id = (t/tm_scale)/delta_time; if (id<0||id>=sweep_cut_num) id = id - 1;
#     With the tiny delta_time, id blows up (~340). The "clamp" only subtracts 1, so id stays far
#     out of range -> v_cloud[id].push_back() writes out of bounds -> crash on the first cloud.
#
# Fix: derive timespan_ from the MAX t over the cloud, and hard-clamp id into [0, sweep_cut_num-1].
# The id clamp is fixed for every *Handler (all share the identical broken line); only the ouster
# timespan line is touched (only Oust64Handler reads .back().t). Idempotent.
#
# Usage: fix_ouster_timespan.py <adaptive_lio_src_root>
import sys
root = sys.argv[1]
path = f"{root}/src/preprocess/cloud_convert/cloud_convert2.cc"
src = open(path).read()
orig = src

BAD_TS = "timespan_ = pl_orig.points.back().t / tm_scale;"
GOOD_TS = ("uint32_t _tmax = 0; for (size_t _i = 0; _i < pl_orig.points.size(); ++_i) "
           "if (pl_orig.points[_i].t > _tmax) _tmax = pl_orig.points[_i].t; "
           "timespan_ = _tmax / tm_scale;  // organized-cloud safe (was points.back().t)")
BAD_ID = "id = id - 1;"
GOOD_ID = "id = (id < 0) ? 0 : (sweep_cut_num - 1);  // clamp (was id-1, left it out of range)"

if GOOD_TS.split("//")[0].strip() in src and BAD_ID not in src:
    print("fix_ouster_timespan: already applied")
    sys.exit(0)

n_ts = src.count(BAD_TS)
n_id = src.count(BAD_ID)
src = src.replace(BAD_TS, GOOD_TS)
src = src.replace(BAD_ID, GOOD_ID)

if src == orig:
    print("ERROR: no substitutions — upstream cloud_convert2.cc changed", file=sys.stderr)
    sys.exit(1)
open(path, "w").write(src)
assert BAD_TS not in src and BAD_ID not in src, "leftover buggy lines"
print(f"fix_ouster_timespan: applied (timespan sites={n_ts}, id-clamp sites={n_id})")
