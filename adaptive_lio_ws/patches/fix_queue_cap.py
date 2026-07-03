#!/usr/bin/env python3
# Adaptive-LIO's lidarodom_m::pushData() appends every deskewed cloud to lidar_buffer_ (a
# std::deque) with NO size cap (src/lio/lidarodom.cpp). When per-scan processing falls below the
# 20 Hz ingress (it does, early, while the map builds), the backlog grows ~6 MB/scan and RSS
# balloons to multiple GB -> OOM / std::terminate. This caps the buffer to a rolling window: under
# overload it DROPS the oldest scans instead of buffering them (correct real-time behaviour, and it
# hard-bounds memory). Cap = 40 scans (~2 s at 20 Hz). Idempotent.
#
# Usage: fix_queue_cap.py <adaptive_lio_src_root>
import sys
root = sys.argv[1]
path = f"{root}/src/lio/lidarodom.cpp"
src = open(path).read()

CAP = "lidar_buffer_.size() > 40"
if CAP in src:
    print("fix_queue_cap: already applied")
    sys.exit(0)

# Anchor on the push in pushData(std::vector<point3D> ...); insert the cap right after the pushes,
# still inside the mtx_buf lock (consumer takes the same lock), before mtx_buf.unlock().
ANCHOR = ("          lidar_buffer_.push_back(msg);\n"
          "          time_buffer_.push_back(data);\n"
          "          last_timestamp_lidar_ = data.first;\n")
INSERT = ("          // cap backlog: drop oldest scans under overload -> bounded RAM (added)\n"
          "          while (lidar_buffer_.size() > 40) { lidar_buffer_.pop_front(); time_buffer_.pop_front(); }\n")

if ANCHOR not in src:
    print("ERROR: pushData anchor not found — upstream lidarodom.cpp changed", file=sys.stderr)
    sys.exit(1)
if src.count(ANCHOR) != 1:
    print(f"ERROR: anchor appears {src.count(ANCHOR)}x, expected 1", file=sys.stderr)
    sys.exit(1)

src = src.replace(ANCHOR, ANCHOR + INSERT, 1)
open(path, "w").write(src)
assert CAP in src
print("fix_queue_cap: applied (lidar_buffer_ capped at 40 scans)")
