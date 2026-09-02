#!/usr/bin/env python3
"""Independent tiebreaker: onboard EKF (/mavros/local_position/pose) start vs end.
Not ground truth, but a third opinion vs ICP and the SLAM under test."""
import sys, numpy as np, rosbag
bag = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else '/mavros/local_position/pose'
first = None; last = None; pts = []
with rosbag.Bag(bag, 'r') as b:
    for _, m, _ in b.read_messages(topics=[topic]):
        p = m.pose.position
        v = np.array([p.x, p.y, p.z])
        if first is None:
            first = v
        last = v
        pts.append(v)
pts = np.array(pts)
if first is None:
    print("no msgs on", topic); sys.exit(0)
rto = np.linalg.norm(last - first)
# rough path length
path = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) if len(pts) > 1 else 0.0
print(f"{bag.split('/')[-1]}")
print(f"  mavros start: {first.round(3)}  end: {last.round(3)}")
print(f"  mavros RTO (onboard EKF) = {rto:.3f} m   path_len ~= {path:.1f} m   n={len(pts)}")
