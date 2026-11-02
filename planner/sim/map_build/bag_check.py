#!/usr/bin/env python3
"""Check a (split) flight recording before mapping it (offline tool).

  bag_check.py <bag> [<bag> ...]

Prints per topic: type, count, rate, the longest gap; for /lidar_points the scan gaps over 0.3 s;
for /Odometry (the live FAST-LIO, raw frame +x left, +y up, +z forward) the path in ENU
([0,0,1; 1,0,0; 0,1,0]), the distance from the start, the largest per-message step and speed.
"""
import sys
from collections import defaultdict

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

ts = get_typestore(Stores.ROS1_NOETIC)
P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
stamps = defaultdict(list)
types = {}
odom = []
for path in sys.argv[1:]:
    with Reader(path) as r:
        print("%s: %.1f s, %d msgs, %s .. %s" % (path.split("/")[-1], (r.end_time - r.start_time) * 1e-9,
              r.message_count, r.start_time * 1e-9, r.end_time * 1e-9))
        for c in r.connections:
            types[c.topic] = c.msgtype
        for con, t, raw in r.messages():
            stamps[con.topic].append(t * 1e-9)
            if con.topic == "/Odometry":
                m = ts.deserialize_ros1(raw, con.msgtype)
                p = m.pose.pose.position
                odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
t0 = min(v[0] for v in stamps.values())
for k in sorted(stamps):
    s = np.array(stamps[k])
    g = np.diff(s) if len(s) > 1 else np.array([0.0])
    print("%-45s %-35s n=%7d  %.1f..%.1f s  %.1f Hz  max gap %.2f s" % (
        k, types[k], len(s), s[0] - t0, s[-1] - t0, (len(s) - 1) / max(s[-1] - s[0], 1e-9), g.max()))
if "/lidar_points" in stamps:
    s = np.array(stamps["/lidar_points"])
    gi = np.where(np.diff(s) > 0.3)[0]
    print("lidar gaps > 0.3 s: %d %s" % (len(gi), [(round(s[i] - t0, 1), round(s[i + 1] - s[i], 2)) for i in gi[:20]]))
else:
    print("NO /lidar_points")
if odom:
    O = np.array(odom)
    E = O[:, 1:] @ P.T
    d = np.linalg.norm(E - E[0], axis=1)
    st = np.linalg.norm(np.diff(E, axis=0), axis=1)
    dt = np.maximum(np.diff(O[:, 0]), 1e-3)
    v = st / dt
    print("Odometry ENU: start %s end %s; path %.1f m; max dist from start %.1f m; z range %.2f..%.2f" % (
        np.round(E[0], 2), np.round(E[-1], 2), st.sum(), d.max(), E[:, 2].min(), E[:, 2].max()))
    print("  max step %.3f m, speed p99 %.2f max %.2f m/s; steps > 0.5 m: %d; odom gaps > 0.5 s: %d" % (
        st.max(), np.percentile(v, 99), v.max(), (st > 0.5).sum(), (dt > 0.5).sum()))
    for q in np.linspace(0, len(E) - 1, 12).astype(int):
        print("   t=%6.1f  %s" % (O[q, 0] - O[0, 0], np.round(E[q], 2)))
