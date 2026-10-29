#!/usr/bin/env python3
"""Print a failure-test run's true trajectory (/sim/truth, map frame) and the setpoints the planner
bridge sent, every `step` seconds, relative to the fault (or the bag start), with the executive's
phases and the FC's modes interleaved. Needs rosbags (Shannon: ~/sim/venv/bin/python).

usage: trace.py <run dir> [t_from t_to step]     (times in s relative to the fault)
"""
import json
import math
import os
import sys

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

run = sys.argv[1].rstrip("/")
t_from, t_to, step = (float(x) for x in sys.argv[2:5]) if len(sys.argv) > 4 else (-5.0, 60.0, 1.0)
res = json.load(open(os.path.join(run, "fault_result.json")))
t0 = res["fault"]["t"] if res.get("fault") else None
ts = get_typestore(Stores.ROS1_NOETIC)
want = {"/sim/truth", "/mavros/setpoint_raw/local", "/mission_executive/status", "/mavros/state",
        "/planner_bridge/status", "/track_planner/status"}
truth, sp, notes = [], [], []
last_b = None
with Reader(os.path.join(run, "closed_loop.bag")) as r:
    conns = [c for c in r.connections if c.topic in want]
    for c in conns:
        try:
            ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        except Exception:
            pass
    for c, t, d in r.messages(connections=conns):
        t = t / 1e9
        if t0 is None:
            t0 = t
        if t < t0 + t_from - 1 or t > t0 + t_to + 1:
            continue
        m = ts.deserialize_ros1(d, c.msgtype)
        if c.topic == "/sim/truth":
            p, v = m.pose.pose.position, m.twist.twist.linear
            truth.append((t, p.x, p.y, p.z, math.hypot(v.x, v.y)))
        elif c.topic == "/mavros/setpoint_raw/local":
            sp.append((t, m.position.x, m.position.y, m.position.z, m.type_mask))
        elif c.topic == "/mission_executive/status":
            notes.append((t, "exec " + m.data))
        elif c.topic == "/mavros/state":
            notes.append((t, "mode %s armed %s" % (m.mode, m.armed)))
        elif c.topic in ("/planner_bridge/status", "/track_planner/status"):
            s = json.loads(m.data)
            key = (c.topic, s.get("state"), "".join(ch for ch in s.get("reason", "") if not ch.isdigit()))
            if c.topic == "/planner_bridge/status" and key != last_b:
                last_b = key
                notes.append((t, "bridge %s %s enabled %s mode %s" % (s.get("state"), s.get("reason"), s.get("enabled"), s.get("mode"))))
notes.sort()
dedup, prev = [], None
for t, n in notes:
    if n != prev and not (n.startswith("mode") and prev and prev.startswith("mode") and n == prev):
        dedup.append((t, n))
    prev = n
k = 0
nxt = t0 + t_from
si = 0
for t, x, y, z, s in truth:
    while k < len(dedup) and dedup[k][0] <= t:
        if not dedup[k][1].startswith("mode") or (k == 0 or dedup[k][1] != dedup[k - 1][1]):
            print("%+7.2f   %s" % (dedup[k][0] - t0, dedup[k][1][:160]))
        k += 1
    if t >= nxt:
        while si + 1 < len(sp) and sp[si + 1][0] <= t:
            si += 1
        spx = sp[si] if sp and abs(sp[si][0] - t) < 0.2 else None
        print("%+7.2f truth %7.2f %7.2f %5.2f  v %.2f   sp %s" % (
            t - t0, x, y, z, s, "-" if spx is None else "fc(%.2f, %.2f, %.2f) mask %d" % spx[1:]))
        nxt += step
