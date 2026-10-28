#!/usr/bin/env python3
"""True speed of an L2 run while in Guided, from SITL's dataflash log: SITL's true velocity (SIM2)
against the FC's target velocity (PSCN/PSCE TVN/TVE, feed-forward plus its position correction).

usage: speed_check.py <run name> <sitl .BIN>
"""
import sys

import numpy as np
from pymavlink import mavutil

name, path = sys.argv[1:3]
m = mavutil.mavlink_connection(path)
tv, sv = [], []
guided = False
tvn = None
while True:
    x = m.recv_match(type=["PSCN", "PSCE", "SIM2", "MODE"])
    if x is None:
        break
    d = x.to_dict()
    t = x.get_type()
    if t == "MODE":
        guided = d["Mode"] == 15
    elif not guided:
        continue
    elif t == "SIM2":
        sv.append(np.hypot(d["VN"], d["VE"]))
    elif t == "PSCN":
        tvn = d["TVN"]
    elif t == "PSCE" and tvn is not None:
        tv.append(np.hypot(tvn, d["TVE"]))
sv, tv = np.array(sv), np.array(tv)
mv = sv > 0.2
print("%s  true speed max %.2f p99 %.2f p95 %.2f | FC target vel max %.2f p95 %.2f | time over 1.1 m/s %.1f%%" % (
    name, sv.max(), np.percentile(sv[mv], 99), np.percentile(sv[mv], 95), tv.max(), np.percentile(tv[tv > 0.2], 95),
    100 * np.mean(sv[mv] > 1.1)))
