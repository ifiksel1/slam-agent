#!/usr/bin/env python3
"""Was the magnetometer vote in log 49 independent of the SLAM failure?

ekf_check needs two of {SM, SV, SP}. In log 49 the pair was SM+SP. If SM is
really a consequence of the same lagged external-nav data (EK3_SRC1_YAW=6 means
yaw comes from SLAM too), then the "two independent votes" the check is designed
around collapse to one witness.
"""
import sys
from collections import defaultdict

from pymavlink import mavutil

path = sys.argv[1]
core = int(sys.argv[2]) if len(sys.argv) > 2 else 0

m = mavutil.mavlink_connection(path)
want = set(['XKF4', 'XKF3', 'XKF1', 'VISP', 'ATT'])
rows = defaultdict(list)
while True:
    msg = m.recv_match(type=want, blocking=False)
    if msg is None:
        break
    d = msg.to_dict()
    d['_t'] = msg.TimeUS / 1e6
    rows[msg.get_type()].append(d)

t0 = min(r['_t'] for r in rows['XKF1'])
k4 = {r['_t']: r for r in rows['XKF4'] if r.get('C') == core}
k3 = sorted([r for r in rows['XKF3'] if r.get('C') == core], key=lambda r: r['_t'])
k1 = sorted([r for r in rows['XKF1'] if r.get('C') == core], key=lambda r: r['_t'])
vp = sorted(rows['VISP'], key=lambda r: r['_t'])


def near(seq, t):
    return min(seq, key=lambda r: abs(r['_t'] - t))


print("%8s %7s %7s   %9s %9s %9s   %8s %8s %8s   %7s" %
      ("t", "SM", "SP", "IMX", "IMY", "IMZ", "IYAW", "EKF_yaw", "VIS_yaw", "speed"))
prev = None
for t in sorted(k4):
    rt = t - t0
    if not (14.0 <= rt <= 32.0) or int(rt * 5) % 2:
        continue
    r = k4[t]
    i3 = near(k3, t)
    e1 = near(k1, t)
    v = near(vp, t)
    sp = 0.0
    if prev is not None:
        dt = e1['_t'] - prev['_t']
        if dt > 0:
            sp = ((e1['PN'] - prev['PN']) ** 2 + (e1['PE'] - prev['PE']) ** 2) ** 0.5 / dt
    prev = e1
    dy = (e1['Yaw'] - v['Y'] + 180) % 360 - 180
    print("%8.2f %7.3f %7.3f   %9.3f %9.3f %9.3f   %8.3f %8.1f %8.1f   %7.2f" %
          (rt, r['SM'], r['SP'], i3['IMX'], i3['IMY'], i3['IMZ'], i3['IYAW'],
           e1['Yaw'], v['Y'], sp))
print("\n  EKF_yaw - VIS_yaw shown as the last two columns; speed is EKF ground speed (m/s).")
print("  If SM tracks speed rather than anything magnetic, the mag vote is a SLAM artefact.")
