#!/usr/bin/env python3
"""List the distinct high-g events in log 49 so the ground contact is not confused
with the drop that follows the pilot's disarm."""
import sys
from collections import defaultdict

from pymavlink import mavutil

m = mavutil.mavlink_connection(sys.argv[1])
rows = defaultdict(list)
while True:
    msg = m.recv_match(type=set(['IMU', 'XKF1', 'RFND', 'MODE']), blocking=False)
    if msg is None:
        break
    d = msg.to_dict()
    d['_t'] = msg.TimeUS / 1e6
    rows[msg.get_type()].append(d)

t0 = min(r['_t'] for r in rows['XKF1'])
print("XKF1 t0 = %.2f s absolute (postmortem 'Log t')" % t0)

ev, last = [], -9
for r in rows['IMU']:
    if r.get('I') != 0:
        continue
    g = (r['AccX'] ** 2 + r['AccY'] ** 2 + r['AccZ'] ** 2) ** 0.5
    dev = abs(g - 9.80665)
    if dev > 25.0 and r['_t'] - last > 0.30:
        ev.append((r['_t'], dev, g))
        last = r['_t']

rf = sorted([r for r in rows['RFND'] if r.get('Instance') == 0], key=lambda r: r['_t'])
print("\n%10s %10s %9s %9s %8s" % ("abs t", "rel t", "dev m/s2", "|a| g", "rngfnd"))
for t, dev, g in ev:
    near = min(rf, key=lambda r: abs(r['_t'] - t)) if rf else None
    print("%10.2f %10.2f %9.1f %9.2f %8.2f" %
          (t, t - t0, dev, g / 9.80665, (near['Dist'] if near else float('nan'))))
