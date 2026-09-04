#!/usr/bin/env python3
"""Second pass on log 49: which EKF votes actually tripped the failsafe, and what
latency did the FC itself observe on the vision stream?

VISP/VISV log CTimeMS = "corrected system time" in FC milliseconds (after MAVLink
timesync), so TimeUS/1000 - CTimeMS is the age of the sample AS THE FC SAW IT.
That is the on-board counterpart to the companion-side data[7] latency.
"""
import sys
from collections import defaultdict

from pymavlink import mavutil

path = sys.argv[1]
core = int(sys.argv[2]) if len(sys.argv) > 2 else 0
THRESH = 0.8

m = mavutil.mavlink_connection(path)
want = set(['XKF4', 'XKF1', 'VISP', 'VISV', 'MODE', 'MSG'])
rows = defaultdict(list)
while True:
    msg = m.recv_match(type=want, blocking=False)
    if msg is None:
        break
    d = msg.to_dict()
    d['_t'] = msg.TimeUS / 1e6
    rows[msg.get_type()].append(d)

t0 = min(r['_t'] for r in rows['XKF1'])
k4 = [r for r in rows['XKF4'] if r.get('C') == core]
k1 = [r for r in rows['XKF1'] if r.get('C') == core]
alt = sorted((r['_t'], -r['PD']) for r in k1)


def alt_at(t):
    return min(alt, key=lambda x: abs(x[0] - t))[1]


print("=" * 84)
print("ekf_check VOTES  (needs 2 of {SM,SV,SP} >= %.2f for 10 consecutive 10 Hz iterations)" % THRESH)
print("=" * 84)
print("%8s %7s %7s %7s   %-14s %s" % ("t", "SM", "SV", "SP", "over-threshold", "alt"))
run = 0
declared = None
for r in k4:
    t = r['_t'] - t0
    if not (16.0 <= t <= 32.0):
        continue
    names = [f for f in ('SM', 'SV', 'SP') if r[f] >= THRESH]
    run = run + 1 if len(names) >= 2 else 0
    if run == 10 and declared is None:
        declared = t
    flag = "  <== 1.0 s sustained" if run == 10 else ""
    print("%8.2f %7.3f %7.3f %7.3f   %-14s %6.2f%s" %
          (t, r['SM'], r['SV'], r['SP'], "+".join(names) or "-", alt_at(r['_t']), flag))

print("\n  ekf_check would declare at t=%.2f s" % declared if declared else "\n  never declared")
for r in rows['MODE']:
    print("  MODE -> %s at t=%.2f s" % (r['Mode'], r['_t'] - t0))
for r in rows['MSG']:
    if 'EKF' in r['Message'] or 'Failsafe' in r['Message']:
        print("  MSG  %-40s t=%.2f s" % (r['Message'], r['_t'] - t0))

print("\n" + "=" * 84)
print("VISION LATENCY AS THE FC MEASURED IT  (TimeUS/1000 - CTimeMS, ms)")
print("VISO_DELAY_MS is set to 75 — anything above that is UNCOMPENSATED lag")
print("=" * 84)
for name in ('VISP', 'VISV'):
    v = rows[name]
    ages = [(r['_t'] - t0, r['TimeUS'] / 1000.0 - r['CTimeMS']) for r in v]
    ages = [(t, a) for t, a in ages if -1e4 < a < 1e5]
    if not ages:
        print("  %s: no plausible ages" % name)
        continue
    vals = sorted(a for _, a in ages)
    print("\n  %s  n=%d   p50 %.0f  p90 %.0f  p99 %.0f  max %.0f ms" %
          (name, len(vals), vals[len(vals) // 2], vals[int(.9 * len(vals))],
           vals[int(.99 * len(vals))], vals[-1]))
    print("  %8s %10s" % ("t", "age_ms"))
    step = max(1, len(ages) // 45)
    for t, a in ages[::step]:
        bar = "#" * min(60, int(a / 15))
        print("  %8.2f %10.0f  %s" % (t, a, bar))

print("\n" + "=" * 84)
print("VISP position vs EKF position (did SLAM position JUMP, or just lag?)")
print("=" * 84)
vp = rows['VISP']
print("  %8s %8s %8s %8s   %8s %8s %8s   %6s" %
      ("t", "VIS_PX", "VIS_PY", "VIS_PZ", "EKF_PN", "EKF_PE", "EKF_PD", "Rst"))
k1s = sorted(k1, key=lambda r: r['_t'])
step = max(1, len(vp) // 50)
for r in vp[::step]:
    t = r['_t'] - t0
    e = min(k1s, key=lambda x: abs(x['_t'] - r['_t']))
    print("  %8.2f %8.2f %8.2f %8.2f   %8.2f %8.2f %8.2f   %6d" %
          (t, r['PX'], r['PY'], r['PZ'], e['PN'], e['PE'], e['PD'], r['Rst']))
