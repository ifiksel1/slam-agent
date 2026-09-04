#!/usr/bin/env python3
"""Inspect EKF3 velocity-fusion health in a dataflash log.

Written for the 25 Aug 2026 hard landing (log 49). The question it answers:
did ExternalNav VELOCITY fusion (EK3_SRC1_VELXY/VELZ=6, VISO_VEL_M_NSE=0.1)
actually go bad during the FAST-LIO latency ramp, and did it go bad BEFORE
position did?

XKF4.SV / .SP / .SH / .SM are the EKF3 innovation test ratios for velocity,
position, height and magnetometer. >1.0 means the innovation gate rejected the
measurement; ekf_check compares these against FS_EKF_THRESH and needs two of
{SM, SV, SP} over threshold to declare a failsafe.
"""
import sys
from collections import defaultdict

from pymavlink import mavutil

path = sys.argv[1]
core = int(sys.argv[2]) if len(sys.argv) > 2 else 0
FS_EKF_THRESH = 0.8            # from the FC param dump

m = mavutil.mavlink_connection(path)
want = set(['XKF4', 'XKF3', 'XKF1', 'VISP', 'VISV', 'MODE', 'EV', 'MSG',
            'CTUN', 'RFND', 'IMU', 'ATT', 'POS'])
rows = defaultdict(list)
while True:
    msg = m.recv_match(type=want, blocking=False)
    if msg is None:
        break
    d = msg.to_dict()
    d['_t'] = msg.TimeUS / 1e6
    rows[msg.get_type()].append(d)

t0 = min(r['_t'] for r in rows['XKF1']) if rows['XKF1'] else 0.0


def rel(t):
    return t - t0


# ---- impact: biggest deviation from 1 g on the primary IMU ----
imp_t, imp_g = None, 0.0
for r in rows['IMU']:
    if r.get('I') != 0:
        continue
    mag = (r['AccX'] ** 2 + r['AccY'] ** 2 + r['AccZ'] ** 2) ** 0.5
    if abs(mag - 9.80665) > imp_g:
        imp_g, imp_t = abs(mag - 9.80665), r['_t']

print("=" * 76)
print("TIMELINE  (t=0 at first XKF1, core %d)" % core)
print("=" * 76)
for r in rows['MODE']:
    print("  %8.2f  MODE -> %s (num %s, reason %s)" % (rel(r['_t']), r['Mode'], r['ModeNum'], r['Rsn']))
for r in rows['EV']:
    print("  %8.2f  EV %s" % (rel(r['_t']), r['Id']))
for r in rows['MSG']:
    print("  %8.2f  MSG %s" % (rel(r['_t']), r['Message']))
if imp_t:
    print("  %8.2f  *** peak accel deviation %.1f m/s^2 (%.1f g) ***" % (rel(imp_t), imp_g, imp_g / 9.80665 + 1))

# ---- test-ratio time series ----
k4 = [r for r in rows['XKF4'] if r.get('C') == core]
k3 = [r for r in rows['XKF3'] if r.get('C') == core]
k1 = [r for r in rows['XKF1'] if r.get('C') == core]
print("\n%d XKF4 samples on core %d, %.1f s" % (len(k4), core, rel(k4[-1]['_t']) if k4 else 0))

print("\n" + "=" * 76)
print("EKF3 INNOVATION TEST RATIOS  (>1.0 = gate rejected; ekf_check thresh %.2f)" % FS_EKF_THRESH)
print("=" * 76)
print("%8s %7s %7s %7s %7s   %8s %8s %8s   %7s" %
      ("t", "SV(vel)", "SP(pos)", "SH(hgt)", "SM(mag)", "IVN", "IVE", "IVD", "alt"))
alt_by_t = sorted((r['_t'], -r['PD']) for r in k1)


def alt_at(t):
    best = min(alt_by_t, key=lambda x: abs(x[0] - t), default=(0, 0))
    return best[1]


k3_by_t = sorted(k3, key=lambda r: r['_t'])


def k3_at(t):
    return min(k3_by_t, key=lambda r: abs(r['_t'] - t)) if k3_by_t else None


step = max(1, len(k4) // 120)
for r in k4[::step]:
    i3 = k3_at(r['_t'])
    print("%8.2f %7.3f %7.3f %7.3f %7.3f   %8.3f %8.3f %8.3f   %7.2f" %
          (rel(r['_t']), r['SV'], r['SP'], r['SH'], r['SM'],
           i3['IVN'] if i3 else 0, i3['IVE'] if i3 else 0, i3['IVD'] if i3 else 0,
           alt_at(r['_t'])))

# ---- threshold crossings ----
print("\n" + "=" * 76)
print("FIRST CROSSINGS")
print("=" * 76)
for field, label in (('SV', 'velocity'), ('SP', 'position'), ('SH', 'height'), ('SM', 'mag')):
    for thr in (FS_EKF_THRESH, 1.0):
        hit = next((r for r in k4 if r[field] >= thr), None)
        print("  %-9s %-8s >= %.2f : %s" %
              (field, label, thr, ("t=%.2f s (alt %.2f m)" % (rel(hit['_t']), alt_at(hit['_t']))) if hit else "never"))

# how many of {SM,SV,SP} over threshold at once, and for how long
over = [(rel(r['_t']), sum(1 for f in ('SM', 'SV', 'SP') if r[f] >= FS_EKF_THRESH)) for r in k4]
two_plus = [t for t, c in over if c >= 2]
print("\n  samples with >=2 of {SM,SV,SP} over %.2f : %d" % (FS_EKF_THRESH, len(two_plus)))
if two_plus:
    print("  first at t=%.2f s, last at t=%.2f s" % (two_plus[0], two_plus[-1]))

# ---- what the FC received as vision velocity ----
print("\n" + "=" * 76)
print("VISV — vision velocity as RECEIVED by the FC")
print("=" * 76)
vv = rows['VISV']
if vv:
    ign = sum(1 for r in vv if r.get('Ign'))
    print("  %d samples, %d ignored, VErr %.3f..%.3f" %
          (len(vv), ign, min(r['VErr'] for r in vv), max(r['VErr'] for r in vv)))
    print("%8s %8s %8s %8s   %8s %8s" % ("t", "VX", "VY", "VZ", "|V|", "age_ms"))
    st = max(1, len(vv) // 60)
    for r in vv[::st]:
        sp = (r['VX'] ** 2 + r['VY'] ** 2 + r['VZ'] ** 2) ** 0.5
        age = (r['RTimeUS'] / 1000.0 - r['CTimeMS']) if r.get('CTimeMS') else float('nan')
        print("%8.2f %8.3f %8.3f %8.3f   %8.3f %8.1f" %
              (rel(r['_t']), r['VX'], r['VY'], r['VZ'], sp, age))
