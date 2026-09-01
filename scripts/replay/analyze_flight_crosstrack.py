#!/usr/bin/env python3
"""
Measure LATERAL (cross-track) error on straight forward runs in an ArduPilot
dataflash log. Built to compare flights across a one-parameter-at-a-time sweep,
so every flight must be measured the SAME way -- hence a script, not ad-hoc code.

Symptom this exists for: pitching forward to translate produces 10-15 cm of
lateral deviation. Established on logs 180/181 (2026-08-14):
  - SLAM /Odometry itself is clean under pitch (see analyze_pitch_drift.py)
  - EKF fusion is healthy: innovation test ratios peak ~0.31 vs 1.0 threshold
  - vision and EKF agree (divergence sd 6.7 cm N / 12.4 cm E)
  - the position controller COMMANDS a straight line (target cross-track ~2 mm)
    while the vehicle deviates 32-86 mm -> genuine, not stick input
  - tracking error scales with SPEED (Spearman +0.50), more strongly than with
    vibration (+0.35, itself partly a proxy for speed)
  - VISO_DELAY_MS measured at 76 ms vs 75 ms set -> latency is NOT the cause

BASELINE to beat (logs 180+181, 5 clean runs, VISO_POS_M_NSE=0.2):
    |cross|/along  mean 2.7%  sd 2.6%  sem 1.1%
    mean |cross-track|  85 mm
Only a >50% improvement is resolvable against that baseline; see README note.

A run counts as CLEAN when roll stayed small enough that the lateral motion
cannot be commanded roll: |roll| < 3.5 deg AND |DesRoll| < 3.0 deg, with more
than 0.5 m of forward travel. On the source logs that kept 5 of 13 segments --
the excluded ones carried 5-8 deg of roll and up to 1.7 m of cross-track, which
is pilot input, not drift.

Usage:  analyze_flight_crosstrack.py <log.bin> [more.bin ...] [--json]
"""
import sys, json
import numpy as np
from pymavlink import mavutil

PITCH_CMD_THRESH = -4.0    # deg; sustained nose-down = commanded forward
MIN_SEG_S        = 1.0
MAX_ROLL         = 3.5     # deg
MAX_DESROLL      = 3.0     # deg
MIN_ALONG_M      = 0.5

BASELINE = dict(ratio_mean=0.027, ratio_sd=0.026, n=5, cross_mm=85.0)


def load(path):
    m = mavutil.mavlink_connection(path)
    K, A, X4, VP = [], [], [], []
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        tp = msg.get_type()
        t = getattr(msg, 'TimeUS', None)
        if t is None:
            continue
        t /= 1e6
        if tp == 'XKF1' and msg.C == 0:
            K.append((t, msg.PN, msg.PE, msg.VN, msg.VE, msg.Yaw))
        elif tp == 'ATT':
            A.append((t, msg.Roll, msg.DesRoll, msg.Pitch, msg.DesPitch, msg.Yaw))
        elif tp == 'XKF4' and msg.C == 0:
            X4.append((t, msg.SV, msg.SP))
        elif tp == 'VISP':
            VP.append((t, msg.PX, msg.PY))
    return (np.array(K, float), np.array(A, float),
            np.array(X4, float), np.array(VP, float))


def runs(K, A, tag):
    t, PN, PE, VN, VE, YAW = K[:, 0], K[:, 1], K[:, 2], K[:, 3], K[:, 4], K[:, 5]
    at, roll, desroll, despitch, yaw = A[:, 0], A[:, 1], A[:, 2], A[:, 4], A[:, 5]
    act = despitch < PITCH_CMD_THRESH
    segs, i = [], 0
    while i < len(act):
        if act[i]:
            j = i
            while j + 1 < len(act) and act[j + 1]:
                j += 1
            if at[j] - at[i] > MIN_SEG_S:
                segs.append((at[i], at[j]))
            i = j + 1
        else:
            i += 1

    out = []
    for n, (a, b) in enumerate(segs, 1):
        m = (t >= a) & (t <= b)
        am = (at >= a) & (at <= b)
        if m.sum() < 5 or am.sum() < 2:
            continue
        hdg = np.radians(np.interp(a, at, yaw))
        fwd = np.array([np.cos(hdg), np.sin(hdg)])
        lat = np.array([-np.sin(hdg), np.cos(hdg)])
        p0 = np.array([PN[m][0], PE[m][0]])
        dp = np.array([PN[m][-1], PE[m][-1]]) - p0
        along, cross = float(dp @ fwd), float(dp @ lat)
        mroll, mdes = np.abs(roll[am]).max(), np.abs(desroll[am]).max()
        speed = np.hypot(VN[m], VE[m]).max()
        clean = (mroll < MAX_ROLL and mdes < MAX_DESROLL and along > MIN_ALONG_M)
        out.append(dict(log=tag, n=n, t0=float(a), dur=float(b - a),
                        pitch=float(despitch[am].min()), along=along, cross=cross,
                        roll=float(mroll), desroll=float(mdes),
                        hdg=float(np.degrees(hdg) % 360), speed=float(speed),
                        clean=bool(clean)))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    as_json = '--json' in sys.argv
    if not args:
        print(__doc__)
        sys.exit(2)

    allruns = []
    innov = []
    for p in args:
        K, A, X4, VP = load(p)
        if len(K) < 10 or len(A) < 10:
            print("%s: insufficient XKF1/ATT" % p)
            continue
        allruns += runs(K, A, p.split('/')[-1])
        if len(X4):
            innov.append((p.split('/')[-1], np.percentile(X4[:, 1], 95), X4[:, 1].max(),
                          np.percentile(X4[:, 2], 95), X4[:, 2].max()))

    print("%-28s %-3s %-7s %-7s %-9s %-9s %-6s %-6s %-6s"
          % ("log", "#", "t0", "pitch", "along_m", "cross_mm", "roll", "hdg", "spd"))
    for r in allruns:
        print("%-28s %-3d %-7.1f %-7.1f %+9.2f %+9.0f %-6.1f %-6.0f %-6.2f%s"
              % (r['log'], r['n'], r['t0'], r['pitch'], r['along'], r['cross'] * 1000,
                 r['roll'], r['hdg'], r['speed'], "" if r['clean'] else "  (excluded)"))

    clean = [r for r in allruns if r['clean']]
    print("\nCLEAN runs: %d of %d  (|roll|<%.1f, |desroll|<%.1f, along>%.1fm)"
          % (len(clean), len(allruns), MAX_ROLL, MAX_DESROLL, MIN_ALONG_M))
    if not clean:
        print("  nothing to summarise -- were the runs flown with roll centred?")
        return

    ratio = np.array([abs(r['cross'] / r['along']) for r in clean])
    signed = np.array([r['cross'] / r['along'] for r in clean])
    crossmm = np.array([abs(r['cross']) * 1000 for r in clean])
    hdgs = np.array([r['hdg'] for r in clean])
    sem = ratio.std() / np.sqrt(len(ratio)) if len(ratio) > 1 else float('nan')
    print("  |cross|/along : mean %.3f%%  sd %.3f%%  sem %.3f%%"
          % (ratio.mean() * 100, ratio.std() * 100, sem * 100))
    print("  |cross-track| : mean %.0f mm  max %.0f mm" % (crossmm.mean(), crossmm.max()))
    print("  signed ratio  : mean %+.3f%%  (%d pos / %d neg)"
          % (signed.mean() * 100, (signed > 0).sum(), (signed < 0).sum()))
    print("  headings      : %s" % np.round(np.sort(hdgs), 0))
    spread = hdgs.max() - hdgs.min()
    if spread < 90:
        print("  NOTE: all runs within %.0f deg of heading -- a body-frame cause (sensor/"
              "airframe alignment) cannot be separated from a world-frame one (draft,"
              " directional estimator bias). Add runs at the reciprocal heading." % spread)
    else:
        # split by heading hemisphere to test body-vs-world
        mid = hdgs.min() + 180
        gA = signed[(hdgs < mid)]
        gB = signed[(hdgs >= mid)]
        if len(gA) > 1 and len(gB) > 1:
            print("  heading split : A n=%d mean %+.2f%% | B n=%d mean %+.2f%%"
                  % (len(gA), gA.mean() * 100, len(gB), gB.mean() * 100))
            print("    same sign  => body-frame cause (sensor/airframe alignment)")
            print("    flipped    => world-frame cause (draft, directional bias)")

    if innov:
        print("\ninnovation test ratios (reject at 1.0):")
        for nm, sv95, svmx, sp95, spmx in innov:
            print("  %-28s SV p95 %.2f max %.2f | SP p95 %.2f max %.2f"
                  % (nm, sv95, svmx, sp95, spmx))
        if max(i[2] for i in innov) > 0.7 or max(i[4] for i in innov) > 0.7:
            print("  WARNING: innovations above 0.7 -- vision is close to being rejected."
                  " If VISO_*_M_NSE was just lowered, revert it.")

    print("\nvs BASELINE (logs 180+181, VISO_POS_M_NSE=0.2): mean %.1f%%, %0.f mm, n=%d"
          % (BASELINE['ratio_mean'] * 100, BASELINE['cross_mm'], BASELINE['n']))
    if len(ratio) > 1:
        d = ratio.mean() - BASELINE['ratio_mean']
        sed = np.sqrt(sem ** 2 + (BASELINE['ratio_sd'] / np.sqrt(BASELINE['n'])) ** 2)
        print("  difference %+.2f%%  (se %.2f%%) -> %.1f sigma"
              % (d * 100, sed * 100, abs(d) / sed if sed > 0 else 0))
        if abs(d) < 2 * sed:
            print("  NOT RESOLVABLE: consistent with no change. The baseline's n=5 and"
                  " sd=2.6%% cannot resolve improvements smaller than ~50%%.")

    if as_json:
        print("\nJSON " + json.dumps(dict(
            clean_n=len(clean), ratio_mean=float(ratio.mean()), ratio_sd=float(ratio.std()),
            cross_mm_mean=float(crossmm.mean()), runs=clean)))


if __name__ == '__main__':
    main()
