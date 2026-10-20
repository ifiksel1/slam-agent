#!/usr/bin/env python3
"""
Evaluate bievr_live_replay.sh outputs (PROGRESS P23 step 3).

  bridge  <flight.bag>[,<part2>] <prefix>
      BIEVR bridge output (/replay/vision_pose/pose, /replay/vision_speed/speed_vector) against what
      the FC actually received in flight from FAST-LIO (/mavros/vision_pose/pose, .../speed_vector),
      matched by stamp, and roll/pitch against the FC's own attitude (/mavros/imu/data).

  calibrate <healthy_prefix>... --event=<prefix>:<healthy_until_s>:<flight.bag>
      /bievr_health distributions on healthy flight time (all healthy prefixes, plus the event
      flight before healthy_until_s), candidate drift_monitor thresholds from them, and how each
      candidate behaves on the event flight. Times are seconds from the first /bievr_health message.
"""
import sys

import numpy as np
import rosbag

sys.path.insert(0, '/replay')
from bievr_compare import q2R, rpy, match, wrap  # noqa: E402


def read(path, topics):
    out = {t: [] for t in topics}
    for part in path.split(','):
        with rosbag.Bag(part) as b:
            for tp, m, t in b.read_messages(topics=topics):
                out[tp].append((m, t.to_sec()))
    return out


def poses(msgs):
    return np.array([(m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
                      m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)
                     for m, _ in msgs])


def vecs(msgs):
    return np.array([(m.header.stamp.to_sec(), m.vector.x, m.vector.y, m.vector.z) for m, _ in msgs])


def bridge(flight, prefix):
    fl = read(flight, ['/mavros/vision_pose/pose', '/mavros/vision_speed/speed_vector', '/mavros/imu/data'])
    ch = read(prefix + '_chain.bag', ['/replay/vision_pose/pose', '/replay/vision_speed/speed_vector'])
    F, B = poses(fl['/mavros/vision_pose/pose']), poses(ch['/replay/vision_pose/pose'])
    Fv, Bv = vecs(fl['/mavros/vision_speed/speed_vector']), vecs(ch['/replay/vision_speed/speed_vector'])
    fc = np.array([(m.header.stamp.to_sec(), m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
                   for m, _ in fl['/mavros/imu/data']])
    print(f"== bridge: {flight}")
    print(f"   flown vision poses {len(F)}, BIEVR bridge poses {len(B)}, speeds {len(Fv)} / {len(Bv)}")
    ib, jf = match(B[:, 0], F[:, 0], 0.01)
    # Both bridges zero the heading at their first pose and start at their backend's origin; the
    # flown FAST-LIO may have started earlier (log 218), so compare displacement from the first match.
    dB = B[ib, 1:4] - B[ib[0], 1:4]
    dF = F[jf, 1:4] - F[jf[0], 1:4]
    e = np.linalg.norm(dB - dF, axis=1)
    print(f"   matched {len(ib)}: position displacement |BIEVR - flown| mean {e.mean():.3f} max {e.max():.3f} m"
          f"  (at 25/50/75% of flight: {e[len(e)//4]:.3f} {e[len(e)//2]:.3f} {e[3*len(e)//4]:.3f})")
    rb, rf = rpy(q2R(B[ib, 4:8])), rpy(q2R(F[jf, 4:8]))
    d = wrap(rb - rf)
    d[:, 2] = wrap(d[:, 2] - d[0, 2])
    print("   attitude BIEVR - flown: " + "  ".join(
        f"{k} mean {d[:, n].mean():+.2f} |max| {np.abs(d[:, n]).max():.2f}" for n, k in enumerate(('roll', 'pitch', 'yaw'))) + " deg")
    for name, P in (('BIEVR', B), ('flown', F)):
        i, j = match(P[:, 0], fc[:, 0], 0.03)
        dd = wrap(rpy(q2R(P[i, 4:8]))[:, :2] - rpy(q2R(fc[j, 1:5]))[:, :2])
        print(f"   {name:6} roll/pitch vs FC: mean {dd[:, 0].mean():+.2f} / {dd[:, 1].mean():+.2f}  "
              f"|max| {np.abs(dd[:, 0]).max():.2f} / {np.abs(dd[:, 1]).max():.2f} deg")
    iv, jv = match(Bv[:, 0], Fv[:, 0], 0.01)
    ev = Bv[iv, 1:4] - Fv[jv, 1:4]
    print(f"   velocity BIEVR - flown (ENU, m/s): rms per axis {np.sqrt((ev**2).mean(0)).round(3)}, "
          f"|v| flown rms {np.sqrt((Fv[jv, 1:4]**2).sum(1).mean()):.3f}")


def health(prefix):
    rows = [(t,) + tuple(m.data) for m, t in read(prefix + '_chain.bag', ['/bievr_health'])['/bievr_health']]
    h = np.array(rows)
    h[:, 0] -= h[0, 0]
    return h  # t, n_eff, cost, ms, n_src, e0, e1, e2, age_ms, eig_rot


COLS = {'matched': 1, 'cost': 2, 'proc_ms': 3, 'eig_min': 5, 'eig_rot': 9, 'age_ms': 8}


def runs(mask):
    best = c = 0
    for v in mask:
        c = c + 1 if v else 0
        best = max(best, c)
    return best


def calibrate(healthy, event):
    ev_prefix, ev_until, _ = event.split(':', 2)
    ev_until = float(ev_until)
    H = [health(p) for p in healthy]
    E = health(ev_prefix)
    pool = np.vstack(H + [E[E[:, 0] < ev_until]])
    print(f"== calibration pool: {len(pool)} healthy scans ({', '.join(healthy)} + {ev_prefix} < {ev_until:.0f} s)")
    for k, c in COLS.items():
        a = pool[:, c]
        print(f"   {k:8} min {a.min():10.4g}  p0.1 {np.percentile(a, .1):10.4g}  p1 {np.percentile(a, 1):10.4g}  "
              f"p50 {np.median(a):10.4g}  p99 {np.percentile(a, 99):10.4g}  max {a.max():10.4g}")
    # Lower-bound signals (eig_min, matched, eig_rot) and upper-bound (cost): candidate = healthy
    # extreme with margin. For each, report the worst consecutive run in the healthy pool and when
    # it first holds for k scans on the event flight.
    print(f"\n== candidates, event flight {ev_prefix} (failure onset ~85 s by IMU dead-reckoning)")
    cands = [('eig_min', 'lt', np.percentile(pool[:, 5], .1)), ('matched', 'lt', np.percentile(pool[:, 1], .1)),
             ('eig_rot', 'lt', np.percentile(pool[:, 9], .1)), ('cost', 'gt', np.percentile(pool[:, 2], 99.9))]
    for name, op, thr in cands:
        c = COLS[name]
        bad = (lambda a: a < thr) if op == 'lt' else (lambda a: a > thr)
        worst_healthy = max(runs(bad(h[:, c])) for h in H + [E[E[:, 0] < ev_until]])
        firsts = []
        for k in (3, 5, 10):
            m = bad(E[:, c])
            t_hit = next((E[i, 0] for i in range(k - 1, len(m)) if m[i - k + 1:i + 1].all()), None)
            firsts.append(f"k={k}: {t_hit:.2f} s" if t_hit is not None else f"k={k}: never")
        print(f"   {name:8} {'<' if op == 'lt' else '>'} {thr:10.4g}   worst healthy run {worst_healthy:3d} scans   "
              f"event first held: {'  '.join(firsts)}")
    print("\n== event flight timeline (0.5 s means)")
    print("   t[s]   matched     cost   eig_min    eig_rot  proc_ms")
    for T in np.arange(78, E[-1, 0], 0.5):
        s = (E[:, 0] >= T) & (E[:, 0] < T + 0.5)
        if s.any():
            r = E[s].mean(0)
            print(f"   {T:5.1f}  {r[1]:7.0f}  {r[2]:7.4f}  {r[5]:8.4g}  {r[9]:9.4g}  {r[3]:6.1f}")


if __name__ == '__main__':
    if sys.argv[1] == 'bridge':
        bridge(sys.argv[2], sys.argv[3])
    else:
        ev = [a for a in sys.argv[2:] if a.startswith('--event=')]
        calibrate([a for a in sys.argv[2:] if not a.startswith('--event=')], ev[0].split('=', 1)[1])
