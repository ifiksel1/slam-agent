#!/usr/bin/env python3
"""
Compare an offline BIEVR-LIO replay (bievr_run_one.sh) against what FLEW on the same bag:
the recorded FAST-LIO /Odometry and the FC's own attitude (/mavros/imu/data). No ground truth;
the FC attitude is the only independent reference (roll/pitch from the FC IMU; FC yaw is fused
with our vision yaw, so yaw is only compared RELATIVE).

Frames (see config/fastlio_to_fc.launch and PROGRESS P23):
  JT128 IMU axes: x=left, y=up, z=forward.   M = IMU -> FLU body = [0,0,1, 1,0,0, 0,1,0].
  FAST-LIO world = IMU frame at start           -> ENU via M (the bridge's position_matrix).
  BIEVR world    = gravity-levelled at start    -> ENU via P (predicted [0,-1,0, 1,0,0, 0,0,1]).
The script FITS the BIEVR->ENU rotation from positions (using FAST-LIO+M as the reference) and
reports how far it is from P, so P is checked against data rather than trusted.

Usage (ROS python env, e.g. inside slam-system):
  bievr_compare.py <flight.bag>[,<part2.bag>] <prefix>     # reads <prefix>.tum, <prefix>_odom.bag, <prefix>.log
"""
import re
import sys

import numpy as np
import rosbag

M = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
P_PRED = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)


def q2R(q):  # q: (n,4) x y z w
    x, y, z, w = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def ang(Ra, Rb):  # angle of Ra^T Rb, degrees
    Rd = np.einsum('nji,njk->nik', Ra, Rb)
    return np.degrees(np.arccos(np.clip((np.trace(Rd, axis1=1, axis2=2) - 1) / 2, -1, 1)))


def rpy(R):  # ZYX: roll, pitch, yaw in degrees
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    pitch = -np.arcsin(np.clip(R[:, 2, 0], -1, 1))
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.degrees(np.stack([roll, pitch, yaw], 1))


def wrap(a):
    return (a + 180) % 360 - 180


def match(ta, tb, tol):
    """indices ia, ib with |ta[ia]-tb[ib]| < tol (nearest neighbour into tb)."""
    j = np.clip(np.searchsorted(tb, ta), 1, len(tb) - 1)
    j = np.where(np.abs(tb[j - 1] - ta) < np.abs(tb[j] - ta), j - 1, j)
    ok = np.abs(tb[j] - ta) < tol
    return np.nonzero(ok)[0], j[ok]


def kabsch(a, b):  # R, t minimising |a - (R b + t)|
    ca, cb = a.mean(0), b.mean(0)
    U, _, Vt = np.linalg.svd((b - cb).T @ (a - ca))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, ca - R @ cb


def load_flight(path):
    odom, fc = [], []
    for part in path.split(','):  # split flight bags, comma-separated, in order
        with rosbag.Bag(part) as b:
            for topic, m, _ in b.read_messages(topics=['/Odometry', '/mavros/imu/data']):
                if topic == '/Odometry':
                    p, q = m.pose.pose.position, m.pose.pose.orientation
                    odom.append((m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                else:
                    q = m.orientation
                    fc.append((m.header.stamp.to_sec(), q.x, q.y, q.z, q.w))
    return np.array(odom), np.array(fc)


def load_bievr_odom(path):
    rows = []
    try:
        with rosbag.Bag(path) as b:
            for _, m, _ in b.read_messages(topics=['/bievr_lio/odom']):
                q, v = m.pose.pose.orientation, m.twist.twist.linear
                rows.append((m.header.stamp.to_sec(), q.x, q.y, q.z, q.w, v.x, v.y, v.z))
    except Exception as e:  # noqa: BLE001 - best-effort side channel
        print(f"  (no odom bag: {e})")
    return np.array(rows)


def step_times(log_path):
    """Per-scan 'step' durations [ms] from BIEVR's timing printout (best effort)."""
    out = []
    try:
        for line in open(log_path, errors='replace'):
            m = re.match(r'\s*step\s+(.*)', line)
            if m:
                nums = re.findall(r'[-+]?\d*\.?\d+(?:e[-+]?\d+)?', m.group(1))
                if nums:
                    out.append(nums)
    except FileNotFoundError:
        pass
    return out


def main():
    flight, prefix = sys.argv[1], sys.argv[2]
    tum = np.loadtxt(prefix + '.tum')
    fl, fc = load_flight(flight)
    print(f"== {flight}")
    print(f"BIEVR poses {len(tum)}  FAST-LIO /Odometry {len(fl)}  FC attitude {len(fc)}")
    gaps = np.diff(tum[:, 0])
    print(f"BIEVR span {tum[-1,0]-tum[0,0]:.1f} s, first pose {tum[0,0]-fl[0,0]:+.2f} s after first /Odometry, "
          f"max gap {gaps.max()*1e3:.0f} ms")

    tb, pb, Rb = tum[:, 0], tum[:, 1:4], q2R(tum[:, 4:8])
    tf, pf, Rf = fl[:, 0], fl[:, 1:4], q2R(fl[:, 4:8])

    # ---- A. frame fit from positions ---------------------------------------------------
    ib, jf = match(tb, tf, 0.01)
    a = (M @ pf[jf].T).T                        # FAST-LIO -> ENU (what the bridge sends)
    b = pb[ib]
    R_fit, t_fit = kabsch(a, b)
    res_fit = np.linalg.norm(a - (b @ R_fit.T + t_fit), axis=1)
    d_pred = np.degrees(np.arccos(np.clip((np.trace(P_PRED.T @ R_fit) - 1) / 2, -1, 1)))
    print(f"\n[A] BIEVR->ENU rotation fitted on {len(ib)} matched poses (vs FAST-LIO*M):")
    for row in R_fit:
        print("      [" + ", ".join(f"{v:+.3f}" for v in row) + "]")
    print(f"    angle from predicted P [0,-1,0, 1,0,0, 0,0,1]: {d_pred:.2f} deg")
    print(f"    residual after fit: rms {np.sqrt((res_fit**2).mean()):.3f} m, max {res_fit.max():.3f} m")

    # ---- D. position disagreement with the PREDICTED matrix (what a bridge would send) -----
    e = (b @ P_PRED.T) - a
    en = np.linalg.norm(e, axis=1)
    print(f"\n[D] |BIEVR*P - FAST-LIO*M| over the flight: mean {en.mean():.3f}  max {en.max():.3f}  "
          f"final {en[-1]:.3f} m  (final dxyz {e[-1,0]:+.3f} {e[-1,1]:+.3f} {e[-1,2]:+.3f})")
    pl = lambda p: np.linalg.norm(np.diff(p, axis=0), axis=1).sum()  # noqa: E731
    print(f"    path length: BIEVR {pl(b):.1f} m, FAST-LIO {pl(a):.1f} m;  final z: "
          f"BIEVR {(b @ P_PRED.T)[-1,2]:+.3f}, FAST-LIO {a[-1,2]:+.3f} m (both start at 0)")

    # ---- B. absolute roll/pitch vs FC, relative yaw -------------------------------------------
    tq = fc[:, 0]
    Rfc = q2R(fc[:, 1:5])
    print("\n[B] attitude vs FC /mavros/imu/data (roll/pitch absolute; yaw relative to first sample):")
    for name, t, R_enu in (("BIEVR", tb, np.einsum('ij,njk,kl->nil', P_PRED, Rb, M.T)),
                           ("FAST-LIO", tf, np.einsum('ij,njk,kl->nil', M, Rf, M.T))):
        i, j = match(t, tq, 0.03)
        d = rpy(R_enu[i]) - rpy(Rfc[j])
        d[:, 2] = wrap(d[:, 2] - d[0, 2])
        d[:, :2] = wrap(d[:, :2])
        s = " ".join(f"{k} mean {d[:,n].mean():+.2f} |max| {np.abs(d[:,n]).max():.2f}"
                     for n, k in enumerate(("roll", "pitch", "yaw")))
        print(f"    {name:8} n={len(i):5}  {s}  deg")

    # ---- C. frame-free relative rotation vs FC ------------------------------------------------
    print("\n[C] relative-rotation error vs FC over windows (deg, |angle(SLAM) - angle(FC)|):")
    for name, t, R in (("BIEVR", tb, Rb), ("FAST-LIO", tf, Rf)):
        i, j = match(t, tq, 0.03)
        Rs, Rq, ts = R[i], Rfc[j], t[i]
        cols = []
        for W in (1.0, 5.0):
            k = np.searchsorted(ts, ts + W)
            ok = k < len(ts)
            err = np.abs(ang(Rs[ok], Rs[k[ok]]) - ang(Rq[ok], Rq[k[ok]]))
            cols.append(f"{W:.0f}s mean {err.mean():.2f} p95 {np.percentile(err, 95):.2f}")
        print(f"    {name:8} " + "   ".join(cols))

    # ---- G. twist frame -------------------------------------------------------------------------
    od = load_bievr_odom(prefix + '_odom.bag')
    if len(od) > 20:
        ti, tj = match(od[:, 0], tb, 0.002)
        vel_fd = np.gradient(pb, tb, axis=0)[tj]
        v_raw = od[ti, 5:8]
        v_rot = np.einsum('nij,nj->ni', q2R(od[ti, 1:5]), v_raw)
        r = lambda x: np.sqrt(((x - vel_fd) ** 2).sum(1).mean())  # noqa: E731
        print(f"\n[G] twist on {len(ti)} odom msgs vs d(position)/dt: as-is rms {r(v_raw):.3f} m/s, "
              f"rotated by pose (body->world) rms {r(v_rot):.3f} m/s   (smaller = the frame it is in)")

    # ---- F. compute ----------------------------------------------------------------------------
    st = step_times(prefix + '.log')
    if st:
        print(f"\n[F] last 'step' timing line: {' '.join(st[-1])}  (see {prefix}.log for the column meaning)")


if __name__ == '__main__':
    main()
