#!/usr/bin/env python3
"""Build a MARSIM / L2 closed-loop sim map from one SC-PGO run (offline tool, not robot software).

  build_sim_map.py <pgo_out_dir> <lidar_imu.bag> <out_dir> <name> "<description>" [options]

Input: an SC-PGO run of the robot's own FAST-LIO (slam-pgo:tuned, run_pgo.sh: Scans/%06d.pcd in the IMU
body frame, optimized_poses.txt, times.txt) and the lidar+IMU bag it was run on (for gravity).

Output in out_dir: <name>.pcd (x y z intensity, binary float32, z up along gravity, floor at z = 0, a
6 cm grid like hp_1015.pcd / canusa_2034.pcd), <name>_raw.pcd (the same before cleanup), <name>.meta.json
(path, extent, levels, start pose, cleanup counts), <name>_top.png (top-down view with the path and start).

Steps (the HP / CAN USA maps' pipeline, flights/flight_map.py, plus cleanup):
 1. Up: the accelerometer over the stillest 1 s of the first 10 s, rotated into the map by that
    keyframe's pose; then the floor plane is fitted (lowest points per 1 m cell, robust) and levelled.
 2. Ground: the levelled floor plane at z = 0.
 3. Crop: --margin m around the path in x/y; z from -0.3 to the 99.5th percentile + 0.3.
 4. Moving objects (people): free-space carving. Each keyframe's rays (sensor -> return, stopped
    --ray_stop m short of the return) are walked through a --carve_vox grid. A map voxel that later
    or earlier keyframes saw through in at least --free_min keyframes, and more often than
    --free_ratio times the keyframes that hit it, and that was seen for no longer than --span_max s (people
    pass; walls are seen from many places over a long time, and grazing rays over them must not erase them),
    is removed. Rays stop max(--ray_stop, --ray_stop_frac * range) short of their return. Voxels below --floor_keep m are kept
    (grazing rays over a 0.1-0.3 m floor smear would otherwise eat the floor).
 5. Noise: 6 cm cells hit by a single keyframe with fewer than --nbr_min occupied neighbours
    (26-neighbourhood) are removed.
 6. Path corridor: points within --path_r m of the driven path (keyframes below --ground_path_z only, so a
    flight leaves what it flew over), from --path_zlo m up to --path_zhi m above
    it, are removed (the robot's own body / people walking with it; like hp_1015_carved).
 7. Start: the first keyframe's position on the floor, heading = the body's forward (+z, FAST-LIO's raw
    frame) projected on the floor. sitl_harness / closed_loop start args: start_x start_y start_yaw_deg.
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def read_scan(path):
    raw = open(path, "rb").read()
    k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:k].decode()
    fields = [l for l in hdr.splitlines() if l.startswith("FIELDS")][0].split()[1:]
    n = int([l for l in hdr.splitlines() if l.startswith("POINTS")][0].split()[1])
    assert fields == ["x", "y", "z", "intensity"], fields
    return np.frombuffer(raw[k:k + 16 * n], dtype="<f4").reshape(-1, 4)


def save_pcd(path, P, I, comment):
    A = np.c_[P, I].astype("<f4")
    hdr = ("# .PCD v0.7 - %s\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (comment, len(A), len(A)))
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(np.ascontiguousarray(A).tobytes())


def rot_to_z(u):
    u = u / np.linalg.norm(u)
    z = np.array([0, 0, 1.0])
    v = np.cross(u, z)
    s, c = np.linalg.norm(v), u @ z
    if s < 1e-9:
        return np.eye(3)
    V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + V + V @ V * ((1 - c) / s**2)


def vkey(G):
    """int64 key of integer voxel coords (each within +-2^20)"""
    G = G.astype(np.int64) + 2**20
    return (G[:, 0] << 42) | (G[:, 1] << 21) | G[:, 2]


def gravity_up(bag, tk, Tk):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    A, W, T = [], [], []
    with Reader(bag) as r:
        c = [x for x in r.connections if x.topic == "/lidar_imu"]
        t0 = None
        for con, _, raw in r.messages(connections=c):
            m = ts.deserialize_ros1(raw, con.msgtype)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            t0 = t0 or st
            if st - t0 > 10:
                break
            a, w = m.linear_acceleration, m.angular_velocity
            A.append((a.x, a.y, a.z)); W.append((w.x, w.y, w.z)); T.append(st)
    A, W, T = np.array(A), np.array(W), np.array(T)
    best = None
    for s in np.arange(T[0], T[-1] - 1.0, 0.25):
        k = (T >= s) & (T < s + 1.0)
        score = np.linalg.norm(W[k].std(0)) + np.linalg.norm(A[k].std(0)) / 9.81
        if best is None or score < best[0]:
            best = (score, s, k)
    _, s, k = best
    g_body = A[k].mean(0)
    ki = int(np.argmin(np.abs(tk - (s + 0.5))))
    up = Tk[ki][:, :3] @ g_body
    info = {"stillest_s": round(float(s - T[0]), 2), "acc_norm": round(float(np.linalg.norm(g_body)), 3),
            "gyro_std": round(float(np.linalg.norm(W[k].std(0))), 4)}
    return up / np.linalg.norm(up), info


# --- free-space carving (multiprocessing workers) ---------------------------------------------------
_G = {}


def _init(occ_keys, scans_dir, Tk, R, off, args):
    _G.update(occ=occ_keys, dir=scans_dir, Tk=Tk, R=R, off=off, a=args)


def _carve(idx):
    a = _G["a"]
    occ, Tk, R, off = _G["occ"], _G["Tk"], _G["R"], _G["off"]
    free = np.zeros(len(occ), np.int32)
    rng = np.random.default_rng(idx[0])
    for i in idx:
        S = read_scan(os.path.join(_G["dir"], "%06d.pcd" % i))[:, :3].astype(np.float64)
        if len(S) > a.rays:
            S = S[rng.choice(len(S), a.rays, replace=False)]
        Pw = (S @ Tk[i][:, :3].T + Tk[i][:, 3]) @ R.T + off
        o = Tk[i][:, 3] @ R.T + off
        d = Pw - o
        L = np.linalg.norm(d, axis=1)
        ok = (L > a.ray_stop + 0.5) & (L < a.ray_max)
        stop = np.maximum(a.ray_stop, a.ray_stop_frac * L)
        d, L, stop = d[ok] / L[ok, None], L[ok], stop[ok]
        hits = []
        step = a.carve_vox * 0.5
        # walk in chunks of rays to bound memory
        for c0 in range(0, len(L), 4000):
            dd, LL, SS = d[c0:c0 + 4000], L[c0:c0 + 4000], stop[c0:c0 + 4000]
            n = int(np.ceil(LL.max() / step))
            t = 0.5 + np.arange(n) * step
            m = t[None, :] < (LL - SS)[:, None]
            ri, si = np.nonzero(m)
            pts = o + dd[ri] * t[si, None]
            k = np.unique(vkey(np.floor(pts / a.carve_vox)))
            j = np.searchsorted(occ, k)
            v = j < len(occ)
            j, k = j[v], k[v]
            j = j[occ[j] == k]
            hits.append(j)
        if hits:
            free[np.unique(np.concatenate(hits))] += 1
    return free


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pgo"); ap.add_argument("bag"); ap.add_argument("out"); ap.add_argument("name")
    ap.add_argument("desc")
    ap.add_argument("--vox", type=float, default=0.06)
    ap.add_argument("--margin", type=float, default=10.0)
    ap.add_argument("--carve_vox", type=float, default=0.12)
    ap.add_argument("--rays", type=int, default=12000)
    ap.add_argument("--ray_stop", type=float, default=0.35)
    ap.add_argument("--ray_max", type=float, default=25.0)
    ap.add_argument("--free_min", type=int, default=3)
    ap.add_argument("--free_ratio", type=float, default=2.0)
    ap.add_argument("--span_max", type=float, default=8.0)
    ap.add_argument("--ray_stop_frac", type=float, default=0.06)
    ap.add_argument("--floor_keep", type=float, default=0.2)
    ap.add_argument("--nbr_min", type=int, default=3)
    ap.add_argument("--path_r", type=float, default=0.30)
    ap.add_argument("--path_zlo", type=float, default=0.15)
    ap.add_argument("--ground_path_z", type=float, default=0.5, help="path keyframes below this are on the ground")
    ap.add_argument("--path_zhi", type=float, default=0.5)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--poses", choices=("optimized", "odom"), default="optimized",
                    help="SC-PGO's loop-closed poses, or FAST-LIO's odometry at the same keyframes (before PGO)")
    ap.add_argument("--no_clean", action="store_true", help="skip steps 4-6 (a quick before/after view)")
    ap.add_argument("--t_end", type=float, default=None, help="use keyframes up to this time (s) only")
    ap.add_argument("--ground_level", action="store_true", help="ground constraint: level each keyframe on the floor it sees (one flat floor)")
    ap.add_argument("--ground_win", type=int, default=6, help="ground constraint: median window, keyframes either side")
    ap.add_argument("--low_range_max", type=float, default=6.0, help="with --ground_level: points below --low_z only from keyframes this close (m, horizontal); 0 = off")
    ap.add_argument("--low_z", type=float, default=0.6)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    Tk = np.loadtxt(a.pgo + "/%s_poses.txt" % a.poses).reshape(-1, 3, 4)
    tk = np.loadtxt(a.pgo + "/times.txt")
    n_kf = len(Tk)
    if a.t_end is not None:
        n_kf = int(np.searchsorted(tk - tk[0], a.t_end))
    print("keyframes %d over %.1f s" % (n_kf, tk[n_kf - 1] - tk[0]))

    up, ginfo = gravity_up(a.bag, tk, Tk)
    R = rot_to_z(up)
    print("gravity", ginfo, "up in map", np.round(up, 3))

    ground_info = None
    floor_ref = None
    if a.ground_level:
        # ground constraint for a single flat floor: each keyframe's own scan gives the floor height under it
        # (the lowest dense level 0.5-3 m around the sensor, horizontally). Its drift over the run is vertical
        # drift the loops did not remove (SC-PGO's few loops leave z free); shift each keyframe by the
        # smoothed difference so the floor lies at one height. Flights see the floor too (from higher up).
        fl = np.full(n_kf, np.nan)
        for i in range(n_kf):
            S = read_scan(os.path.join(a.pgo, "Scans", "%06d.pcd" % i))[:, :3].astype(np.float64)
            Q = S @ Tk[i][:, :3].T @ R.T          # sensor-relative, up = +z
            h = np.hypot(Q[:, 0], Q[:, 1])
            q = Q[(h > 0.5) & (h < 3.0) & (Q[:, 2] < -0.05), 2]
            if len(q) < 200:
                continue
            lo = np.percentile(q, 5)
            sel = q[q < lo + 0.10]
            if len(sel) >= 50:
                fl[i] = (Tk[i][:, 3] @ R.T)[2] + np.median(sel)
        good = np.isfinite(fl)
        idx = np.arange(n_kf)
        fi = np.interp(idx, idx[good], fl[good])
        w = a.ground_win
        fs = np.array([np.median(fi[max(0, i - w):i + w + 1]) for i in range(n_kf)])
        floor_ref = float(np.median(fs[:max(10, n_kf // 50)]))
        dz = -(fs - floor_ref)   # every keyframe's floor at the start's level
        for i in range(n_kf):
            Tk[i][:, 3] += R.T @ np.array([0.0, 0.0, dz[i]])
        ground_info = {"keyframes_with_floor": int(good.sum()), "dz_min": round(float(dz.min()), 3),
                       "dz_max": round(float(dz.max()), 3), "dz_p5_p95": np.round(np.percentile(dz, [5, 95]), 3).tolist()}
        print("ground constraint:", ground_info)
        np.save(os.path.join(a.out, a.name + "_ground_dz.npy"), np.c_[tk[:n_kf] - tk[0], fl, fs, dz])

    # accumulate all keyframes into a fine grid (keyframe hit counts per voxel)
    keys, inten, kf_of, lowxy = [], [], [], []
    for i in range(n_kf):
        S = read_scan(os.path.join(a.pgo, "Scans", "%06d.pcd" % i))
        r = np.linalg.norm(S[:, :3], axis=1)
        S = S[(r > 0.4) & (r < a.ray_max)]
        Pw = (S[:, :3].astype(np.float64) @ Tk[i][:, :3].T + Tk[i][:, 3]) @ R.T
        if floor_ref is not None and a.low_range_max > 0:
            # far, grazing floor returns: a 0.2-0.5 m smear (small tilt / range errors over distance) that the
            # sim would render as obstacles. Low points only from keyframes close by; the floor is refilled flat.
            o_i = Tk[i][:, 3] @ R.T
            far = np.hypot(Pw[:, 0] - o_i[0], Pw[:, 1] - o_i[1]) > a.low_range_max
            low = Pw[:, 2] - floor_ref < a.low_z
            lowxy.append(np.unique(np.floor(Pw[low, :2] / 0.3).astype(np.int64), axis=0))
            S, Pw = S[~(far & low)], Pw[~(far & low)]
        k = vkey(np.floor(Pw / a.vox))
        k, first = np.unique(k, return_index=True)
        keys.append(k); inten.append(S[first, 3]); kf_of.append(np.full(len(k), i, np.int32))
    K = np.concatenate(keys); I = np.concatenate(inten)
    KF = np.concatenate(kf_of)
    uk, inv, kfc = np.unique(K, return_inverse=True, return_counts=True)
    Iv = np.bincount(inv, I) / kfc
    tkf = tk[KF] - tk[0]
    t_first = np.full(len(uk), np.inf); np.minimum.at(t_first, inv, tkf)
    t_last = np.full(len(uk), -np.inf); np.maximum.at(t_last, inv, tkf)
    span = t_last - t_first   # how long each voxel was seen (s)
    G = np.stack([(uk >> 42) & (2**21 - 1), (uk >> 21) & (2**21 - 1), uk & (2**21 - 1)], 1) - 2**20
    C = (G + 0.5) * a.vox
    path = np.array([T[:, 3] @ R.T for T in Tk[:n_kf]])
    print("accumulated %d voxels" % len(C))

    # floor plane: lowest 5 % of each 1 m cell below the path, robust fit, level it
    low = C[:, 2] < np.median(path[:, 2]) - 0.05
    Cl = C[low]
    cell = np.floor(Cl[:, :2]).astype(np.int64)
    ck = (cell[:, 0] + 10000) * 100000 + (cell[:, 1] + 10000)
    order = np.lexsort((Cl[:, 2], ck))
    _, first = np.unique(ck[order], return_index=True)
    counts = np.diff(np.append(first, len(order)))
    F = np.array([Cl[order[f:f + max(3, c // 20)]].mean(0) for f, c in zip(first, counts) if c >= 20])
    # keep only cells whose lowest level is near the floor under the path (not a pit or a ledge)
    zf0 = np.median(F[:, 2])
    mask = np.abs(F[:, 2] - zf0) < 0.5
    for _ in range(10):
        A = np.c_[F[mask, 0], F[mask, 1], np.ones(mask.sum())]
        coef, *_ = np.linalg.lstsq(A, F[mask, 2], rcond=None)
        res = F[:, 2] - (coef[0] * F[:, 0] + coef[1] * F[:, 1] + coef[2])
        mask = np.abs(res) < max(0.08, 2.5 * np.median(np.abs(res[mask])))
    nrm = np.array([-coef[0], -coef[1], 1.0]); nrm /= np.linalg.norm(nrm)
    tilt = float(np.degrees(np.arccos(nrm[2])))
    R2 = rot_to_z(nrm)
    C = C @ R2.T; path = path @ R2.T; F2 = F[mask] @ R2.T
    floor_z = float(np.median(F2[:, 2]))
    C[:, 2] -= floor_z; path[:, 2] -= floor_z
    Rt = R2 @ R
    off = np.array([0, 0, -floor_z])   # world_map = Rt @ p_pgo + off
    print("floor fit on %d of %d cells, residual tilt %.2f deg, floor residual median %.3f m"
          % (mask.sum(), len(F), tilt, float(np.median(np.abs(res[mask])))))

    # height levels
    h, e = np.histogram(C[:, 2], bins=np.arange(-1.0, C[:, 2].max() + 0.05, 0.05), weights=kfc)
    peaks = []
    for i in np.argsort(h)[::-1]:
        z = e[i] + 0.025
        if all(abs(z - p) > 0.5 for p in peaks):
            peaks.append(z)
        if len(peaks) == 4:
            break
    top = float(np.percentile(C[:, 2], 99.5))
    lo, hi = path[:, :2].min(0) - a.margin, path[:, :2].max(0) + a.margin
    keep = (C[:, 2] > -0.3) & (C[:, 2] < top + 0.3) & np.all((C[:, :2] >= lo) & (C[:, :2] <= hi), axis=1)
    C, Iv, kfc, G, span = C[keep], Iv[keep], kfc[keep], G[keep], span[keep]
    ilo, ihi = np.percentile(Iv, [2, 98])
    I8 = np.clip((Iv - ilo) / max(ihi - ilo, 1e-6) * 255, 0, 255).astype(np.float32)
    save_pcd(os.path.join(a.out, a.name + "_raw.pcd"), C, I8, a.desc + " (uncleaned)")
    print("cropped: %d points; levels %s; top %.2f m" % (len(C), np.round(sorted(peaks), 2), top))
    stats = {"raw_points": int(len(C))}

    if a.no_clean:
        save_pcd(os.path.join(a.out, a.name + ".pcd"), C, I8, a.desc)
        _plot(a, C, path, None)
        return
    # 4. free-space carving on a coarser grid
    cg = vkey(np.floor(C / a.carve_vox))
    occ, cinv = np.unique(cg, return_inverse=True)
    hitkf = np.zeros(len(occ), np.int64)
    np.maximum.at(hitkf, cinv, kfc)   # keyframes that hit a carve voxel: the most of its fine voxels
    span_c = np.zeros(len(occ)); np.maximum.at(span_c, cinv, span)   # the longest any part of it was seen
    chunks = [list(range(i, n_kf, a.workers * 4)) for i in range(a.workers * 4)]
    with mp.Pool(a.workers, initializer=_init,
                 initargs=(occ, os.path.join(a.pgo, "Scans"), Tk, Rt, off, a)) as pool:
        free = np.sum(pool.map(_carve, chunks), axis=0)
    dyn_c = (free >= a.free_min) & (free > a.free_ratio * hitkf) & (span_c <= a.span_max)
    dyn = dyn_c[cinv] & (C[:, 2] >= a.floor_keep)
    stats["moving_removed"] = int(dyn.sum())
    print("free-space carving: %d of %d points removed (%d carve voxels)" % (dyn.sum(), len(C), dyn_c.sum()))
    keep = ~dyn

    # 5. isolated single-keyframe specks
    fk = vkey(G)
    sk = np.sort(fk[keep])
    nb = np.zeros(len(C), np.int32)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                q = vkey(G + np.array([dx, dy, dz]))
                j = np.clip(np.searchsorted(sk, q), 0, len(sk) - 1)
                nb += (sk[j] == q)
    speck = keep & (kfc <= 1) & (nb < a.nbr_min)
    stats["specks_removed"] = int(speck.sum())
    keep &= ~speck
    print("isolated specks removed: %d" % speck.sum())

    # 6. path corridor
    from scipy.spatial import cKDTree
    gnd = path[path[:, 2] < a.ground_path_z]      # keyframes on the ground only: not the flights' footprints
    pt = cKDTree(gnd[:, :2])
    dist, j = pt.query(C[:, :2])
    rel = C[:, 2] - gnd[j, 2]
    corr = keep & (dist < a.path_r) & (C[:, 2] > a.path_zlo) & (rel < a.path_zhi)
    stats["corridor_removed"] = int(corr.sum())
    keep &= ~corr
    print("path corridor removed: %d" % corr.sum())

    C, I8 = C[keep], I8[keep]
    if lowxy:
        # flat floor at z = 0 over the footprint (0.3 m cells where any keyframe saw something low), on the
        # 6 cm grid, where the map has no floor point (|z| < 0.1) yet: SITL's ground is a flat z = 0 too
        F = np.unique(np.concatenate(lowxy), axis=0)
        sub = np.arange(0, 0.3 - 1e-6, a.vox)
        gx, gy = np.meshgrid(sub, sub)
        Pf = (F[:, None, :] * 0.3 + np.c_[gx.ravel(), gy.ravel()][None]).reshape(-1, 2)
        Pf = np.c_[Pf, np.zeros(len(Pf))]
        Pf = Pf @ R2.T[:, :]  # into the levelled frame (R2 is a sub-degree tilt)
        Pf[:, 2] = 0.0
        inside = np.all((Pf[:, :2] >= lo) & (Pf[:, :2] <= hi), axis=1)
        Pf = Pf[inside]
        have = set(vkey(np.floor(C[np.abs(C[:, 2]) < 0.1] / a.vox) * np.array([1, 1, 0])).tolist())
        kf = vkey(np.floor(Pf / a.vox) * np.array([1, 1, 0]))
        Pf = Pf[~np.isin(kf, np.fromiter(have, np.int64))]
        Pf[:, :2] = (np.floor(Pf[:, :2] / a.vox) + 0.5) * a.vox
        Pf = np.unique(Pf, axis=0)
        stats["floor_filled"] = int(len(Pf))
        print("flat floor filled: %d points" % len(Pf))
        C = np.r_[C, Pf]; I8 = np.r_[I8, np.full(len(Pf), 40, np.float32)]
    stats["points"] = int(len(C))
    save_pcd(os.path.join(a.out, a.name + ".pcd"), C, I8, a.desc)

    # 7. start pose: first keyframe, body +z (forward) on the floor
    fwd = Rt @ (Tk[0][:, :3] @ np.array([0, 0, 1.0]))
    yaw0 = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
    start = [round(float(path[0, 0]), 2), round(float(path[0, 1]), 2), round(yaw0, 1)]

    step = np.linalg.norm(np.diff(path, axis=0), axis=1)
    meta = {"name": a.name, "desc": a.desc, "points": int(len(C)), "voxel_m": a.vox,
            "extent_min": np.round(C.min(0), 2).tolist(), "extent_max": np.round(C.max(0), 2).tolist(),
            "levels_m": [float(x) for x in np.round(sorted(peaks), 2)], "z_top_995_m": round(top, 2),
            "floor_tilt_removed_deg": round(tilt, 2), "gravity": ginfo,
            "keyframes": n_kf, "duration_s": round(float(tk[n_kf - 1] - tk[0]), 1),
            "path_m": round(float(step.sum()), 1), "path_height_m": [round(float(path[:, 2].min()), 2),
                                                                    round(float(path[:, 2].max()), 2)],
            "start_xy_yawdeg": start, "map_from_pgo": {"R": np.round(Rt, 6).tolist(), "t": off.tolist()},
            "cleanup": stats, "ground_constraint": ground_info, "params": {k: v for k, v in vars(a).items() if k not in ("pgo", "bag", "out")},
            "path": np.round(path[:, :3], 2).tolist()}
    json.dump(meta, open(os.path.join(a.out, a.name + ".meta.json"), "w"))
    print(json.dumps({k: v for k, v in meta.items() if k not in ("path", "params", "map_from_pgo")}))

    _plot(a, C, path, start)


def _plot(a, C, path, start):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    band = (C[:, 2] > 0.3) & (C[:, 2] < 1.8)
    fig, ax = plt.subplots(figsize=(14, 14 * (np.ptp(C[:, 1]) + 1) / (np.ptp(C[:, 0]) + 1) + 0.5))
    Hf, xe, ye = np.histogram2d(C[:, 0], C[:, 1], bins=[np.arange(C[:, 0].min(), C[:, 0].max() + 0.1, 0.1),
                                                        np.arange(C[:, 1].min(), C[:, 1].max() + 0.1, 0.1)])
    Hb, _, _ = np.histogram2d(C[band, 0], C[band, 1], bins=[xe, ye])
    img = np.ones(Hf.T.shape + (3,))
    img[Hf.T > 0] = (0.85, 0.85, 0.85)
    img[Hb.T > 0] = (0.1, 0.1, 0.1)
    ax.imshow(img, origin="lower", extent=[xe[0], xe[-1], ye[0], ye[-1]], aspect="equal")
    ax.plot(path[:, 0], path[:, 1], "-", color="tab:red", lw=1.2, label="driven path")
    if start is not None:
        ax.plot(*start[:2], "o", color="tab:green", ms=10, label="start (%.2f, %.2f, %.1f deg)" % tuple(start))
        ax.arrow(start[0], start[1], np.cos(np.radians(start[2])), np.sin(np.radians(start[2])), width=0.08,
             color="tab:green")
    ax.set_title("%s: obstacles 0.3-1.8 m (black), floor/other (grey); map frame, m, 2 m grid" % a.name)
    ax.set_xticks(np.arange(np.floor(xe[0]), xe[-1], 2.0)); ax.set_yticks(np.arange(np.floor(ye[0]), ye[-1], 2.0))
    ax.grid(alpha=0.25); ax.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(os.path.join(a.out, a.name + "_top.png"), dpi=80)


if __name__ == "__main__":
    main()
