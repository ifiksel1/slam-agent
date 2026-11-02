#!/usr/bin/env python3
"""Vertical consistency of a keyframe map's floor (offline check).

  floor_consistency.py <pgo_out_dir> <map.meta.json> [<ground_dz.npy>] [--poses optimized|odom]

Each keyframe sees the floor close by (0.5-3 m horizontally, little range or tilt error). Per 1 m cell
and keyframe: the floor = the keyframe's lowest dense level in that cell (5th percentile + median of the
lowest 10 cm). Cells seen up close by keyframes more than 20 s apart (different passes): the spread
(max - min) of their floor heights is the vertical inconsistency of the map there (drift between passes).
With a build_sim_map ground_dz.npy (the --ground_level correction), the keyframes are shifted by it first.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_sim_map import read_scan  # noqa: E402

pgo, meta = sys.argv[1], json.load(open(sys.argv[2]))
poses = "odom" if "--poses" in sys.argv and sys.argv[sys.argv.index("--poses") + 1] == "odom" else "optimized"
dz = None
if len(sys.argv) > 3 and sys.argv[3].endswith(".npy"):
    dz = np.load(sys.argv[3])[:, 3]
Tk = np.loadtxt(pgo + "/%s_poses.txt" % poses).reshape(-1, 3, 4)
tk = np.loadtxt(pgo + "/times.txt"); tk -= tk[0]
R = np.array(meta["map_from_pgo"]["R"])
rows = []
for i in range(0, len(Tk), 2):
    S = read_scan(os.path.join(pgo, "Scans", "%06d.pcd" % i))[:, :3].astype(np.float64)
    Q = S @ Tk[i][:, :3].T @ R.T
    h = np.hypot(Q[:, 0], Q[:, 1])
    o = Tk[i][:, 3] @ R.T
    if dz is not None:
        o[2] += dz[i]
    W = Q[(h > 0.5) & (h < 3.0) & (Q[:, 2] < -0.05)] + o
    if len(W) < 100:
        continue
    c = np.floor(W[:, :2]).astype(np.int64)
    ck = (c[:, 0] + 10000) * 100000 + (c[:, 1] + 10000)
    order = np.lexsort((W[:, 2], ck))
    u, first = np.unique(ck[order], return_index=True)
    cnt = np.diff(np.append(first, len(order)))
    for key, f, n in zip(u, first, cnt):
        if n < 15:
            continue
        z = W[order[f:f + n], 2]
        lo = np.percentile(z, 5)
        rows.append((key, tk[i], np.median(z[z < lo + 0.1])))
A = np.array(rows)
spreads, where = [], []
for key in np.unique(A[:, 0]):
    s = A[A[:, 0] == key]
    if s[:, 1].max() - s[:, 1].min() < 20:
        continue
    # per pass (keyframes within 20 s): the pass's median floor
    order = np.argsort(s[:, 1]); s = s[order]
    passes = np.split(s, np.where(np.diff(s[:, 1]) > 20)[0] + 1)
    if len(passes) < 2:
        continue
    pz = np.array([np.median(p[:, 2]) for p in passes])
    spreads.append(pz.max() - pz.min())
    where.append((int(key // 100000 - 10000), int(key % 100000 - 10000), [round(float(p[0, 1])) for p in passes]))
sp = np.array(spreads)
keys = np.unique(A[:, 0])
cz = np.array([np.median(A[A[:, 0] == key, 2]) for key in keys])
cx, cy = keys // 100000 - 10000, keys % 100000 - 10000
Mx = np.c_[cx, cy, np.ones(len(cz))]
for _ in range(5):   # robust plane (the map is levelled by a plane anyway); flatness = residual about it
    keep = np.ones(len(cz), bool) if _ == 0 else np.abs(res) < 3 * np.median(np.abs(res)) + 0.05
    coef, *_r = np.linalg.lstsq(Mx[keep], cz[keep], rcond=None)
    res = cz - Mx @ coef
cz = res
print("global floor flatness about a plane (per-cell close-up floor, cells %d): p5..p95 = %.2f m, p1..p99 = %.2f m"
      % (len(cz), np.percentile(cz, 95) - np.percentile(cz, 5), np.percentile(cz, 99) - np.percentile(cz, 1)))
print("%s poses%s: cells seen up close on >= 2 passes: %d; floor spread between passes: median %.2f m, p75 %.2f, p95 %.2f, "
      "max %.2f; > 0.15 m in %.0f %%" % (poses, " + ground_dz" if dz is not None else "", len(sp), np.median(sp),
                                          np.percentile(sp, 75), np.percentile(sp, 95), sp.max(), 100 * np.mean(sp > 0.15)))
for j in np.argsort(sp)[::-1][:6]:
    print("  cell (%d, %d): spread %.2f m, passes at t %s" % (*where[j][:2], sp[j], where[j][2]))
