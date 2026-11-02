#!/usr/bin/env python3
"""SC-PGO run report: loop closures and how much the pose graph moved FAST-LIO's keyframes (offline tool).

  pgo_report.py <pgo_out_dir> <pgo.log> <out.png> [<map.meta.json>]

Loops: SC-PGO's log ("Loop detected! - between i and j", then "ICP fitness test passed ... Add this SC loop"
or a failed test). For each accepted pair (i, j): the closure error of FAST-LIO's odometry, i.e. how far the
relative pose odom_i^-1 odom_j is from the loop-closed optimized_i^-1 optimized_j (translation, rotation).
Correction: |optimized - odom| per keyframe (both start at the first keyframe). The plot is in the map frame
when a build_sim_map meta file is given (its map_from_pgo rotation), else the PGO frame (top view along y).
"""
import json
import re
import sys

import numpy as np

out_dir, log, png = sys.argv[1:4]
O = np.loadtxt(out_dir + "/odom_poses.txt").reshape(-1, 3, 4)
P = np.loadtxt(out_dir + "/optimized_poses.txt").reshape(-1, 3, 4)
t = np.loadtxt(out_dir + "/times.txt")
n = min(len(O), len(P))
O, P, t = O[:n], P[:n], t[:n] - t[0]

text = open(log, "rb").read().decode("utf-8", "replace").replace("\r", "\n")
acc, rej = [], []
cur = None
for line in text.splitlines():
    m = re.search(r"Loop detected! - between (\d+) and (\d+)", line)
    if m:
        cur = (int(m.group(1)), int(m.group(2)))
    elif cur and "Add this SC loop" in line:
        acc.append(cur); cur = None
    elif cur and ("ICP fitness test failed" in line or "Reject" in line or "reject" in line):
        rej.append(cur); cur = None
acc_u = sorted(set(acc))
rej_u = sorted(set(rej) - set(acc_u))


def rel(T, i, j):
    Ri, Rj = T[i][:, :3], T[j][:, :3]
    return Ri.T @ Rj, Ri.T @ (T[j][:, 3] - T[i][:, 3])


rows = []
for i, j in acc_u:
    if max(i, j) >= n:
        continue
    Ro, to = rel(O, i, j)
    Rp, tp = rel(P, i, j)
    dR = Ro.T @ Rp
    ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    rows.append((i, j, t[i], t[j], float(np.linalg.norm(to - tp)), float(ang)))
corr = np.linalg.norm(P[:, :, 3] - O[:, :, 3], axis=1)
print("keyframes %d over %.0f s; SC loops accepted %d (%d unique pairs), rejected by the ICP check %d pairs"
      % (n, t[-1], len(acc), len(acc_u), len(rej_u)))
if rows:
    e = np.array([r[4] for r in rows]); a = np.array([r[5] for r in rows])
    print("odometry closure error at the accepted loops: translation median %.2f m, max %.2f m; "
          "rotation median %.2f deg, max %.2f deg" % (np.median(e), e.max(), np.median(a), a.max()))
    for r in sorted(rows, key=lambda r: -r[4])[:10]:
        print("  loop %4d (t %5.0f s) <-> %4d (t %5.0f s): odom off by %.2f m, %.2f deg" % (r[0], r[2], r[1], r[3], r[4], r[5]))
print("PGO correction per keyframe |optimized - odom|: median %.2f m, p95 %.2f m, max %.2f m (at t %.0f s)"
      % (np.median(corr), np.percentile(corr, 95), corr.max(), t[np.argmax(corr)]))
print("start-end distance: odom %.2f m, optimized %.2f m"
      % (np.linalg.norm(O[-1][:, 3] - O[0][:, 3]), np.linalg.norm(P[-1][:, 3] - P[0][:, 3])))

R = np.eye(3)
if len(sys.argv) > 4:
    R = np.array(json.load(open(sys.argv[4]))["map_from_pgo"]["R"])
    xo, xp = O[:, :, 3] @ R.T, P[:, :, 3] @ R.T
    ax_i, ax_j, frame = 0, 1, "map frame"
else:
    xo, xp = O[:, :, 3], P[:, :, 3]
    ax_i, ax_j, frame = 0, 2, "PGO frame x/z"
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(22, 10))
ax[0].plot(xo[:, ax_i], xo[:, ax_j], "-", color="tab:orange", lw=1, label="FAST-LIO odometry (before)")
ax[0].plot(xp[:, ax_i], xp[:, ax_j], "-", color="tab:blue", lw=1, label="SC-PGO optimized (after)")
for i, j in acc_u:
    if max(i, j) < n:
        ax[0].plot([xp[i, ax_i], xp[j, ax_i]], [xp[i, ax_j], xp[j, ax_j]], "-", color="tab:green", lw=0.6, alpha=0.6)
ax[0].plot([], [], "-", color="tab:green", label="accepted loop (%d pairs)" % len(acc_u))
ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3); ax[0].legend(); ax[0].set_title("keyframe paths, %s (m)" % frame)
ax[1].plot(t, corr, "-", color="tab:blue")
for r in rows:
    ax[1].axvline(r[3], color="tab:green", lw=0.4, alpha=0.4)
ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("|optimized - odom| (m)"); ax[1].grid(alpha=0.3)
ax[1].set_title("PGO correction per keyframe; green: accepted loop times")
plt.tight_layout(); plt.savefig(png, dpi=60)
