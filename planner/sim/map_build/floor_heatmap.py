#!/usr/bin/env python3
"""Floor height per 1 m cell of a sim map PCD (offline check for vertical drift / seams).

  floor_heatmap.py <map.pcd> <out.png> [<meta.json>]

Floor of a cell: the mean of its lowest 5 % of points (cells with >= 20 points below 1.0 m). Prints the
spread; with the meta file the driven path is drawn too.
"""
import json
import sys

import numpy as np


def load(p):
    raw = open(p, "rb").read()
    k = raw.index(b"DATA binary\n") + 12
    return np.frombuffer(raw[k:], dtype="<f4").reshape(-1, 4)


P = load(sys.argv[1])
P = P[P[:, 2] < 1.0]
cell = np.floor(P[:, :2]).astype(np.int64)
ck = (cell[:, 0] + 10000) * 100000 + (cell[:, 1] + 10000)
order = np.lexsort((P[:, 2], ck))
u, first = np.unique(ck[order], return_index=True)
cnt = np.diff(np.append(first, len(order)))
xs, ys, zs = [], [], []
for f, c in zip(first, cnt):
    if c >= 20:
        q = P[order[f:f + max(3, c // 20)]]
        xs.append(np.floor(q[0, 0])); ys.append(np.floor(q[0, 1])); zs.append(q[:, 2].mean())
xs, ys, zs = np.array(xs), np.array(ys), np.array(zs)
print("floor cells %d: z p5 %.2f, p25 %.2f, median %.2f, p75 %.2f, p95 %.2f; |z| < 0.1 m in %.0f %% of cells"
      % (len(zs), *np.percentile(zs, [5, 25, 50, 75, 95]), 100 * np.mean(np.abs(zs) < 0.1)))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
x0, y0 = xs.min(), ys.min()
img = np.full((int(ys.max() - y0) + 1, int(xs.max() - x0) + 1), np.nan)
img[(ys - y0).astype(int), (xs - x0).astype(int)] = zs
fig, ax = plt.subplots(figsize=(14, 14 * img.shape[0] / img.shape[1] + 1))
im = ax.imshow(img, origin="lower", extent=[x0, xs.max() + 1, y0, ys.max() + 1], cmap="coolwarm", vmin=-0.5, vmax=0.5)
plt.colorbar(im, ax=ax, shrink=0.6, label="floor height (m)")
if len(sys.argv) > 3:
    path = np.array(json.load(open(sys.argv[3]))["path"])
    ax.plot(path[:, 0], path[:, 1], "k-", lw=0.6)
ax.set_title("%s: floor height per 1 m cell (lowest 5 %% of points), path in black" % sys.argv[1].split("/")[-1])
plt.tight_layout(); plt.savefig(sys.argv[2], dpi=70)
