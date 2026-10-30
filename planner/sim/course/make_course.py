#!/usr/bin/env python3
"""Gate L2 obstacle course: add obstacles to a real map as surface points, saved as a new PCD.

  make_course.py <base.pcd> <course.json> <out.pcd>

course.json: {"spacing": m (optional, default: the base map's own median point spacing),
              "obstacles": [ {"name": ..., "type": "box", "c": [x, y], "size": [sx, sy, h], "yaw_deg": 0},
                             {"name": ..., "type": "pole", "c": [x, y], "r": m, "h": m},
                             {"name": ..., "type": "wall_gap", "x": x, "y0": y, "y1": y, "gap": [y_lo, y_hi],
                              "thick": m, "h": m} ],
              "floor": [ {"name": ..., "c": [x, y], "size": [sx, sy], "yaw_deg": 0, "z": z} ]}
                                                           (map frame, metres; h above the local floor)

"floor" (optional): flat floor patches where the base map has none (a hole in the scan), at the given
map z, sampled on the same square grid. They are floor, not obstacles: listed under "floor" in the
obstacles file, so the clearance check against the added obstacles leaves them out.

Each obstacle stands on the map's local floor: its bottom is the 10th percentile of the base map's
points below 0.4 m within 0.6 m of its footprint, less 0.05 m (the merged maps' floor is a 0.1-0.3 m
smear, so a bottom at z = 0 could float above it or sink into it). Every face is sampled on a square
grid at the spacing (top included, bottom left out), so the render node's 5 cm distance field sees a
closed surface. The points get intensity 200 (the base map keeps its own).

Writes <out.pcd> and <out>.obstacles.json: every obstacle as the boxes and cylinders actually placed
(with the floor z used), for clearance checks against the added geometry alone.
"""
import json
import math
import sys

import numpy as np
from scipy.spatial import cKDTree


def load_pcd(path):
    raw = open(path, "rb").read()
    k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:k].decode()
    fields = [l for l in hdr.splitlines() if l.startswith("FIELDS")][0].split()[1:]
    n = int([l for l in hdr.splitlines() if l.startswith("POINTS")][0].split()[1])
    assert fields == ["x", "y", "z", "intensity"], fields
    return hdr, np.frombuffer(raw[k:k + 16 * n], dtype="<f4").reshape(-1, 4).copy()


def save_pcd(path, P, comment):
    hdr = ("# .PCD v0.7 - %s\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (comment, len(P), len(P)))
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(np.ascontiguousarray(P, dtype="<f4").tobytes())


def grid(a, b, s):
    """samples from a to b inclusive, at most s apart"""
    n = max(1, int(math.ceil((b - a) / s)))
    return np.linspace(a, b, n + 1)


def box_points(c, size, yaw, z0, s):
    """surface of an upright box: footprint centre c, size (sx, sy, h), bottom z0, no bottom face"""
    hx, hy, h = size[0] / 2, size[1] / 2, size[2]
    xs, ys, zs = grid(-hx, hx, s), grid(-hy, hy, s), grid(0, h, s)
    pts = []
    for x in (-hx, hx):
        Y, Z = np.meshgrid(ys, zs)
        pts.append(np.c_[np.full(Y.size, x), Y.ravel(), Z.ravel()])
    for y in (-hy, hy):
        X, Z = np.meshgrid(xs, zs)
        pts.append(np.c_[X.ravel(), np.full(X.size, y), Z.ravel()])
    X, Y = np.meshgrid(xs, ys)
    pts.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, h)])
    P = np.vstack(pts)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([[cy, -sy], [sy, cy]])
    P[:, :2] = P[:, :2] @ R.T + c
    P[:, 2] += z0
    return P


def pole_points(c, r, h, z0, s):
    n = max(8, int(math.ceil(2 * math.pi * r / s)))
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    zs = grid(0, h, s)
    A, Z = np.meshgrid(a, zs)
    side = np.c_[c[0] + r * np.cos(A.ravel()), c[1] + r * np.sin(A.ravel()), z0 + Z.ravel()]
    top = [np.c_[c[0] + rr * np.cos(a), c[1] + rr * np.sin(a), np.full(n, z0 + h)] for rr in grid(0, r, s)[1:]]
    return np.vstack([side, np.array([[c[0], c[1], z0 + h]])] + top)


def floor_z(P, tree, c, half):
    """10th percentile of the base map's points below 0.4 m within 0.6 m of the footprint"""
    idx = tree.query_ball_point([c[0], c[1]], half + 0.6)
    z = P[idx, 2]
    z = z[z < 0.4]
    return float(np.percentile(z, 10)) - 0.05 if len(z) else -0.05


def main():
    base, spec_path, out = sys.argv[1:4]
    _, M = load_pcd(base)
    spec = json.load(open(spec_path))
    tree2 = cKDTree(M[:, :2].astype(np.float64))
    if "spacing" in spec:
        s = float(spec["spacing"])
    else:
        t3 = cKDTree(M[:, :3].astype(np.float64))
        d, _ = t3.query(M[::200, :3].astype(np.float64), k=2)
        s = float(np.median(d[:, 1]))
    print("base %s: %d points; obstacle spacing %.3f m" % (base, len(M), s))
    placed, added = [], []
    for o in spec["obstacles"]:
        if o["type"] == "box":
            c = np.array(o["c"], float)
            z0 = floor_z(M, tree2, c, 0.5 * math.hypot(*o["size"][:2]))
            yaw = math.radians(o.get("yaw_deg", 0.0))
            pts = box_points(c, o["size"], yaw, z0, s)
            placed.append({"name": o["name"], "shape": "box", "c": c.tolist(), "size": o["size"], "yaw_deg": o.get("yaw_deg", 0.0), "z0": z0})
        elif o["type"] == "pole":
            c = np.array(o["c"], float)
            z0 = floor_z(M, tree2, c, o["r"])
            pts = pole_points(c, o["r"], o["h"], z0, s)
            placed.append({"name": o["name"], "shape": "pole", "c": c.tolist(), "r": o["r"], "h": o["h"], "z0": z0})
        elif o["type"] == "wall_gap":
            # a straight wall along y at x, from y0 to y1, open between gap[0] and gap[1]
            pts = []
            for i, (a, b) in enumerate(((o["y0"], o["gap"][0]), (o["gap"][1], o["y1"]))):
                c = np.array([o["x"], (a + b) / 2])
                size = [o["thick"], b - a, o["h"]]
                z0 = floor_z(M, tree2, c, 0.5 * math.hypot(*size[:2]))
                pts.append(box_points(c, size, 0.0, z0, s))
                placed.append({"name": "%s_%s" % (o["name"], "south" if i == 0 else "north"), "shape": "box",
                               "c": c.tolist(), "size": size, "yaw_deg": 0.0, "z0": z0})
            pts = np.vstack(pts)
        else:
            raise ValueError(o["type"])
        added.append(pts)
        print("  %-10s %-8s %6d points, floor z %.2f" % (o["name"], o["type"], len(pts), placed[-1]["z0"]))
    floors = []
    for f in spec.get("floor", []):
        hx, hy = f["size"][0] / 2, f["size"][1] / 2
        X, Y = np.meshgrid(grid(-hx, hx, s), grid(-hy, hy, s))
        yaw = math.radians(f.get("yaw_deg", 0.0))
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        xy = np.c_[X.ravel(), Y.ravel()] @ R.T + np.array(f["c"], float)
        added.append(np.c_[xy, np.full(len(xy), float(f["z"]))])
        floors.append({"name": f["name"], "c": f["c"], "size": f["size"], "yaw_deg": f.get("yaw_deg", 0.0), "z": f["z"]})
        print("  %-10s %-8s %6d points, z %.2f" % (f["name"], "floor", len(xy), f["z"]))
    A = np.vstack(added)
    A = np.c_[A, np.full(len(A), 200.0)]
    save_pcd(out, np.vstack([M, A.astype(np.float32)]),
             "%s + Gate L2 course %s (%d obstacle points)" % (base.split("/")[-1], spec_path.split("/")[-1], len(A)))
    meta = {"base": base, "n_base": len(M), "n_added": len(A), "spacing": s, "obstacles": placed}
    if floors:
        meta["floor"] = floors
    json.dump(meta, open(out + ".obstacles.json", "w"), indent=1)
    print("wrote %s (%d + %d points) and %s.obstacles.json" % (out, len(M), len(A), out))


if __name__ == "__main__":
    main()
