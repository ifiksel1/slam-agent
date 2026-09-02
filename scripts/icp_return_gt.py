#!/usr/bin/env python3
"""
ICP return-to-origin ground truth.

The problem: return-to-origin (RTO) accuracy of a SLAM run is only meaningful if
the vehicle's PHYSICAL start and end poses are actually coincident. On these bags
the operator isn't sure -- start/end may differ by up to ~10 cm. That places a
~10 cm noise floor on any RTO metric derived from "assume start == end".

The fix implemented here: don't assume. The scene (737 airframe + hangar) is
static, so we ICP-register the FIRST LiDAR scan onto the LAST LiDAR scan and read
the physical start->end offset straight out of the sensor data (~1-2 cm on dense
Ouster returns). That offset becomes the ground truth the SLAM endpoint is scored
against, decoupling true drift from placement error.

It also doubles as a loop-closure detector: if first/last scans don't overlap
(poor ICP fitness), the vehicle did NOT return to origin -> the bag is open-path
and RTO is unscoreable (flagged, not silently reported as a huge drift).

Runs on the host with ROS1 Noetic python + numpy/scipy. No open3d/PCL needed.

Convention:
  target = LAST  scan (stays fixed)
  source = FIRST scan (moved onto target)
  T maps first-frame points into last-frame; translation(T) = displacement of the
  sensor from its START pose to its END pose, expressed in the START frame.
  A perfect SLAM run's reported end position equals translation(T).
"""
import argparse, json, sys
import numpy as np
from scipy.spatial import cKDTree

import rosbag
from sensor_msgs import point_cloud2


def cloud_to_xyz(msg, rmin, rmax):
    pts = np.array(list(point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
    if pts.size == 0:
        return pts.reshape(0, 3)
    r = np.linalg.norm(pts, axis=1)
    keep = (r > rmin) & (r < rmax) & np.isfinite(r)
    return pts[keep]


def voxel_downsample(pts, voxel):
    if pts.shape[0] == 0 or voxel <= 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def icp(src, tgt, max_corr, iters=50, tol=1e-7):
    """Point-to-point ICP (Kabsch per iter). Returns (T 4x4, rmse, inlier_frac)."""
    tree = cKDTree(tgt)
    T = np.eye(4)
    cur = src.copy()
    prev = None
    inlier_frac = 0.0
    rmse = float('inf')
    for _ in range(iters):
        d, idx = tree.query(cur, workers=-1)
        m = d < max_corr
        n = int(m.sum())
        inlier_frac = n / len(cur)
        if n < 50:
            break
        A = cur[m]
        B = tgt[idx[m]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        Ti = np.eye(4); Ti[:3, :3] = R; Ti[:3, 3] = t
        cur = (R @ cur.T).T + t
        T = Ti @ T
        rmse = float(np.sqrt((d[m] ** 2).mean()))
        if prev is not None and abs(prev - rmse) < tol:
            break
        prev = rmse
    return T, rmse, inlier_frac


def yaw_deg(R):
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def collect(bag_path, topic, nstart, nend, rmin, rmax):
    """Single pass: grab first nstart raw msgs, keep a ring buffer of last nend."""
    first, ring = [], []
    total = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            total += 1
            if len(first) < nstart:
                first.append(msg)
            ring.append(msg)
            if len(ring) > nend:
                ring.pop(0)
    start_pts = np.vstack([cloud_to_xyz(m, rmin, rmax) for m in first]) if first else None
    end_pts = np.vstack([cloud_to_xyz(m, rmin, rmax) for m in ring]) if ring else None
    return start_pts, end_pts, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/ouster/points')
    ap.add_argument('--nstart', type=int, default=5, help='scans stacked at start')
    ap.add_argument('--nend', type=int, default=5, help='scans stacked at end')
    ap.add_argument('--voxel', type=float, default=0.10, help='downsample voxel (m)')
    ap.add_argument('--rmin', type=float, default=0.6)
    ap.add_argument('--rmax', type=float, default=60.0)
    ap.add_argument('--coarse', type=float, default=2.0, help='coarse max corr (m)')
    ap.add_argument('--yaw-search', default='0,45,90,135,180,-45,-90,-135',
                    help='comma deg yaw inits for coarse ICP (heading-change robust)')
    ap.add_argument('--fine', type=float, default=0.5, help='fine max corr (m)')
    # closed-loop verdict thresholds
    ap.add_argument('--min-inlier', type=float, default=0.55)
    ap.add_argument('--max-rmse', type=float, default=0.20)
    ap.add_argument('--json', default=None)
    args = ap.parse_args()

    start_pts, end_pts, total = collect(
        args.bag, args.topic, args.nstart, args.nend, args.rmin, args.rmax)
    if start_pts is None or end_pts is None or len(start_pts) < 100 or len(end_pts) < 100:
        out = {'bag': args.bag, 'error': 'insufficient points',
               'total_scans': total,
               'n_start': 0 if start_pts is None else len(start_pts),
               'n_end': 0 if end_pts is None else len(end_pts)}
        print(json.dumps(out, indent=2))
        if args.json:
            open(args.json, 'w').write(json.dumps(out, indent=2))
        return

    src = voxel_downsample(start_pts, args.voxel)  # FIRST -> moved
    tgt = voxel_downsample(end_pts, args.voxel)    # LAST  -> fixed

    # Multi-yaw coarse init: a true return-to-origin may end on a different
    # heading, which identity-initialized ICP can't recover. Try several yaws,
    # keep the coarse basin with the best inlier fraction, then refine.
    yaws = [float(y) for y in args.yaw_search.split(',')]
    best = None
    for y in yaws:
        c, s = np.cos(np.radians(y)), np.sin(np.radians(y))
        Y = np.eye(4)
        Y[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        src_y = (Y[:3, :3] @ src.T).T
        Tc, _, inl_c = icp(src_y, tgt, args.coarse, iters=40)
        Tc = Tc @ Y
        if best is None or inl_c > best[1]:
            best = (Tc, inl_c)
    T = best[0]
    src2 = (T[:3, :3] @ src.T).T + T[:3, 3]
    Tf, rmse, inlier = icp(src2, tgt, args.fine, iters=60)
    T = Tf @ T

    trans = T[:3, 3]
    offset = float(np.linalg.norm(trans))
    yaw = yaw_deg(T[:3, :3])
    closed = bool(inlier >= args.min_inlier and rmse <= args.max_rmse)

    out = {
        'bag': args.bag,
        'total_scans': total,
        'points_src': int(len(src)), 'points_tgt': int(len(tgt)),
        'icp_offset_m': round(offset, 4),           # true physical start->end distance
        'icp_translation_xyz': [round(float(v), 4) for v in trans],
        'icp_yaw_deg': round(yaw, 3),
        'icp_rmse_m': round(rmse, 4),
        'icp_inlier_frac': round(inlier, 3),
        'closed_loop': closed,
        'note': ('closed loop: icp_offset_m is the true start->end offset; score '
                 'SLAM against it' if closed else
                 'POOR OVERLAP: vehicle likely did NOT return to origin -> RTO '
                 'unscoreable on this bag (open path)'),
        'T': [[round(float(v), 6) for v in row] for row in T],
    }
    print(json.dumps(out, indent=2))
    if args.json:
        open(args.json, 'w').write(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
