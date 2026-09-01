#!/usr/bin/env python3
"""
Compare two FAST-LIO replay output bags (e.g. baseline 'current' vs 'tuned').
Both must come from the SAME input bag, so there's no ground truth -- the comparison is:
  (1) per-run SLAM-health stats from /fastlio_health (residual, ekf time, observability),
  (2) per-run trajectory summary from /Odometry,
  (3) run-to-run trajectory DIVERGENCE (how much the tuning changed the estimate).
Interpretation for confined-space tuning: 'tuned' is better if mean_residual is <= and
eig_min is HIGHER (better-constrained) and matched_pts is higher, while ekf_update_ms stays
well under the ~50 ms / 20 Hz budget.

Usage: compare_bags.py <bagA_current> <bagB_tuned>   (run with ROS sourced)
Optional PNGs saved next to bagB if matplotlib is available.
"""
import sys, math
import rosbag
import numpy as np

def load(bag_path):
    odom = []     # (t, x, y, z)
    health = []   # rows of data[]
    with rosbag.Bag(bag_path) as b:
        for topic, msg, t in b.read_messages(topics=['/Odometry', '/fastlio_health']):
            if topic == '/Odometry':
                p = msg.pose.pose.position
                odom.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))
            else:
                health.append(list(msg.data))
    return np.array(odom, dtype=float), health

def path_len(o):
    if len(o) < 2: return 0.0
    d = np.diff(o[:, 1:4], axis=0)
    return float(np.sum(np.linalg.norm(d, axis=1)))

def hstat(health, idx):
    vals = np.array([r[idx] for r in health if len(r) > idx], dtype=float)
    return vals if vals.size else np.array([0.0])

def summarize(name, o, health):
    print(f"\n===== {name} =====")
    if len(o):
        # NOTE: net displacement / final position deliberately EXCLUDED -- not a correctness
        # metric without ground-truth endpoints. msgs/duration/path_length are descriptive only.
        print(f"  /Odometry msgs : {len(o)}   duration: {o[-1,0]-o[0,0]:.1f}s  (full bag => no dropped scans)")
        print(f"  path length    : {path_len(o):.2f} m   (descriptive, not a verdict)")
    else:
        print("  /Odometry: NO messages")
    if health:
        n = len(health[0])
        matched = hstat(health, 0); resid = hstat(health, 1); ekf = hstat(health, 2)
        print(f"  /fastlio_health rows: {len(health)}  ({n} fields)")
        print(f"  matched_pts     : mean {matched.mean():.0f}  min {matched.min():.0f}")
        print(f"  mean_residual(m): mean {resid.mean():.4f}  p95 {np.percentile(resid,95):.4f}  max {resid.max():.4f}")
        print(f"  ekf_update_ms   : mean {ekf.mean():.2f}  p95 {np.percentile(ekf,95):.2f}  max {ekf.max():.2f}")
        if n >= 7:
            eig_min = hstat(health, 4)
            print(f"  HtDH eig_min    : mean {eig_min.mean():.0f}  min {eig_min.min():.0f}   (higher = better-constrained)")
    else:
        print("  /fastlio_health: NO messages")

def divergence(oa, ob):
    """Position diff between the two runs, aligned by nearest timestamp."""
    if len(oa) < 2 or len(ob) < 2:
        print("\n(divergence: insufficient odometry)"); return
    ta, tb = oa[:, 0], ob[:, 0]
    diffs = []
    for row in oa:
        j = int(np.argmin(np.abs(tb - row[0])))
        if abs(tb[j] - row[0]) < 0.05:   # within 50 ms
            diffs.append(np.linalg.norm(row[1:4] - ob[j, 1:4]))
    if not diffs:
        print("\n(divergence: no time-aligned samples -- different clocks?)"); return
    d = np.array(diffs)
    print("\n===== RUN-TO-RUN TRAJECTORY DIVERGENCE (same input, different params) =====")
    print(f"  aligned samples: {len(d)}")
    print(f"  position diff  : mean {d.mean():.3f} m   max {d.max():.3f} m   final {d[-1]:.3f} m")

def maybe_plot(oa, ob, ha, hb, out_png):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        print("\n(matplotlib not available -> skipping plots)"); return
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    if len(oa): ax[0].plot(oa[:,1], oa[:,2], label='current')
    if len(ob): ax[0].plot(ob[:,1], ob[:,2], label='tuned')
    ax[0].set_title('XY trajectory'); ax[0].set_xlabel('x'); ax[0].set_ylabel('y'); ax[0].axis('equal'); ax[0].legend()
    for h, lbl in ((ha,'current'), (hb,'tuned')):
        if h: ax[1].plot([r[1] for r in h], label=lbl)
    ax[1].set_title('mean_residual'); ax[1].legend()
    for h, lbl in ((ha,'current'), (hb,'tuned')):
        if h and len(h[0])>=7: ax[2].plot([r[4] for r in h], label=lbl)
    ax[2].set_title('HtDH eig_min (observability)'); ax[2].legend()
    fig.tight_layout(); fig.savefig(out_png, dpi=110)
    print(f"\nPlots -> {out_png}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: compare_bags.py <current.bag> <tuned.bag>"); sys.exit(1)
    a, b = sys.argv[1], sys.argv[2]
    oa, ha = load(a)
    ob, hb = load(b)
    summarize(f"CURRENT  ({a})", oa, ha)
    summarize(f"TUNED    ({b})", ob, hb)
    divergence(oa, ob)
    maybe_plot(oa, ob, ha, hb, b.rsplit('.', 1)[0] + "_compare.png")
    print("\nNOTE: no ground truth (same input bag). Judge 'better' by: lower/equal mean_residual,")
    print("higher eig_min & matched_pts, ekf_update_ms still << 50 ms. Divergence just shows how much tuning moved the estimate.")
