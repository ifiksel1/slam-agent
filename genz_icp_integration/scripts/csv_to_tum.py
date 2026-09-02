#!/usr/bin/env python3
"""Convert genz_traj_sampler CSV -> TUM, so GenZ runs can be scored by the SAME
validated tool used for BIEVR-LIO (slam-agent/scripts/rto_report.py + icp_return_gt.py).

This matters: return-to-origin vs ZERO is only valid on bags that actually close.
rto_report computes true_drift = ||(p_end - p_start) - t_icp||, which removes the real
physical start->end offset, and refuses to score open-loop bags at all.

Usage: csv_to_tum.py <results_dir_glob_suffix> <out_dir>
"""
import csv, glob, os, sys
suffix, outdir = sys.argv[1], sys.argv[2]
os.makedirs(outdir, exist_ok=True)
n = 0
for d in sorted(glob.glob(f'/home/dev/slam-agent/genz_icp_integration/results/*{suffix}')):
    f = os.path.join(d, 'trajectory.csv')
    if not os.path.exists(f):
        continue
    tag = os.path.basename(d)[:-len(suffix)] if suffix else os.path.basename(d)
    rows = list(csv.DictReader(open(f)))
    if len(rows) < 2:
        print(f"  skip (empty): {tag}"); continue
    with open(os.path.join(outdir, f'{tag}_tum.txt'), 'w') as o:
        for r in rows:
            o.write(f"{r['stamp']} {r['x']} {r['y']} {r['z']} {r['qx']} {r['qy']} {r['qz']} {r['qw']}\n")
    n += 1
print(f"wrote {n} TUM files -> {outdir}")
