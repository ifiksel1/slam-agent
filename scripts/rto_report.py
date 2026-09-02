#!/usr/bin/env python3
"""
Merge BIEVR-LIO TUM trajectories with ICP ground-truth offsets into one table.

For each bag:
  RTO_bievr   = ||end - start|| of the BIEVR trajectory (assumes start==end)
  ICP_offset  = true physical start->end offset from icp_return_gt.py
  true_drift  = ||bievr_end_vec - icp_translation_vec||   (placement removed)

If the bag is open-loop (ICP closed_loop=false), true_drift is not computed --
there is no origin to return to, so RTO/drift are meaningless there.
"""
import argparse, glob, json, os
import numpy as np


def bievr_end_start(tum_path):
    d = np.loadtxt(tum_path)
    if d.ndim == 1:
        d = d[None, :]
    return d[0, 1:4], d[-1, 1:4], len(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tumdir', required=True, help='dir with <tag>_tum.txt')
    ap.add_argument('--icpdir', required=True, help='dir with <tag>*.json from icp_return_gt')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    icp = {}
    for jf in glob.glob(os.path.join(args.icpdir, '*.json')):
        try:
            j = json.load(open(jf))
        except Exception:
            continue
        tag = os.path.basename(j.get('bag', jf)).replace('.bag', '')
        icp[tag] = j

    rows = []
    for tum in sorted(glob.glob(os.path.join(args.tumdir, '*_tum.txt'))):
        tag = os.path.basename(tum).replace('_tum.txt', '')
        try:
            p0, pN, n = bievr_end_start(tum)
        except Exception as e:
            rows.append({'bag': tag, 'error': f'tum read: {e}'})
            continue
        rto = float(np.linalg.norm(pN - p0))
        row = {'bag': tag, 'poses': n, 'rto_bievr_m': round(rto, 4)}
        j = icp.get(tag)
        if j and 'icp_translation_xyz' in j:
            inl = j['icp_inlier_frac']
            off = j['icp_offset_m']
            # A return-to-origin needs BOTH scene overlap (inlier frac) AND a
            # small physical offset. High overlap + large offset = a loop that
            # revisits the start scene but ended metres away (not a return).
            # rmse is not used: single-scan rmse runs high regardless.
            if inl < 0.55:
                tier = 'open'          # no overlap -> one-way sweep
            elif off >= 0.60:
                tier = 'drifted'       # overlaps origin scene but ended far
            elif inl >= 0.85:
                tier = 'CLOSED'        # tight return, cm-level GT
            else:
                tier = 'near'          # returned <0.6m, often heading changed
            row['tier'] = tier
            row['icp_offset_m'] = j['icp_offset_m']
            row['icp_yaw_deg'] = j.get('icp_yaw_deg')
            row['icp_inlier_frac'] = inl
            if tier != 'open':
                t_icp = np.array(j['icp_translation_xyz'])
                row['true_drift_m'] = round(float(np.linalg.norm(pN - p0 - t_icp)), 4)
            else:
                row['true_drift_m'] = None  # not a return-to-origin
        else:
            row['tier'] = None
            row['note'] = 'no ICP GT'
        rows.append(row)

    # table
    hdr = (f"{'bag':<46} {'poses':>6} {'RTO_bievr':>9} {'ICP_off':>8} {'yaw':>7} "
           f"{'inlier':>6} {'tier':>7} {'true_drift':>10}")
    print(hdr); print('-' * len(hdr))
    for r in rows:
        if 'error' in r:
            print(f"{r['bag']:<46} ERROR: {r['error']}"); continue
        td = r.get('true_drift_m')
        td = f"{td:.4f}" if isinstance(td, float) else ('-' if r.get('tier') == 'open' else '?')
        yaw = r.get('icp_yaw_deg')
        yaw = f"{yaw:.0f}" if isinstance(yaw, (int, float)) else '-'
        print(f"{r['bag']:<46} {r['poses']:>6} {r.get('rto_bievr_m',0):>9.4f} "
              f"{str(r.get('icp_offset_m','-')):>8} {yaw:>7} "
              f"{str(r.get('icp_inlier_frac','-')):>6} {str(r.get('tier')):>7} {td:>10}")

    if args.out:
        json.dump(rows, open(args.out, 'w'), indent=2)
        print(f"\nwrote {args.out}")


if __name__ == '__main__':
    main()
