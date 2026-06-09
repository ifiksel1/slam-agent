#!/usr/bin/env python3
# #3 Degeneracy deep-dive: read the per-scan CSVs from repeatability.sh and answer
# the concrete question for the ArduPilot/EKF side -- "what signal tracks drift, and
# what threshold gates it?" SuperOdom's /state_estimation_health flag was found stuck
# at true even through corridor degeneracy, so the EKF must NOT trust it; instead gate
# on /super_odometry_stats.uncertainty_* (the per-axis Hessian unobservability, ~1 when
# a direction is degenerate).
#
# For each run CSV it reports: per-axis uncertainty distribution, when/where uncertainty
# spikes, plane-match collapse, and the correlation between uncertainty and the rate of
# drift accumulation. It then suggests a gate threshold (knee of the uncertainty
# distribution) and how much of the run would be rejected at that gate.
#
# Usage: analyze_degeneracy.py <run.csv> [run2.csv ...]
import csv, sys, math, statistics as st

def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows

def pct(v, p):
    if not v: return float('nan')
    v = sorted(v); k = (len(v)-1)*p/100.0
    lo = int(math.floor(k)); hi = int(math.ceil(k))
    return v[lo] if lo == hi else v[lo]*(hi-k)+v[hi]*(k-lo)

def analyze(path):
    r = load(path)
    if not r:
        print(f"{path}: no data"); return None
    print(f"\n===== {path}  ({len(r)} scans, {r[-1]['t']:.1f}s) =====")
    axes = [('ux','x'),('uy','y'),('uz','z'),('uroll','roll'),('upitch','pitch'),('uyaw','yaw')]
    print("  per-axis uncertainty (0=well-observed, ~1=degenerate):")
    for key, nm in axes:
        v = [row[key] for row in r]
        hi = sum(1 for x in v if x >= 0.9)
        print(f"    {nm:5s}: mean {st.mean(v):.3f}  p50 {pct(v,50):.3f}  p95 {pct(v,95):.3f}  max {max(v):.3f}  "
              f"scans>=0.9: {hi} ({100*hi/len(v):.0f}%)")
    # plane matches and ICP residual
    pok = [row['plane_ok'] for row in r]; avgd = [row['icp_avg_dist'] for row in r]
    print(f"  plane_match_success: mean {st.mean(pok):.0f}  min {min(pok):.0f}  "
          f"(scans<200: {sum(1 for x in pok if x<200)})")
    print(f"  icp_avg_dist: mean {st.mean(avgd):.3f}  p95 {pct(avgd,95):.3f}  max {max(avgd):.3f}")
    # health flag vs degeneracy: was health ever false while a direction was degenerate?
    hfalse = sum(1 for row in r if row['health'] < 0.5)
    worst = max(max(row['ux'],row['uy'],row['uz'],row['uyaw']) for row in r)
    print(f"  health=false scans: {hfalse}   worst horizontal/yaw uncertainty seen: {worst:.3f}")
    if hfalse == 0 and worst >= 0.9:
        print("  >> CONFIRMS: health stayed TRUE through degeneracy (uncert hit "
              f"{worst:.2f}). EKF must gate on uncertainty, not the health flag.")
    # drift-rate vs uncertainty correlation: per-scan horizontal step vs max(uncert)
    steps, umax = [], []
    for a, b in zip(r, r[1:]):
        steps.append(math.hypot(b['x']-a['x'], b['y']-a['y']))
        umax.append(max(b['ux'], b['uy'], b['uyaw']))
    if len(steps) > 2 and st.pstdev(umax) > 1e-6 and st.pstdev(steps) > 1e-6:
        mu, ms = st.mean(umax), st.mean(steps)
        cov = sum((u-mu)*(s-ms) for u, s in zip(umax, steps))/len(steps)
        corr = cov/(st.pstdev(umax)*st.pstdev(steps))
        print(f"  corr(max-uncertainty, per-scan step) = {corr:+.2f}")
    # suggested gate: 95th pct of the worst horizontal axis -> the knee
    worst_axis = max(axes[:3]+[axes[5]], key=lambda kn: pct([row[kn[0]] for row in r], 95))
    gate_axis_vals = [max(row['ux'], row['uy'], row['uyaw']) for row in r]
    gate = round(pct(gate_axis_vals, 90), 2)
    rej = sum(1 for x in gate_axis_vals if x >= gate)
    print(f"  SUGGESTED EKF GATE: reject vision when max(ux,uy,uyaw) >= ~{gate:.2f}  "
          f"(rejects {rej}/{len(r)} = {100*rej/len(r):.0f}% of scans here; worst axis: {worst_axis[1]})")
    return {'path': path, 'worst': worst, 'hfalse': hfalse, 'gate': gate}

def main():
    if len(sys.argv) < 2:
        print("usage: analyze_degeneracy.py <run.csv> [run2.csv ...]"); sys.exit(1)
    outs = [o for o in (analyze(p) for p in sys.argv[1:]) if o]
    if len(outs) > 1:
        gates = [o['gate'] for o in outs]
        print(f"\n===== ACROSS {len(outs)} RUNS =====")
        print(f"  suggested gate (max(ux,uy,uyaw)) consistent? min {min(gates):.2f} max {max(gates):.2f}")
        print(f"  health stayed true through degeneracy in ALL runs: "
              f"{all(o['hfalse']==0 and o['worst']>=0.9 for o in outs)}")

if __name__ == '__main__':
    main()
