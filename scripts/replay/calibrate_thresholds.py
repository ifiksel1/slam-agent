#!/usr/bin/env python3
"""Derive eig_min / matched_pts drift thresholds from a replay output bag's /fastlio_health.
Usage: calibrate_thresholds.py <out_bag>"""
import sys, numpy as np, rosbag

eig=[]; matched=[]
with rosbag.Bag(sys.argv[1]) as b:
    for tp,m,t in b.read_messages(topics=['/fastlio_health']):
        matched.append(m.data[0]); eig.append(m.data[4])
eig=np.array(eig,float); matched=np.array(matched,float)
N=len(eig); print("scans: %d"%N)

def row(name,a):
    ps=[np.percentile(a,p) for p in (0,1,5,10,25,50,90,100)]
    print("%-12s min=%.0f  p1=%.0f  p5=%.0f  p10=%.0f  p25=%.0f  median=%.0f  p90=%.0f  max=%.0f"%(name,*ps))
row("eig_min",eig); row("matched",matched)

med_e=np.median(eig); med_m=np.median(matched)
warn_e=np.percentile(eig,10); crit_e=np.percentile(eig,1)
warn_m=np.percentile(matched,10)
print("\n-- recommended thresholds (from this bag) --")
print("eig_min  WARN < %.0f  (= %.2f x median)   CRITICAL < %.0f (= %.3f x median)"%(warn_e, warn_e/med_e, crit_e, crit_e/med_e))
print("matched  WARN < %.0f  (= %.2f x median)"%(warn_m, warn_m/med_m))

# sustained-episode check (pick hysteresis K): contiguous scans below WARN eig
below=eig<warn_e
runs=[]; c=0
for v in below:
    c=c+1 if v else 0
    if c: runs.append(c)
longest=max(runs) if runs else 0
print("\nscans below eig WARN: %d (%.1f%%)   longest CONSECUTIVE run: %d scans"%(below.sum(),100*below.mean(),longest))

# corridor-trap check: is matched_pts still high when eig_min is lowest?
low=eig<=np.percentile(eig,5)
print("at lowest-5%% eig_min: matched median=%.0f vs overall median=%.0f -> %s"%(
    np.median(matched[low]), med_m,
    "matched STAYS high (eig_min catches what matched misses)" if np.median(matched[low])>0.5*med_m
    else "matched also drops"))
