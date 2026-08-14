#!/usr/bin/env python3
"""
Analyze the confined-space param sweep: compare S1..S6 vs S0 baseline across all bags.
Reads /mnt/usb/replay_sweep/<config>/<bag>.bag (each has /fastlio_health + /Odometry).

Health cols: [0]=matched_pts [1]=res_mean [2]=ekf_update_ms [3]=feats_down
             [4]=eig_min [5]=eig_mid [6]=eig_max (+3 extra in 10-col build)

Objective = confined-space observability: raise matched_pts & eig_min, and REDUCE
"starvation exposure" (fraction of scans below the trigger floors). Compute time is
reported but flagged NOT real-time-representative (offline, container-contended).
"""
import rosbag, numpy as np, glob, os, sys

ROOT="/mnt/usb/replay_sweep"
ORDER=["S0_baseline","S1","S2","S3","S4","S5","S6"]
STARVED={"07_17_2026_19_43_05","07_17_2026_19_37_41"}  # confined/feature-poor bags

def load(B):
    H=[]; O=[]
    for tp,m,t in rosbag.Bag(B).read_messages(topics=["/fastlio_health","/Odometry"]):
        if tp=="/fastlio_health":
            d=m.data
            if len(d)>=7:                       # need cols 0..6; ignore malformed/short rows
                H.append([d[0],d[1],d[2],d[3],d[4],d[5],d[6]])
        else:
            p=m.pose.pose.position; O.append((p.x,p.y,p.z))
    return np.array(H,dtype=float), np.array(O,dtype=float)

def run_metrics(H,O):
    m=H[:,0]; res=H[:,1]; ms=H[:,2]; feats=H[:,3]; emin=H[:,4]; emax=H[:,6]
    path=float(np.sum(np.linalg.norm(np.diff(O,axis=0),axis=1))) if len(O)>1 else 0.0
    return dict(n=len(H), matched_mean=m.mean(), matched_min=m.min(), matched_p5=np.percentile(m,5),
        eig_mean=emin.mean(), eig_min=emin.min(), feats_mean=feats.mean(),
        starv_matched=100*(m<300).mean(), crit_matched=100*(m<150).mean(),
        cond_med=float(np.median(emin/np.maximum(emax,1))),
        ms_p95=np.percentile(ms,95), ms_max=ms.max(), path=path)

def main():
    # collect per (config,bag)
    data={}  # config -> bag -> metrics
    bags=set()
    for cfg in ORDER:
        d=os.path.join(ROOT,cfg)
        if not os.path.isdir(d): continue
        data[cfg]={}
        for b in sorted(glob.glob(d+"/*.bag")):
            tag=os.path.basename(b)[:19]
            try:
                H,O=load(b)
                if len(H): data[cfg][tag]=run_metrics(H,O); bags.add(tag)
            except Exception as e:
                print("  ! failed %s/%s: %s"%(cfg,tag,e))
    bags=sorted(bags)

    def agg(cfg,keys,subset=None):
        bl=[t for t in data.get(cfg,{}) if (subset is None or t in subset)]
        out={}
        for k in keys:
            v=[data[cfg][t][k] for t in bl]
            out[k]=np.mean(v) if v else float("nan")
        return out

    KEYS=["matched_mean","matched_min","eig_mean","feats_mean","starv_matched","crit_matched","cond_med","ms_p95","ms_max"]

    for label,subset in [("ALL 5 BAGS",None),("STARVED bags only (confined proxy)",STARVED)]:
        print("\n================ %s ================"%label)
        base=agg("S0_baseline",KEYS,subset)
        hdr="%-12s %9s %9s %9s %9s %8s %8s %7s | %7s %7s"%(
            "config","matched","m_min","eig_min","feats","starv%","crit%","cond","ms_p95*","ms_max*")
        print(hdr)
        for cfg in ORDER:
            if cfg not in data or not data[cfg]: continue
            a=agg(cfg,KEYS,subset)
            def d(k):  # pct change vs baseline
                if cfg=="S0_baseline" or np.isnan(base[k]) or base[k]==0: return ""
                return "(%+.0f%%)"%(100*(a[k]-base[k])/abs(base[k]))
            print("%-12s %9.0f %9.0f %9.0f %9.0f %8.1f %8.1f %7.3f | %7.1f %7.1f"%(
                cfg,a["matched_mean"],a["matched_min"],a["eig_mean"],a["feats_mean"],
                a["starv_matched"],a["crit_matched"],a["cond_med"],a["ms_p95"],a["ms_max"]))
        print("  (* offline compute time — NOT real-time representative; directional only)")

    print("\n=== deltas vs S0 (STARVED bags — the confined-space objective) ===")
    base=agg("S0_baseline",KEYS,STARVED)
    for cfg in ORDER[1:]:
        if cfg not in data or not data[cfg]: continue
        a=agg(cfg,KEYS,STARVED)
        print("  %-4s matched %+5.0f%%  eig_min %+6.0f%%  crit-exposure %+5.1fpt  feats %+5.0f%%"%(
            cfg, 100*(a["matched_mean"]-base["matched_mean"])/base["matched_mean"],
            100*(a["eig_mean"]-base["eig_mean"])/base["eig_mean"],
            a["crit_matched"]-base["crit_matched"],
            100*(a["feats_mean"]-base["feats_mean"])/base["feats_mean"]))

if __name__=="__main__":
    main()
