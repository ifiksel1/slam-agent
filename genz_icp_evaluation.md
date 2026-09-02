---
name: genz-icp-evaluation
description: GenZ-ICP (RA-L 2025) — LiDAR-only, real-time on Orin NX, sub-metre true drift on 6/6 usable hangar bags after a critical retune; beats BIEVR-LIO
metadata:
  type: project
---

**GenZ-LIO source is NOT public** (arXiv 2603.16273, Mar 2026, "available upon publication").
Verified 2026-08-31: `cocel-postech/genz-lio` 404s; the org has only `genz-icp` + `pit-nbv`.
Only the LiDAR-ONLY predecessor **GenZ-ICP** (RA-L 2025, arXiv 2411.06766) is installable.
Watch for the GenZ-LIO release — it is the IMU-fused version and the actual thing wanted.

**Mechanism.** Classifies each correspondence planar vs non-planar from local-covariance
eigenvalues (`lambda3/(l1+l2+l3) < planarity_threshold`, default 0.2, `VoxelHashMap.cpp:120`;
<5 neighbors => auto non-planar, `VoxelHashMap.cpp:58`). Blends point-to-plane and
point-to-point costs by `alpha = N_planar/(N_planar+N_non_planar)`, recomputed every ICP
iteration (`Registration.cpp:202`). The real contribution is **conditioning**: point-to-point's
translational Jacobian block is the IDENTITY (`Registration.cpp:103`), so any admixture bounds
the Hessian's smallest eigenvalue away from zero and forbids ill-posed blow-ups.

## ⭐ THE CRITICAL FINDING: upstream defaults are BROKEN for this use case

On 737_700_FRONT, changing TWO parameters took it from **15.35 m -> 0.21 m** AND from
6.04 Hz -> 20.00 Hz. Accuracy and speed improved TOGETHER; this is not a tradeoff.

| config | throughput | scan capture | final displ |
|---|---|---|---|
| stock (iters 100, voxel 0.3) | 6.04 Hz | 30% | 15.35 m |
| iters 30 | 8.86 Hz | 44% | 0.20 m |
| iters 5 | 17.36 Hz | 87% | 0.23 m |
| **iters 5 + voxel 0.5 (`hangar_rt.yaml`)** | **20.00 Hz** | **9726/9727 = 100%** | **0.21 m** |

- `max_num_iterations: 100 -> 5` is an **ACCURACY fix, not a speed knob**. In a degenerate
  scene the solve is near-singular, so extra Gauss-Newton steps keep walking along the
  ill-conditioned direction instead of converging. Capping = early-stopping regularization.
  Accuracy is FLAT (0.20-0.24 m) across iters 30/15/10/5; ONLY the stock 100 is broken.
- `voxel_size: 0.3 -> 0.5` is the dominant SPEED lever and is FREE: +81% throughput, identical
  accuracy, alpha median unchanged (0.962 -> 0.954). Bigger voxels = fewer but better-populated
  neighborhoods = cheaper AND better-conditioned normals.
- Upstream defaults are tuned for offline benchmark accuracy, NOT embedded real-time.
  **Always sweep max_num_iterations + voxel_size before judging a KISS-ICP-family estimator.**

## 12-bag hangar sweep @ `hangar_rt.yaml`, rate 1.0 (2026-08-31)

**All 12 bags ran at 20.0 Hz with 100% scan capture — genuine real-time on the Orin NX,
LiDAR-ONLY, no IMU.** Scored through the project's own validated tooling
(`scripts/csv_to_tum.py` -> `scripts/rto_report.py` vs `~/bievr_ws/results/icp_gt_v2/`), so
numbers are directly comparable to BIEVR-LIO. true_drift = `||(p_end - p_start) - t_icp||`
(placement removed) — NOT displacement-vs-zero.

| bag | BIEVR true_drift | **GenZ-RT** | GT quality |
|---|---|---|---|
| 737_700_FRONT | 5.908 | **0.124** | near (rmse fails by 3mm) |
| MAX_8_LIO_SAM_3 | 10.639 | **0.342** | **closed_loop TRUE** |
| MAX_8_TAIL 00:45 | 298.136 | **0.096** | near |
| MAX_8_TAIL 01:33 | 11.110 | **0.350** | near |
| cmem3qaqb 02:03 | **0.436** | 0.572 | **closed_loop TRUE** |
| cmem9gs5j 03:18 | 18.703 | **0.912** | **closed_loop TRUE** |

**GenZ: ALL SIX sub-metre (0.096-0.912 m), median 0.35 m.** That absolute result stands.

⚠️ **THE HEAD-TO-HEAD IS RETRACTED (2026-08-31).** The BIEVR column above is STOCK config; the
GenZ column is TUNED. Phase-7 tuning of BIEVR (`map/pixel_size_m: 0.05 -> 0.40`, 8x stock)
improves it up to **32x** -- 10.639 -> 0.334 m on MAX_8_LIO_SAM_3, where GenZ gets 0.342 m.
Across the 4 GT bags, tuned BIEVR = 0.334 / 0.434 / 4.018 / 0.452 (3/4 sub-metre, median
0.443) vs tuned GenZ 0.342 / 0.572 / 0.912 / 0.124 (4/4 sub-metre, median 0.457).
**Medians are effectively identical -- neither estimator is meaningfully more accurate.**
Never compare a tuned estimator against another's stock defaults. See [[bievr-lio-evaluation]].

## ⚠️ GROUND-TRUTH CAVEATS — read before quoting any of the above

- **Only 3/12 bags have `closed_loop: True`.** `icp_return_gt.py` sets it as
  `inlier >= min_inlier AND rmse <= ~0.20`. `rto_report.py` DELIBERATELY overrides this,
  recomputing tiers from inlier+offset and ignoring rmse ("single-scan rmse runs high
  regardless"). Defensible — FRONT misses by 3mm at inlier 0.79 — but 3 of the 6 scored bags
  carry that caveat. Applies EQUALLY to BIEVR, so the head-to-head stays fair.
- **5 bags are open-loop and UNSCORABLE** (inlier 0.15-0.30). They ran clean at 20 Hz but
  their accuracy is unknown. NEVER quote their final displacement as drift.
- **MAX_8_LIO_SAM_WING 02:29 must be EXCLUDED — its ICP GT is bogus.** GenZ's start->end
  vector is 161.6 deg from the ICP "truth" (`||d-t||`=7.54 vs `||d+t||`=1.22), but this is NOT
  a sign-convention bug (FRONT 25.6 deg, cmem9gs5j 41.4 deg align fine). The bag is flagged
  POOR OVERLAP / did-not-return: ICP registered the first scan against the last scan of an
  open path and found a spurious 3.816 m alignment between two DIFFERENT places. **BIEVR's
  42.53 m on this bag is equally meaningless.** Discard both, do not credit GenZ a win.

## 3rd_floor reference bag (stock `indoor.yaml`, rate 0.5, 3 runs)

final displacement **0.0136 / 0.0027 / 0.0051 m** — all sub-1.4cm, 0/3 runaway.
Matches BIEVR-LIO's 0.013 m **with NO IMU**; beats COIN-LIO (3.77-4.38 m) ~280x; EllipseLIO on
this same bag is a coin-flip (7/14 tight, 6/14 >5m). Path 374.8-376.7 m vs BIEVR 374.0 m.
NOTE: 3rd_floor FAILED to discriminate — BIEVR also scored 1.35cm here then produced 863 km on
hangar data. **Do not use 3rd_floor alone to qualify an estimator.**

## RETRACTED — do not reuse

An earlier 12-bag sweep at rate 1.0 with STOCK config gave median 11.3 m and "12/12 bounded but
not accurate", which was written up as evidence of irreducible hangar unobservability. **That was
an artifact of the broken stock default plus 30-60% scan starvation, and is fully superseded by
the table above.** The alpha correlations computed from it (alpha_p05 r=-0.46, median r=+0.10,
max_step r=+0.47) are likewise confounded — they largely measured starvation, not scene
structure. Lesson: for a non-real-time estimator, replay rate and solver defaults are
first-class experimental variables; ALWAYS report scan-capture %.

## Deskew A/B = NULL, and what it says about adding an IMU (2026-08-31)

Our bags DO carry a per-point `t` field (`x y z intensity t reflectivity ring ambient range`)
and the ROS1 node parses `t`/`timestamp`/`time`, so upstream deskew is usable -- it is a
ONE-LINE config change, no IMU needed. A/B at `hangar_rt.yaml` (single variable, deskew
false vs true), rate 1.0, on the 3 `closed_loop:True` bags + FRONT. true_drift, m:

| bag | deskew OFF | deskew ON | delta |
|---|---|---|---|
| MAX_8_LIO_SAM_3 | 0.3419 | 0.3355 | -0.0064 |
| cmem3qaqb 02:03 | 0.5716 | 0.5679 | -0.0037 |
| cmem9gs5j 03:18 | 0.9123 | 0.9139 | +0.0016 |
| 737_700_FRONT | 0.1239 | 0.1166 | -0.0073 |

3 improve, 1 worsens, **all deltas <= 7mm vs an 8.6mm run-to-run spread -> indistinguishable
from zero.** Throughput unchanged at 20.0 Hz on every bag (deskew is free). **In-scan
distortion is NOT a limiting factor for this flight envelope** -- at 20 Hz a scan spans 50ms
and these flights don't move enough within it to matter at the decimetre scale where the real
error lives.

**CONSEQUENCE — do NOT graft an IMU into GenZ-ICP (Tier 2); use ArduPilot EKF3 (Tier 1).**
The two IMU touchpoints are cleanly isolated in `GenZICP.cpp` and neither is in
`Registration.cpp`: `GetPredictionModel()` (constant velocity `poses_[N-2].inverse()*poses_[N-1]`)
and `DeSkewScan(frame, timestamps, start_pose, finish_pose)`. So a graft WOULD be far safer
than the EllipseLIO photometric one — that failed because it injected a residual into the
IKFoM (the estimator CORE); this would only change INPUTS (initial guess + cloud shape), and
degrades gracefully. **But it is not worth doing:** half the value of an IMU is gyro deskew,
just measured at ~0; the other half is the motion prior, and constant-velocity is already
sufficient (max_step 0.046 m at the RT config, so successive scans are close). The OS1-64's
internal IMU has a factory extrinsic and we have Allan-variance params, so the cost is days
not weeks — the payoff is what's missing.
**Two caveats:** (1) the bags may not contain the aggressive motion that would break the
constant-velocity assumption — this tests the envelope we RECORDED, not one we might fly;
(2) upstream deskew derives scan velocity from the previous two poses, so it is weakest during
ACCELERATION, exactly where an IMU would differ. A null under gentle motion does not rule out
a gain under hard motion. If aggressive flight later breaks the prior, prefer the GenZ-LIO
release over a hand-rolled graft.

## Practical

**LiDAR-ONLY, `deskew: false`** (no IMU; upstream advises off for aggressive motion, their
deskew is a constant-velocity model). Harness `slam-agent/genz_icp_integration/` — committed on branch **`feat/genz-icp-integration`** (428d7a6, forked from main, results/ gitignored; still untracked on other branches). Image
`genz_icp:noetic` (native arm64, builds in 69s; no PCL/OpenCV/Ceres so none of the known build
traps apply). **Determinism at the RT config: STABLE (4x FRONT, 2026-08-31).** true_drift
0.1240 / 0.1269 / 0.1240 / 0.1183 m -- **spread 8.6 mm**; end-vector max pairwise 10.9 mm; all
4 at 20.00 Hz. NOT byte-identical (differing md5s -- `tbb::parallel_reduce` FP order, as
expected), and one run captured 8667/9727 scans instead of 9727 yet still landed 0.1183 m.
So: bounded and repeatable at the cm level, NOT bit-reproducible like BIEVR, but with none of
EllipseLIO's coin-flip runaways. Good enough to trust single-run numbers to ~1cm on this bag.
Gotchas: (1) NEVER `--net=host` — collides with the host rosmaster from the foxglove_bridge
stack, silent no-odometry (solution #12); (2) alpha is not published — recovered from
`/genz/planar_points` + `/genz/non_planar_points` sizes, needs `visualize:=true` (free: 682 vs
709 poses); (3) upstream `odometry.launch` starts rviz under that same arg — use our
`genz_hangar.launch`; (4) both hot paths use `tbb::parallel_reduce` (non-deterministic FP order).
**Live sensor / MAVROS / flight: NOT attempted.**
