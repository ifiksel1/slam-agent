---
name: genz-icp-evaluation
description: GenZ-ICP (RA-L 2025) bag-validated on 3rd_floor — LiDAR-only matched BIEVR-LIO's 1.3cm return-to-origin; hangar sweep still pending
metadata:
  type: project
---

**GenZ-LIO source is NOT public** (arXiv 2603.16273, March 2026, "available upon publication").
Verified 2026-08-31: `cocel-postech/genz-lio` 404s; the org has only `genz-icp` and `pit-nbv`.
Only the LiDAR-only predecessor **GenZ-ICP** (RA-L 2025, arXiv 2411.06766) is installable.
Watch for the GenZ-LIO release — it is the IMU-fused version and the actual thing wanted.

**Mechanism.** Classifies each correspondence planar vs non-planar via local-covariance
eigenvalues (`lambda3/(l1+l2+l3) < planarity_threshold`, default 0.2, `VoxelHashMap.cpp:120`;
<5 neighbors => auto non-planar, `VoxelHashMap.cpp:58`). Blends point-to-plane and
point-to-point costs by `alpha = N_planar/(N_planar+N_non_planar)`, recomputed EVERY ICP
iteration (`Registration.cpp:202`). The real contribution is **conditioning, not observability**:
point-to-point's translational Jacobian block is the IDENTITY (`Registration.cpp:103`), so any
admixture bounds the Hessian's smallest eigenvalue away from zero. It PREVENTS ill-posed
blow-ups; it cannot add information along a genuinely unconstrained axis.

**3rd_floor result (2026-08-31), stock upstream `indoor.yaml`, replay rate 0.5, 3 runs:**
final displacement **0.0136 / 0.0027 / 0.0051 m** — all sub-1.4cm, 0/3 runaway.
- vs BIEVR-LIO 0.013 m (LiDAR-INERTIAL) — **matched, with NO IMU**
- vs COIN-LIO 3.77-4.38 m — beat by ~280x
- vs EllipseLIO on this same bag: 7/14 tight, 6/14 blew up >5m — GenZ shows no such coin-flip
Path length 374.8-376.7 m vs BIEVR 374.0 m; peak 57.1-57.9 m vs 57.8 m.
Caveat: 3rd_floor is EASY (see [[bievr-lio-evaluation]]); this does NOT predict hangar behavior.

**alpha as free degeneracy telemetry.** Hangar (60s probe): mean 0.928, median 0.968.
3rd_floor: mean 0.863-0.888, median ~0.89, p05 as low as 0.63. The hangar reads measurably
MORE planar and swings LESS — the signature of persistent semi-degeneracy vs structured space.
Candidate input for the degeneracy-gating module in [[hangar-737-inspection]].

**Not real-time on Orin NX**: ~11.8 Hz sustained vs a 20 Hz stream (59% of scans) at replay
rate 1.0; ~15.5 Hz (77%) at rate 0.5. Cost is the ICP itself (`max_num_iterations: 100`
converging slowly), NOT the debug-cloud publishing (682 vs 709 poses with it off). Replay
offline sweeps at rate 0.5. Accuracy held despite dropping 23% of scans.

**12-BAG HANGAR SWEEP (2026-08-31), rate 1.0 = REAL-TIME condition, `hangar.yaml`:**
**12/12 bounded, ZERO divergences.** Final displacement 0.24-35.4 m, median 11.3 m.
Head-to-head vs BIEVR-LIO return-to-origin (`~/bievr_ws/results/rto_report.json`):
**GenZ-ICP wins 10/12 with NO IMU.** Both BIEVR km-divergences eliminated:
863,700 m -> 8.84 m (737_700_TAIL) and 362,934 m -> 29.19 m (cmem3qaqb 00:38).
Also 648.8->11.7, 298.2->2.07, 55.2->35.4, 43.2->4.72, 34.9->15.5, 18.6->1.52, 11.1->0.24.
BIEVR wins only 2, both narrow and both on bags it did not diverge on:
737_700_FRONT 5.87 vs 15.35, MAX_8_LIO_SAM_3 10.47 vs 11.31.

**INTERPRETATION — both halves of the prediction held.** The point-to-point identity
translational Jacobian block bounds the Hessian conditioning, so ill-posed BLOW-UPS are
gone. But drift is still METRES (median 11.3 m) on flights that provably returned to ~0.2 m,
vs 1.36 cm from the same binary on 3rd_floor. GenZ-ICP makes hangar failure BOUNDED AND
HONEST, not ACCURATE. **Pure LiDAR odometry remains non-viable standalone in the hangar;
the absolute-anchor architecture in [[hangar-737-inspection]] is unchanged and validated.**
Diagnostic value achieved: the two previously-confounded failure modes are now separated —
estimator fragility (real, fixable, GenZ fixes it) vs true unobservability (real, residual).

**alpha is a WEAK degeneracy predictor — do not over-trust it.** At n=12, correlation with
final drift: alpha_p05 r=-0.46 (right sign, moderate); alpha_median r=+0.10 (useless);
max_step r=+0.47; path_len r=+0.38. An n=6 read suggested p05 was a clean predictor; it is
NOT (TAIL_RIGHT_SCAN has healthy p05 0.815 but the worst drift, 35.4 m). If gating on alpha,
use a LOW PERCENTILE not the median, and combine with max_step — neither alone suffices.

**BIEVR's own GT tiering predicts GenZ difficulty.** The one `CLOSED` bag (icp_inlier_frac
0.94) is GenZ's best result (0.30 m). On the 4 `open` bags `true_drift_m` is null — the ICP
ground truth could not be established, so GenZ drift there is measured against an UNVERIFIED
reference and should not be quoted as confirmed error.

**LiDAR-ONLY, `deskew: false`** (upstream advises off for aggressive motion — their deskew is a
constant-velocity model). Not a flight candidate. Harness: `slam-agent/genz_icp_integration/`,
image `genz_icp:noetic` (native arm64, builds in 69s; no PCL/OpenCV/Ceres so none of the known
build traps apply). **12-bag hangar sweep still PENDING** — that is the test that matters.
