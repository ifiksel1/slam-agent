---
name: genz-icp-algorithm-reference
description: How GenZ-ICP's adaptive planar/non-planar weighting actually works — math, code line refs, paper benchmarks, and why it fixes conditioning but not observability
metadata:
  type: reference
---

Algorithm/paper reference for **GenZ-ICP** — *"Generalizable and Degeneracy-Robust LiDAR
Odometry Using an Adaptive Weighting"*, RA-L 10(1):152-159, 2025. arXiv **2411.06766**.
Repo `github.com/cocel-postech/genz-icp` (MIT, a KISS-ICP fork). Empirical results on our own
data: [[genz-icp-evaluation]]. Successor **GenZ-LIO** (IMU-fused): arXiv **2603.16273**,
Mar 2026, **code unreleased** ("upon publication") — watch for it.

## The two failure modes it straddles

ICP solves `H dx = -b`; H is a table the solution rests on and its eigenvalues are leg lengths.
**Degeneracy = a direction where no correspondence has an opinion** -> tiny eigenvalue ->
dividing by ~0 amplifies noise into huge pose jumps.

- **Point-to-plane** residual `(p-q).n` counts only the out-of-plane error. Metaphor: each
  constraint is a sheet of frictionless ice — it resists motion THROUGH the surface, and lets
  you glide ALONG it free. In a corridor, floor/ceiling normals point up/down and wall normals
  sideways; **no normal points down the corridor**, so along-axis translation is unconstrained
  and the estimate slides. Worse: the far sparse points that COULD constrain that axis have
  noisy normals and get rejected by the robust kernel — the pipeline discards exactly the
  constraint it needs.
- **Point-to-point** residual `p-q` pins to a specific map point in all 3 axes. Metaphor: a
  rubber band to a pin, resisting equally in every direction. On a smooth wall the nearest map
  point is the WRONG spot, so it drags tangentially — inaccurate in structured scenes. But it
  is omnidirectional, and "wrong anchor nearby" beats "no anchor at all".

## The mechanism

1. **Classification** (`VoxelHashMap.cpp:120`): gather map points from the 27 surrounding
   voxels, build a 3x3 covariance, eigendecompose. `is_planar <=> lambda3/(l1+l2+l3) <
   planarity_threshold` (default **0.2**; KITTI 0.18). That ratio is *surface variation* — the
   fraction of patch variance sticking OUT of its best-fit plane (pancake ~0, isotropic blob
   1/3). At 0.2 the test is permissive: gently curved surfaces (a fuselage) pass as planar.
   **<5 neighbors => auto non-planar** (`VoxelHashMap.cpp:58`, "not intended to be changed").
2. **Blend** (`Registration.cpp:202`): `alpha = planar_count/(planar_count+non_planar_count)`,
   a literal census recomputed EVERY ICP iteration.
3. **Cost**: `min  alpha * SUM_j [(R p_j + t - q_j).n_j]^2  +  (1-alpha) * SUM_k ||R p_k + t - q_k||^2`
4. **Jacobians** (`Registration.cpp:94-104`) — the load-bearing detail:
   - planar: `J = [n^T, (p x n)^T]` — 1 row, rank-1, votes only along its own normal.
   - point-to-point: `J = [I3, -[p]x]` — 3 rows, **translational block = IDENTITY**,
     isotropic, condition number exactly 1 regardless of where the point is.
   => mixing in any `(1-alpha)` **bounds H's smallest translational eigenvalue away from zero**.
   Robust weight `kernel^2/(kernel+r^2)^2` with `kernel = sigma/3`, gate `3 sigma`
   (`GenZICP.cpp:82-83`). Solve `dx = H.ldlt().solve(-b)` (`Registration.cpp:204`), retract
   `SE3::exp(dx)`. Initial guess = constant velocity. Adaptive voxelization from LOCUS 2.0
   (`GenZICP.cpp:64`) rescales the filter to hold `desired_num_voxelized_points`.

## ⭐ What it actually buys: conditioning, NOT observability

The paper's claim is that it "prevents mathematically ill-posed problems" — strictly weaker
than "recovers the true motion", and it says the blend "may not represent the global minimum".
A stubby leg stops the table collapsing; it does not tell you the floor's true height.
**No reweighting scheme adds information along an axis that carries none.** This is exactly
what we observed: zero divergences at any config, but accuracy came from FIXING THE SOLVER
DEFAULTS, not from the weighting.
Second, quieter benefit: points that pure point-to-plane THROWS AWAY (too few neighbors, too
curved) get reclassified as point-to-point rather than discarded.

## Paper's own benchmark results

- **General scenes — only on par.** KITTI 00-10 avg 0.51% (KISS-ICP 0.50% wins); MulRan best on
  KAIST/Sejong, 2nd on DCC/Riverside; Newer College short 0.46% best, long 0.94% (CT-ICP wins).
- **Degenerate scenes — decisive.** SubT-MRS Long_Corridor APE RMSE **1.99 m** vs KISS-ICP 8.72,
  DLO 9.09, pure point-to-plane 33.2, CT-ICP 45.7 — while X-ICP and Zhang et al. (dedicated
  degeneracy methods) DIVERGED. HILTI-Oxford Exp07 score 33.33 vs Zhang 23.33, DLO 13.33,
  KISS-ICP and X-ICP 0.00.
- ⚠️ **1.99 m is the headline win: bounded, not accurate.** And **every baseline is LiDAR-only**
  (KISS-ICP, CT-ICP, DLO, MAD-ICP, F-LOAM, MULLS, SuMa, X-ICP, Zhang). **FAST-LIO2/Point-LIO are
  NOT in the comparison** — there is no published GenZ-vs-LIO number. Don't let "SOTA" imply one.
- Paper reports **NO runtime numbers and no CPU spec**. Our measurement: ~6-8 Hz at stock
  defaults on an Orin NX, 20 Hz after retune — see [[genz-icp-evaluation]].
- Paper's own conclusion: *"Future work will be devoted to applying GenZ-ICP to a LiDAR-inertial
  odometry framework to enhance the robustness against aggressive motion."* That is GenZ-LIO.

## Gotchas inherent to the design

`deskew: false` in every shipped config — their deskew is a constant-velocity model and the
tuning guide explicitly says keep it OFF for aggressive/hand-held motion. No IMU anywhere, so
the initial guess is constant-velocity: **dropped scans hurt disproportionately** (extrapolating
across gaps), which is why scan-capture % must always be reported. Both hot paths use
`tbb::parallel_reduce` (`Registration.cpp:135`, `VoxelHashMap.cpp:197`) — non-deterministic FP
summation order; swap `parallel_deterministic_reduce` if bit-reproducibility is ever needed.
