# EllipseLIO & SuperOdom — validation findings & gotchas

Distilled from the validation/benchmark work on this rig (Jetson Orin NX 8GB, Ouster OS1-64).
Read [jetson_orin_ouster_operational.md](jetson_orin_ouster_operational.md) first — CycloneDDS
and clock boost are prerequisites for everything below.

Both are **camera-free LiDAR-inertial** (they sidestep the FAST-LIVO2 camera-selection blocker).

| | EllipseLIO (`ellipse_lio`) | SuperOdom (`super_odometry`) |
|---|---|---|
| Lineage | FAST-LIO2 + IKFoM + i-Octree (vendored; no GTSAM/Ceres) | GTSAM/Ceres/Sophus, 6-DOF degeneracy/uncertainty |
| ROS 2 | Humble + Jazzy | Humble only |
| Odom topic | `/ellipselio_odom` (IMU-rate forward-prop, ~79 Hz) | `/state_estimation` (~25 Hz) + `/state_estimation_health` |
| Build | light (image ~3.3 GB) | heavy (source-builds GTSAM/Sophus/Ceres) |
| Status (2026-06-21) | Phase 5 **live-validated** (handheld); `integration_complete:false` until flight-scale loop + bridge | Phase 5 **bench+motion validated**; `integration_complete:false` |

---

## 1. Validation status

- **SuperOdom (2026-06-09 → 06-11):** live driver + SuperOdom co-running under CycloneDDS,
  `/state_estimation` 25 Hz / `/ouster/points` 20 Hz **under motion**, loop closure 0.07 m over
  a 20 m path, valid TF. Real-time-capable at 1024x20 on this rig.
- **EllipseLIO (2026-06-21):** live driver + EllipseLIO, `/ouster/points` 20.02 Hz, `/ellipselio_odom`
  ~79 Hz, single publisher (no dup-estimator leak). Handheld motion test: **loop closure 0.02 m,
  max single-step 0.01 m (smoothest measured on this rig), yaw drift −3.2°.** Caveat: ~1 m handheld
  envelope, not a flight-scale walking loop.
- **Remaining for both before flight:** flight-scale loop + `vision_to_mavros` bridge (negate yaw +
  world-Z/heading correction) + GPS-denied flight.

---

## 2. The 3-way benchmark (apartment bag, boosted clock, CPU-isolated)

Cleanest comparison = each framework run **sequentially with all other LIO nodes killed** (a
co-running node confounds the compute-marginal ones). Frame-invariant metrics only — the three
use different world-frame conventions, so raw x/y/z are **not** comparable; compare peak excursion,
path length, loop closure, max single-step.

| Framework (tuned) | path | peak excursion | loop closure | max step | diverge? |
|---|---|---|---|---|---|
| **EllipseLIO** | 62.73 m (≈ true 63 m) | 9.65 m | **0.01 m** | **0.13 m** | none |
| **FAST-LIO** (`extrinsic_est_en:false`) | 70.60 m | 9.49 m | 0.32 m | 0.27 m | none |
| **SuperOdom** (light config) | 95.95 m (integrated jitter) | 9.58 m | 0.13 m | 0.23 m | none |

**EllipseLIO wins on clean data** (perfect loop closure, smoothest), but is compute-marginal at
stock clock (needs boost). SuperOdom is solid **given the cores to itself**. All three are
challenged by a feature-poor section ~t=84–90 s in this bag — a bag property, not a single-framework flaw.

---

## 3. ⚠️ Non-determinism & flyaways — the estimator's own confidence CANNOT gate them

This is the most important flight-safety finding. Both estimators can diverge run-to-run on the
same input, and the divergence is **invisible in every self-reported metric**.

- **EllipseLIO** on the `3rd_floor` bag (14 clean-slate runs, boosted, rate 1.0): final displacement
  was **bimodal, near coin-flip** — 7/14 tight (~0.02 m), 6/14 blew up >5 m (up to 302 m). With
  fresh-container-per-run the rate dropped to ~25% but did not vanish. Likely cause: multi-threaded
  FP reduction-order (`component_container_mt` + parallel tensor-voting) tipping a knife-edge
  barely-observable spot — **not** zombie contamination (tested & refuted) and **not** scan-drop
  (sample counts didn't track the drift).
- **`obs_score` is a real dynamic signal** (dips 0.26–0.55 in feature-poor zones, unlike SuperOdom's
  stuck health flag) **but does NOT predict the runaway** — a runaway run had *higher* endpoint
  obs_score than a clean one; num_feats / rng_mean / num_reject all indistinguishable.
- **SuperOdom:** ~1/5 runs hit a crash-grade ±13 m oscillation transient that recovered to a normal
  endpoint (so **return-to-origin error HID it** — always track peak excursion + path length too).
  `/state_estimation_health` stayed `true` through it; `uncertainty_x/y/yaw` saturated ~1.0 from
  startup in good **and** bad runs (zero discrimination). The SuperOdom transient is
  **CPU-contention-driven** — given the cores to itself it was stable (the kinematic gate is still
  a required safety net).

**Gating guidance (authoritative):**
1. **Never trust the estimator's health flag OR uncertainty.** Both are useless for flyaway detection here.
2. **Gate externally on kinematic plausibility of the pose stream** — reject any pose implying
   >5–10 m/s (SuperOdom's jumps were ~500 m/s).
3. **But a kinematic gate is NOT sufficient:** EllipseLIO's runaways are *smooth/gradual* (max-step
   0.10 m, ~4–9 m accumulated over 20 s) and slip under any velocity gate. Only an **absolute
   reference (fiducial / prior-map relocalization)** catches the position discrepancy. → direct
   evidence for the Hangar-737 absolute-anchor architecture: degeneracy/confidence gating alone is insufficient.
4. **Treat any single-run SLAM number as one draw from a wide distribution** — report N-run
   distributions with clean slates (`docker restart` + verified-empty `ros2 node list` per run), never single shots.

---

## 4. Config facts you must get right

### Both frameworks — this rig's Ouster OS1-64 LiDAR↔IMU extrinsic
The OS1 IMU is yaw-flipped vs the LiDAR. Use:
`extrinsic_R = [-1,0,0, 0,-1,0, 0,0,1]`, `T = [-0.006253, 0.011775, 0.028535]`.
A placeholder **identity** extrinsic causes yaw instability (±130° jumps). Applies to EllipseLIO's
`r_imu_lidar`, SuperOdom's `extrinsicRotation_imu_laser`, and FAST-LIO's `extrinsic_T`.

### ⚠️⚠️ EllipseLIO `lidar.rate` MUST match the real scan rate
Set `rate: 20` for this rig's 1024x20 — **not** the dataset default of 10. EllipseLIO uses `rate`
for IMU↔LiDAR association (`imu_processing.cpp` `match_idx=floor(time_diff*rate)`) and deskew period
(`scan_time=1/rate`); a 2× error mis-associates IMU to scans → **catastrophic 8000 m runaway under
motion.** When porting any EllipseLIO dataset config, always set `lidar.rate` to the live sensor Hz.

### EllipseLIO other tuning
- `lidar.vertical_fov` is **tune-critical** (OS1-64 = 42.4); wrong value silently degrades accuracy.
- `map_resolution: 0.2` (not the 0.1 default — 0.1 fragments sparse features → tensor-voting collapse
  → terminal runaway; 0.2 fixed it, 2/2 clean at boost). Floor 0.1 m; raise to 0.15–0.2 if RAM-bound.
  EllipseLIO has **no online-extrinsic flag** (weights self-adapt) so FAST-LIO's `extrinsic_est_en`
  fix is unavailable here.

### FAST-LIO: set the real `extrinsic_T`, and `extrinsic_est_en: false`
`extrinsic_T: [0,0,0]` looks fine for most of a run then tips into an end-of-run IMU runaway
(14→92 m while standing still). Set the real lever-arm (above). And **`extrinsic_est_en: true` is
HARMFUL** with a correct extrinsic — the free DOF drifts during degeneracy (diverged to 9044 m);
`false` gave loop 0.96–1.27 m, no divergence, 2/2.

### QoS — EllipseLIO publishes `/ellipselio_odom` as BEST_EFFORT
A RELIABLE subscriber gets **zero** messages. Samplers/bridges must use BEST_EFFORT QoS.
(`ros2 topic hz` masks this by auto-negotiating, so it can look fine while a real subscriber sees nothing.)

### ⚠️ Yaw-sign inversion (BOTH frameworks)
A physical **left/CCW** turn reads as a **yaw DECREASE** (≈ −90° for a 90° turn). Magnitude is
accurate — it's purely a sign/frame convention. **`vision_to_mavros` MUST negate yaw before MAVROS.**
Yaw is also the weakest/least-observable DOF (≈ 15° drift over a short loop on SuperOdom) — the
flight-limiting axis; position tracks well.

---

## 5. Foxglove frame orientation (display only — metrics stay frame-invariant)

EllipseLIO renders Z-down + X-backward (and SuperOdom builds the world in sensor-start orientation,
so it renders upside-down). Proper fix for EllipseLIO (rig branch `fix/frame-names-no-leading-slash`):
drop the non-canonical leading slash on frame ids in `map_processing.cpp`, then publish a **`/tf_static`**
`map_view → odom_ellipselio` **180° pitch** (flips X and Z, keeps Y → upright **and** X-forward) and set
Foxglove Fixed Frame = `map_view`. Two gotchas: use `/tf_static` (timeless — applies at any message
stamp; a continuous `/tf` stamped with the node clock breaks the chain vs sensor-time TF), and Foxglove
**caches `/tf_static`** so you must reconnect the client after changing the rotation. For SuperOdom use
a 180° roll `map_view` static TF. EllipseLIO saliency viz: `/visualization_marker` (plane=blue/line=green/
ball=yellow ellipsoids) + `/cloud_effected` (features used in the EKF update).

---

## 6. Harness pointers (in this repo)
- EllipseLIO: `ellipselio_integration/scripts/run_ellipselio.sh` (live), `run_bag.sh` (offline metrics),
  `run_bag_foxglove.sh` (watch + analyze, logs `/analytics`), `loop_until_runaway.sh`,
  `ellipselio_diag.py` (obs_score/feats vs disp/step), `ellipselio_traj_sampler.py`.
- SuperOdom: `superodom_integration/scripts/run_superodom_bench.sh`, `process_field_bag.sh`,
  `benchmark_bag.sh`, `repeatability.sh`, `check_bag_gaps.py`, `set_cpu_clock.sh`;
  `results/DEGENERACY_FINDINGS.md`.
- **Operational order baked into the harness:** run frameworks sequentially, kill all other LIO
  nodes first, clean DDS slate per run, boosted clock.
