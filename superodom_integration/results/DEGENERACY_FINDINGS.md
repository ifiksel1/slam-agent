# SuperOdom Degeneracy Characterization — Orin NX 8GB / OS1-64 (2026-06-09)

> **🔁 5× REPEATABILITY + STATS-USABILITY SWEEP 2026-06-09 (5 identical replays of `degen_bag`,
> 1.0× CycloneDDS) — this is the strongest result; it REVISES the "gate on uncertainty"
> recommendation below.** Harness `scripts/repeatability.sh` (per-scan CSVs in
> `~/superodom_ws/results/repeat/run_*.csv`), analysis `scripts/analyze_degeneracy.py`.
>
> **#1 Endpoint repeatability is excellent but MISLEADING:**
> - net return-to-origin: 0.85 / 0.91 / 0.87 / 0.87 / 0.87 m → **mean 0.87 m, sd 0.02 m**.
>   (net = drift + the operator's real end-offset from the true origin → an UPPER BOUND on drift.)
> - end-yaw: −19.1…−21.5° (~±1° band). Highly deterministic by endpoint.
> - **BUT 1 of 5 runs (run 4) had a crash-grade transient:** the pose **oscillated ±13 m between
>   consecutive scans for ~0.7 s around t=112 s, then recovered** to a normal endpoint
>   (net 0.87 m!). path 251 m vs ~63 m baseline, peak 13.2 m vs ~7.2 m. Same input, nondeterministic.
>   **Endpoint error completely hides it** — never validate this stack on return-to-origin alone.
>
> **#3 NO `/super_odometry_stats` field is a usable EKF-gating signal in this build** (all 5 runs):
> - `/state_estimation_health`: stuck **`true`** (0 false scans) even through run 4's explosion.
> - `uncertainty_x/y/yaw`: **saturated ~1.0 from startup** in good AND bad runs → zero discrimination.
> - `plane_match_success`: **0 every scan** (not populated). `latency`: large **negative** (clock
>   mismatch). `n_iter`: pinned at cap (5). `translation_from_last`: **0.03 m while pose jumped 10 m.**
> - **The transient that would crash a drone is invisible in every self-reported metric.**
>
> **→ REVISED GATING RECOMMENDATION:** do NOT gate on health OR on `uncertainty_*` (saturated here).
> Gate **externally** on a kinematic plausibility check of `/state_estimation` itself: run 4's jumps
> were ~10 m in ~0.02 s ≈ **500 m/s** implied velocity — reject any pose implying >5–10 m/s. The
> EKF must sanity-check the pose STREAM, not trust any SuperOdom confidence field.

> **✅ RE-RUN 2026-06-09 (corrected extrinsic + CycloneDDS, 1.0× native) — supersedes the
> drift/yaw numbers below.** Same `degen_bag`, fixed extrinsic `[-1,0,0,0,-1,0,0,0,1]` +
> `use_imu_roll_pitch:true`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, replayed at **1.0×** (old run
> needed 0.5× — Cyclone removed the Fast-DDS bottleneck; see README "Live rate"):
> - **path 62.05 m, peak excursion 7.07 m, return-to-start 0.84 m (~1.4%)**, 4355 samples / 174 s.
> - **Yaw blowup ELIMINATED:** end-yaw drift **−20.5°** vs the old identity-extrinsic run's
>   **±130–137° jumps**. The extrinsic fix cured the catastrophic heading failure.
> - Degeneracy signature unchanged: `uncertainty_x/y/yaw` mean ~1.0; `z` best (mean 0.52);
>   `icp_avg_distance` 0.03→0.81. **`/state_estimation_health` stayed `true` 100% of the run
>   (0.0 s false)** — the health-flag-insensitivity finding STANDS.
> - Tracked at native 1.0×, confirming real-time capability on this rig under CycloneDDS.

> **CAVEAT (original run below):** used a placeholder *identity* LiDAR↔IMU extrinsic. That was
> wrong for this rig and caused the yaw instability (±130° jumps) and "yaw sign inverted" below —
> now superseded by the RE-RUN above.

Featureless-corridor test. The data was captured to a bag with the **driver alone**
(clean 20 Hz) and **replayed offline into SuperOdom** (original run below at 0.5×; the RE-RUN above
at 1.0× under CycloneDDS).

- Bag: `~/superodom_ws/degen_bag` (10 GB, NOT in git) — 176 s, 3386 clouds @19 Hz, 17591 IMU @100 Hz.
- Trajectory log: `degen_corridor_replay_2026-06-09.log`.

## Result — walk: ~54.67 m path, out to ~7 m, returned to ~start
| Signal | Value | Interpretation |
|---|---|---|
| uncertainty_yaw | min 0.88, **mean 1.00** | Heading = most degenerate DOF; pinned at max almost always |
| uncertainty_x | mean 0.99, **maxed in ~94% of samples** | Along-corridor translation starved |
| uncertainty_y | mean 0.99, min 0.60 | Cross-corridor also starved, brief relief |
| uncertainty_z | mean 0.52, min 0.14 | Best-constrained (floor/ceiling planes) |
| ICP avg_distance | 0.03 (good) → **0.81** (worst) | Spikes mark worst geometry; usable degeneracy proxy |
| Drift (loop) | **0.79 m over 54.67 m ≈ 1.4%** | Held together despite pervasive degeneracy |
| yaw stability | **jumps to +132°/+137°** at far end (~6.4 m out) | Heading genuinely came apart in deepest/most-degenerate stretch |
| `/state_estimation_health` | **`true` the ENTIRE run** | Did NOT respond to the degeneracy |

## ⚠️ Flight-integration takeaways
1. **Gate EKF fusion on `/super_odometry_stats` per-axis uncertainty, NOT `/state_estimation_health`.**
   The boolean health flag stayed green through a degenerate corridor where x/yaw uncertainty
   was pinned at max — trusting it would feed bad heading to ArduPilot (flyaway risk).
2. **Yaw is the weak DOF** (as `uncertainty_yaw=1.0` predicted) — heading jumped ±130° at the
   far end. Heavily distrust yaw in confined/symmetric spaces; consider a magnetometer/other
   yaw source there. Compounds with the known yaw-sign-vs-ENU issue.
3. **z is the most reliable axis** in corridors (planes above/below).
4. Cold start needs motion — SuperOdom won't initialize while stationary (first ~20 s of the
   walk produced no pose until movement began).

## Live limitation (why offline replay)
On this 8 GB / 6-core Orin, the ROS2 Ouster driver (1024×20) + SuperOdom together drops the
LiDAR to ~11 Hz irregular → feature_extraction throws every scan ("lidar more recent than imu")
→ no tracking. Driver alone holds 20 Hz. **For LIVE flight on this rig SuperOdom at 1024×20 is
not viable**; options: run 512×20 live (lighter, needs validation), reduce SuperOdom CPU, move to
a beefier compute, or use the lighter **EllipseLIO** for live use and keep SuperOdom for
offline/analysis.
