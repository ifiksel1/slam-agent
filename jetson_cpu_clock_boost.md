---
name: jetson-cpu-clock-boost
description: "Boosting Orin NX CPU clock 1497->1984MHz costs only ~3C and buys the compute margin a CPU-bound SLAM node needs for accurate, glitch-free odometry"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 957e3751-78b0-4a42-987f-34e5eb506fb3
---

On the Jetson Orin NX 8GB (6-core), boosting the CPU max clock from the stock 25W cap (1497600 kHz) to the silicon max (1984000 kHz) is a **viable, low-cost way to get more accurate SLAM results** when a node is CPU-bound. Confirmed by the user 2026-06-10 as the preferred approach: "boosting the clock is a viable option to get more accurate results."

**Measured (SuperOdom live, OS1-64 @1024x20, same hand-carried walk loop, A/B):**
| | stock 1497MHz | boost 1984MHz |
|---|---|---|
| CPU temp (under load) | ~69C | ~72C (only **+3C**) |
| feature_extraction load | 100–107% (**core saturated**) | 80% (~20% headroom) |
| /ouster/points under motion | 19.9–20.2Hz (post-run dipped to 19.25) | 20.0Hz steady |
| loop closure (20–30m walk) | 0.17 m | **0.07 m** |
| max single-scan pose step | **1.10 m glitch** (~27 m/s, non-physical) | 0.08 m (clean) |
| vertical (z) drift over loop | **−0.78 m, never recovered** | −0.03 m (excellent) |

**z-drift is a third symptom of the same starvation, NOT a SuperOdom z weakness.** z is normally the BEST-constrained axis (floor/ceiling planes) and the boosted run proved it (0.03 m drift). At stock clock z drifted ~1 m GRADUALLY (biggest single step only −0.32 m, so under-optimization not a glitch) and never self-corrected — same cause as the 1.10 m pose step: a saturated feature_extraction core can't converge the per-scan optimization, so even the well-constrained vertical axis drifts.

**Why:** the boost (+33% clock) costs only ~3C because the load is ~2.3 of 6 cores; the Orin throttles ~95–100C so there is ~25C of headroom either way. Thermal is NOT the constraint — **compute margin is.** At stock clock feature_extraction pegs a full core and a motion spike tips it into a processing hiccup (the 1.10 m step); the boosted run kept ~20% headroom and stayed glitch-free.

**How to apply:**
- Prefer the **boost for flight/accuracy** when a SLAM node is CPU-bound at stock clock. If you must stay at stock 25W for the power budget, instead **drop the LiDAR mode (e.g. 1024x20 -> 512x20)** to halve per-scan compute and restore margin. See [[lidar_mode_tuning]].
- Set it: `sudo ~/superodom_ws/set_cpu_clock.sh boost|stock` (writes `scaling_max_freq` on all 6 cores; **non-persistent**, reverts on reboot). For persistence add a boot unit writing scaling_max_freq.
- **`nvpmodel -m 0` (MAXN) is BROKEN on this 6-core 8GB module** (stock conf references CORE_6/7 -> `NVPM ERROR cpu6/online`); use the sysfs override, not MAXN.
- Caveat: the per-run pose glitches are partly **nondeterministic** (see the SuperOdom replay study, ~12% severe-divergence rate in [[new-slam-frameworks-2026-06]]), so one A/B loop each can't fully isolate clock from chance — but the load/saturation difference is real and repeatable.

Related: [[new-slam-frameworks-2026-06]] (SuperOdom live validation + power note), [[lidar_mode_tuning]].
