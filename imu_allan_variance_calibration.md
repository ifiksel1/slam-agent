---
name: imu_allan_variance-calibration
description: "Allan-variance IMU calibration of the Ouster OS1-64 built-in IMU (2026-07-13) and how it feeds EllipseLIO; COIN-LIO/BIEVR don't consume it"
metadata: 
  node_type: memory
  type: project
  originSessionId: a9391473-c603-4fc3-826c-8f3c4da3e988
---

Allan-variance calibration of the rig's Ouster OS1-64 **built-in IMU** (2026-07-13).

**Which frameworks even use IMU noise params** (checked live configs):
- **EllipseLIO** — YES, 4 explicit fields `acc_noise/gyr_noise/acc_bias/gyr_bias` (Allan-shaped). Only framework where a calibration changes a number the estimator reads.
- **COIN-LIO** — NO. Uses FAST-LIO stock covariances `acc_cov/gyr_cov=0.1, b_*_cov=1e-4` (round defaults, IKFoM insensitive). Consistent with COIN-LIO being rock-stable on 3rd_floor *without* IMU tuning.
- **BIEVR-LIO** — NO. Exposes no noise densities; estimates bias+gravity online (`t_init=0.2`, 10s windowed inertial opt). Sidesteps the prior entirely — plausibly why it's the most deterministic of the three.

**The capture:** 3.00h static, 1,079,800 samples @ 100Hz, 0 gaps, 12×15-min split segments. LiDAR spinning (in-situ — the OS1 IMU only streams while the sensor operates). Tooling written: `slam-agent/scripts/imu_allan_variance.py` (self-contained overlapping-Allan, numpy+rosbag, no ceres/allan_variance_ros build; math-validated on synthetic to 1.00x). Bags in scratchpad `allan/imu_static_seg_*.bag`.

**Results (calibrated vs hand-set baseline):**
- `gyr_noise` 0.000208 -> **0.000391** rad/s/√Hz (1.9x). HIGH trust — clean -0.5 slope, 3 axes agree.
- `gyr_bias` 0.000004 -> **0.000009** rad/s²/√Hz (2.2x). HIGH — real +0.5 RW tail on 2/3 axes.
- `acc_noise` 0.001249 -> **0.0183** m/s²/√Hz (**15x**). In-situ but **vibration-dominated** (spinning LiDAR couples into its own accel; gyro immune). Defensible AS the noise the filter actually sees in flight.
- `acc_bias` 0.000106 -> **kept**. Allan gave NO measurable accel bias RW (no +0.5 tail; curve still falling at 800s). Auto-fit outlier (0.198) discarded.

**Headline:** the hand-set config made EllipseLIO **over-trust the IMU** — accel by ~15x, gyro ~2x. An EKF told the accel is 15x quieter than reality leans too hard on it → plausible contributor to EllipseLIO's knife-edge 3rd_floor runaway (see [[clean_slate_between_repeated_runs]]).

Calibrated variant: `ellipselio_integration/config/os1_64_ouster_allan.yaml` (baseline `os1_64_ouster.yaml` untouched; runtime copies in `~/ellipselio_ws/src/ellipselio/config/`, differ ONLY in the 3 IMU values). A/B harness: `ellipselio_integration/scripts/ab_imu_config.sh` (interleaved, fresh container/run); `run_bag.sh` now takes optional `[config_file] [out_tag]`.

**A/B RESULT (2026-07-13, 20 runs on 3rd_floor, N=10/arm, interleaved, boosted clock, sensor standby):**
- Baseline: runaway(>3m) **2/10 (20%)**, clean closure median **0.020m** (n=8).
- Allan-cal: runaway **3/10 (30%)**, clean closure median **0.020m** (n=7).
- **NULL RESULT** — 2 vs 3 of 10 is noise (95% CIs overlap hugely); clean closure identical. The Allan calibration does NOT fix EllipseLIO's runaways and does NOT hurt accuracy.
- CONFIRMS [[clean_slate_between_repeated_runs]]: the 3rd_floor non-determinism is multi-threaded FP-reduction-order (component_container_mt + parallel tensor-voting), NOT a mis-specified IMU prior. Fixing the real 15x accel over-trust changed nothing because that wasn't the cause.
- **KEEP the calibration anyway** (physically correct, accuracy-neutral, no downside). Pursue determinism via threading (SingleThreadedExecutor / deterministic reduction), not IMU tuning.

Relates to [[bievr_lio_evaluation]], [[coinlio_ellipselio_photometric_fusion]].
