# LiDAR↔IMU Calibration with LI-Init (Hesai JT128)

Independently re-derives the LiDAR↔IMU extrinsic **and** the LiDAR/IMU time offset
using [hku-mars/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init).

**Everything here is offline and read-only.** The calibration container runs with
`--network none` and has no path to the flight controller.

---

## Why this exists

`config/hesai_jt128.yaml` carries an extrinsic labelled "Calibrated IMU->LiDAR":

```yaml
extrinsic_T: [ 0.006622, -0.009142, -0.039155 ]
extrinsic_R: [ 0.998625, -0.032668, -0.040998,
               0.033462,  0.999262,  0.018837,
               0.040353, -0.020183,  0.998982 ]
```

That matrix is a valid rotation (`det = 1.000000`, orthonormal to 1e-6) encoding
**3.21°** total — roll/pitch/yaw of −1.16° / −2.31° / +1.92°. Two reasons to doubt it:

1. **Provenance.** `git blame` puts the values and the comment
   `extrinsic_est_en: false  # online extrinsic estimation causes drift` in the *same*
   commit (`1e0d985`). They are almost certainly the frozen output of the online
   estimator that commit was disabling — not an independent calibration.
2. **Timing changed underneath it.** The estimator ran under `use_timestamp_type: 0`.
   The rig has since moved to `use_timestamp_type: 1` with `restamp_system_time: false`.
   Rotation and time offset are **correlated during motion**, so a timing error can be
   absorbed into the rotation — fitting a condition that no longer exists.

3.21° is not negligible: 0.55 m/s² of gravity misprojection, ~5.6 cm of cross-track
error per metre travelled, and up to 3.21° of yaw error through a 90° turn.

LI-Init is the right tool because it estimates the extrinsic **and the time offset
jointly**, which tests that hypothesis directly. It is also by the FAST-LIO authors,
so it shares the front end.

> Setting the rotation to identity by hand is **not** a fix. If the true offset really
> is ~3°, identity introduces exactly the error you meant to remove. The cost is
> symmetric — measure, don't guess.

---

## Feasibility (verified on this rig)

| Requirement | Status |
|---|---|
| ROS 1 | ✅ Noetic |
| Per-point `time` + `ring` | ✅ `x,y,z,intensity,ring(uint16),time(float32)` |
| IMU rate | ✅ 436 Hz measured |
| Hesai support | ✅ explicitly supported upstream |
| Geometry | ✅ 360° azimuth × 92° — a ~180° forward hemisphere, matching `fov_degree: 180` |
| Range | ⚠️ short: median 2.4 m, p95 6.9 m, max 21.7 m — environment choice matters |

### `lidar_type: 2`, not `5`

Upstream ships `pandar.yaml` with `lidar_type: 5` (Hesai PandarXT). **We use `2`
(Velodyne) anyway.** The decoder is selected by *point struct*, not by vendor: the
JT128 driver emits a **relative `float32 time`** (Velodyne convention), whereas the
Pandar decoder expects an **absolute `double timestamp`**. Verified against the live
topic, and consistent with `lidar_type: 2` already working in `hesai_jt128.yaml`.

---

## Procedure

### 1. Build (once, ~10 min — Ceres 2.0.0 from source)

```bash
docker build -t li-init:latest docker/li_init/
```

Separate image by design. The flight container `slam-hesai-fastlio` runs the live
vision→FC pipeline and must not acquire heavyweight build dependencies. Noetic ships
Ceres 1.14, but LI-Init is tested against 2.0.0 and the APIs differ enough to break
the build, so the image compiles 2.0.0 from source. `livox_ros_driver` is a hard
dependency even for non-Livox sensors — LI-Init includes its message headers
unconditionally.

The image records the exact upstream commits in `/root/li_ws/BUILD_VERSIONS.txt` and
prints them with every run, so a result can be traced to the code that produced it.

### 2. Record an excitation bag

```bash
scripts/li_init/record_calib_bag.sh calib.bag 120
```

> ⚠️ **HANDHELD, MOTORS DISARMED, PROPS OFF.** A static data recording, not a flight
> test. The script refuses to record if `/mavros/state` reports `armed: True`.

**The bag quality dominates the result.** The extrinsic is only observable under
excitation on all three rotation axes plus translation. A degenerate bag yields a
*confidently wrong* extrinsic — plausibly how the incumbent 3.21° came to exist.

- Hold **completely still for the first ~10 s** (upstream needs >5 s to accumulate the
  initial map). Do not skip this.
- Then continuously: roll ±30–45°, pitch ±30–45°, yaw ±45–90°, translate ~0.5–1 m on
  each axis, finishing with slow figure-eights that combine rotation and translation.
- Keep it **smooth** — jerks saturate the IMU, too slow is unobservable.
- **Environment matters as much as motion.** Stand where there is planar structure at
  several *orientations* (walls + floor + ceiling), a few metres out. Given the ~22 m
  max range, an empty corridor or a wide-open space will not constrain the solution.

### 3. Run

```bash
scripts/li_init/run_li_init.sh calib.bag
```

Replays the bag on the container's own ROS master with `--network none`. Results and
the full log land in `li_init_result/`.

The runner prints LI-Init's **per-axis excitation meter** as the first thing after the
run, because that is what you check when no calibration is produced:

```
=================== EXCITATION =======================
Rotation around Lidar X Axis:   0%
Rotation around Lidar Y Axis:   0%
Rotation around Lidar Z Axis:   0%
```

All three need to reach ~100%. The example above is a real run against an ordinary
flight bag — no rotational excitation at all, hence no result. That is the expected
failure mode for a bag not recorded specifically for calibration.

> **Do not run LI-Init live against the flight pipeline.** Upstream suggests running it
> live for real-time excitation guidance. On this rig that is unsafe: LI-Init runs a
> full FAST-LIO front end, and the CPU contention can starve `laserMapping` — which
> trips `drift_monitor`, which is configured with `enable_auto_restart=true` **and**
> `enable_source_failover=true`. It would then command an EKF source failover on the
> live FC (`MAV_CMD_DO_AUX_FUNCTION` 218) and restart nodes. Record first, replay
> after. If you genuinely want live guidance, stop the live FAST-LIO pipeline first and
> run a dedicated calibration session.

Because the feedback loop is record→replay rather than live, budget for re-recording.
Replaying at `-r 0.5` gives the meter more time to settle if the run looks marginal.

---

## Interpreting the result

Compare against the incumbent (`extrinsic_T` above, `extrinsic_R` = 3.21°):

| Outcome | Reading |
|---|---|
| Converges near **identity** rotation | Incumbent was likely a bad fit, plausibly absorbing the old timing error. Candidate for replacement. |
| Reproduces **~3.21°** | Incumbent is real. Leave it alone. |
| Large **time offset** | *That* is the finding — timing was being absorbed into the rotation. Fix timing first, then re-run. |
| Won't converge / insufficient excitation | The **bag** is the problem, not the extrinsic. Re-record with more rotation. |

### Do not write a new extrinsic on the strength of one run

Validate first: replay a separate **closed-loop** bag (starts and ends at the same
physical spot) through `scripts/replay/run_one.sh` with old vs. new values and compare
end-to-end position closure with `scripts/replay/compare_bags.py`. Smaller closure
error wins. If they are within noise, the extrinsic is not your problem.

`extrinsic_est_en` must stay `false` in the live config regardless of the outcome.

---

## Tuning knobs

- **`data_accum_length: 300`** — raise if it reports insufficient data.
- **`gyr_cov` / `acc_cov`** — upstream ships wildly different values per sensor
  (`velodyne.yaml` 0.5/0.5, `pandar.yaml` 20/2). We start neutral at 0.5/0.5; if the
  extrinsic wanders or won't converge, try 20/2.
- **`mean_acc_norm: 9.7977`** — measured on this unit (8690 samples over 20 s
  stationary, sd 0.028 m/s²), not the 9.805 default. Gravity sits on **+y**, consistent
  with the body-frame convention. Re-measure if the IMU is replaced.
- **`filter_size_surf` / `filter_size_map`** — 0.1 / 0.15, set for this short-range
  indoor sensor.

---

## Related

- `docs/VISION_LATENCY_MEASUREMENT.md` — the timing side of the same coupling
- `docs/troubleshooting/sensor_calibration.md` — method overview
- `docs/learned/solutions_log.yaml` entry 4 — the `use_timestamp_type: 1` change
- `scripts/replay/` — offline FAST-LIO replay harness used for validation
