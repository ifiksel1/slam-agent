# SuperOdom Integration — Jetson Orin NX 8GB / Ouster OS1-64 / ArduPilot

Working, bench+motion-validated SuperOdometry (`super_odometry`) setup. Sibling to
`fast_livo2_integration/`. SuperOdom is **LiDAR+IMU only (no camera)** — it sidesteps
the FAST-LIVO2 camera-selection blocker.

- Upstream: https://github.com/superxslam/SuperOdom  (V4RL fork: v4rl-ucy/SuperOdom)
- Profile: `docs/profiles/jetson_orin-ouster_os1_64-super_odometry-ardupilot-humble.yaml`
- Framework notes: `.claude/skills/slam-integration/references/frameworks_ellipse_super.md`

## Architecture (this rig)
Host is **Ubuntu 20.04 / Noetic only — no native Humble**, so everything runs in **one
arm64 Humble Docker container** (`superodom:humble`) on host networking: the ROS 2
Ouster driver + SuperOdom together. FAST-LIVO2 (ROS 1) must be stopped first to free
the Ouster.

## Files
| File | Purpose |
|---|---|
| `Dockerfile` | `ros:humble-ros-base` (arm64) + Sophus@97e7161, GTSAM@4abef92 (`MARCH_NATIVE=OFF`, `-j4`), Ceres, Livox SDK2, **libzip-dev** + ouster_ros deps. torch/numba/scikit pip lines dropped (no aarch64 wheels, unused). |
| `config/os1_64.yaml` | SuperOdom params, `scan_line: 64`, topics `/ouster/points` + `/ouster/imu`. |
| `config/ouster/os1_64_calibration.yaml` | OpenCV extrinsic (identity rot + ref translation — CALIBRATE before flight). |
| `config/ouster_os1_64_driver.yaml` | ROS 2 Ouster driver params: sensor `192.168.2.60`, udp_dest `192.168.2.50`, `1024x20`, LEGACY, `point_type: original`. |
| `scripts/run_superodom_bench.sh` | Full bringup: container → ouster driver → SuperOdom. |
| `scripts/odom_trajectory_sampler.py` | Logs `/state_estimation` x/y/z/yaw @25Hz + return-to-start summary. |
| `results/bench_motion_2026-06-09.log` | Captured motion-test trajectory (below). |

## Build notes (gotchas hit + fixed)
- **Base image:** `osrf/ros:humble-desktop-full` is **amd64-only** → built under QEMU on the Jetson. Switched to multi-arch `ros:humble-ros-base`; build with `--platform linux/arm64`.
- **ouster-sdk needs `libzip-dev`** (not in the upstream Dockerfile).
- **Livox `build.sh` `rm -rf`'s `install/`** and does a *full-workspace* colcon build. Do **one unified** `colcon build` (no `--packages-skip`) with `-DROS_EDITION=ROS2 -DDISTRO_ROS=humble`; don't skip livox (super_odometry build-depends on it).
- 8 GB / 6-core: cap `colcon build --parallel-workers 1`, `MAKEFLAGS=-j3`; GTSAM `-j4` (9.6 GB swap as buffer).

## Bench + motion validation (2026-06-09)
Stationary: `/state_estimation` **25 Hz**, `/state_estimation_health: true`, no NaN, valid `map→sensor` TF.
Motion (lift → move → rotate left 90° → return): translation (x −0.24→+1.03 m) and lift
(z −0.60→+0.12 m) tracked; **rotation measured −91° for a 90° left turn**; position
loop-closure **~0.13 m** (excl. startup transient).

### Degeneracy characterization (featureless corridor, 2026-06-09)
See `results/DEGENERACY_FINDINGS.md`. 54.67 m corridor walk (bag-record + 0.5× offline replay).
- uncertainty x/yaw pinned at max (~94% / always); z best-constrained; drift ~0.79 m / 54.67 m (~1.4%).
- **`/state_estimation_health` stayed `true` the whole time** — it does NOT detect this degeneracy.
- Yaw came apart (±130° jumps) at the deepest stretch.

### ⚠️ Live limitation on this rig
The ROS2 Ouster driver (1024×20) + SuperOdom together overloads the 8 GB/6-core Orin → LiDAR
drops to ~11 Hz → LiDAR-IMU sync fails → no live tracking. Driver alone holds 20 Hz. **Validated
method here is bag-record (driver only) + offline replay.** For LIVE flight: try 512×20, lighten
SuperOdom, use beefier compute, or prefer the lighter EllipseLIO for live and keep SuperOdom for
offline/analysis.

### Extrinsic correction (2026-06-09) — IMPORTANT
The initial bench/motion/degeneracy runs used a **placeholder identity** LiDAR↔IMU extrinsic.
That was wrong for this rig and caused the yaw instability (±130° jumps) and the apparent
"yaw sign inverted" finding. Corrected to this rig's real Ouster extrinsic (from the working
FAST-LIO config): `extrinsicRotation_imu_laser = [-1,0,0, 0,-1,0, 0,0,1]` (OS1 IMU is yaw-flipped
vs the LiDAR), translation `[-0.006253, 0.011775, 0.028535]`, plus `use_imu_roll_pitch: true`.
**Result: the replayed path is now stable.** SuperOdom still builds the world in the sensor's
start orientation (not full gravity-up like FAST-LIO), so the map renders **upside-down**; for
viewing, follow a 180°-roll `map_view` frame; for flight, the vision bridge must apply this
world-Z/heading correction. Re-run the degeneracy replay with the corrected extrinsic for final
drift/yaw numbers (the qualitative degeneracy + health-flag findings still stand).

### ⚠️ Before ArduPilot flight
1. **World-Z + yaw frame correction** in `vision_to_mavros` (SuperOdom world is not ENU/gravity-up
   on this rig). Re-verify yaw sign on a fresh motion test now that the extrinsic is fixed.
2. **Slow yaw drift ~5–7°/min at rest** — yaw is the least-observable DOF; gate fusion on
   `/state_estimation_health` + `/super_odometry_stats` uncertainty.
3. **Measure the real LiDAR–IMU extrinsic** into `os1_64_calibration.yaml`.
4. odom `pose.covariance` is not populated by SuperOdom — drive EKF trust from health/stats.

`integration_complete: false` until flight-tested.
