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

### ⚠️ Before ArduPilot flight
1. **Yaw sign is inverted vs ENU** (physical left → negative yaw). The `vision_to_mavros`
   bridge needs a frame/sign correction or heading will be wrong (flyaway risk).
2. **Slow yaw drift ~5–7°/min at rest** — yaw is the least-observable DOF; gate fusion on
   `/state_estimation_health` + `/super_odometry_stats` uncertainty.
3. **Measure the real LiDAR–IMU extrinsic** into `os1_64_calibration.yaml`.
4. odom `pose.covariance` is not populated by SuperOdom — drive EKF trust from health/stats.

`integration_complete: false` until flight-tested.
