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
| `scripts/setup.sh` | **Turnkey: clone sources → stage configs → build image + workspace.** Run this first on a fresh clone. |
| `Dockerfile` | `ros:humble-ros-base` (arm64) + Sophus@97e7161, GTSAM@4abef92 (`MARCH_NATIVE=OFF`, `-j4`), Ceres, Livox SDK2, **libzip-dev** + ouster_ros deps. torch/numba/scikit pip lines dropped (no aarch64 wheels, unused). |
| `config/os1_64.yaml` | SuperOdom params, `scan_line: 64`, topics `/ouster/points` + `/ouster/imu`. |
| `config/ouster/os1_64_calibration.yaml` | OpenCV extrinsic (identity rot + ref translation — CALIBRATE before flight). |
| `config/ouster_os1_64_driver.yaml` | ROS 2 Ouster driver params: sensor `192.168.2.60`, udp_dest `192.168.2.50`, `1024x20`, LEGACY, `point_type: original`. |
| `scripts/run_superodom_bench.sh` | Full bringup: container → ouster driver → SuperOdom. |
| `scripts/odom_trajectory_sampler.py` | Logs `/state_estimation` x/y/z/yaw @25Hz + return-to-start summary. |
| `results/bench_motion_2026-06-09.log` | Captured motion-test trajectory (below). |

### Sources (pinned by `setup.sh`) — all PUBLIC
| Package | Repo | Branch |
|---|---|---|
| `SuperOdom` | `github.com/superxslam/SuperOdom` | `ros2` |
| `ouster-ros` | `github.com/ouster-lidar/ouster-ros` | `ros2` |
| `livox_ros_driver2` | `github.com/Livox-SDK/livox_ros_driver2` | `master` |
| `rviz_2d_overlay_plugins` | `github.com/teamspatzenhirn/rviz_2d_overlay_plugins` | `main` |

## Quickstart (fresh clone → running)
```bash
# 0. One-time turnkey setup: clones all sources (pinned, public), stages configs, builds
#    the image (source-builds GTSAM/Sophus/Ceres — slow first run) + workspace. Idempotent.
cd ~/slam-agent/superodom_integration && ./scripts/setup.sh

# 1. Set YOUR sensor + host IP / lidar_mode, and CALIBRATE the extrinsic before flight
$EDITOR ~/superodom_ws/src/ouster_os1_64_driver.yaml
$EDITOR ~/superodom_ws/src/SuperOdom/super_odometry/config/ouster/os1_64_calibration.yaml

# 2. Free the Ouster (stop ROS1 FAST-LIVO2), then bring up
docker update --restart=no fast_livo2_slam_autostart && docker stop fast_livo2_slam_autostart 2>/dev/null
./scripts/run_superodom_bench.sh
```
Env overrides: `SUPERODOM_WS`, `SUPERODOM_SRC_URL`/`SUPERODOM_BRANCH` (+ `OUSTER_ROS_*`,
`LIVOX_*`, `OVERLAY_*`), `SUPERODOM_IMAGE`, `SENSOR_IP`/`HOST_IP` (display only).

> **Field lessons (read before flight):** [`docs/lessons/ellipselio_superodom_findings.md`](../docs/lessons/ellipselio_superodom_findings.md)
> (don't trust health/uncertainty — gate externally on kinematics; yaw-sign inversion; transient is
> CPU-contention-driven) and [`docs/lessons/jetson_orin_ouster_operational.md`](../docs/lessons/jetson_orin_ouster_operational.md)
> (CycloneDDS, clock boost, bag validation, STANDBY).

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

### Live rate: ROOT CAUSE was Fast-DDS, FIXED with CycloneDDS (2026-06-09)
The `/ouster/points` drop under SuperOdom (earlier called a "live limitation") was the **ROS 2
middleware**, not compute. Default **Fast-DDS (`rmw_fastrtps_cpp`)** stalls the large PointCloud2
publish when several SuperOdom DDS participants are active → `/ouster/points` collapses to ~15 Hz.
**Switching to CycloneDDS restores a steady 20 Hz.**

How it was isolated — every compute hypothesis ruled out by direct live test:

| Hypothesis | Test | Result |
|---|---|---|
| RAM | — | 24% used, 5.6 GB free → no |
| Single-core saturation | driver pinned to a dedicated core (`taskset -c 0` + `nice -20`) | core at 62%, still ~17 Hz → no |
| UDP buffer overflow | 10 s live delta | **0 drops** (the 1352 were stale) → no |
| `ros2 topic hz` artifact | reporter on dedicated core + `nice -20` | still ~17 Hz → no |
| Memory bandwidth | `tegrastats` | `EMC_FREQ 0%` → no |
| Thermal | `tegrastats` | 66–70 °C → no |
| **CPU clock** | raised 1497→1984 MHz (sysfs `scaling_max_freq`) | CPU util **dropped to ~50%** but rate stayed ~15 Hz → **no** |
| Reliable-QoS backpressure | `ros2 topic info -v` | both ends **BEST_EFFORT** → no |
| `/points` subscription coupling | stopped the only `/points` subscriber | still ~13 Hz with **0 subscribers** → no |

**Decisive isolation:** raw `/ouster/lidar_packets` held **1234 Hz rock-steady** under full load
while assembled `/ouster/points` collapsed — so the loss is in the driver's **large-PointCloud2
assembly/publish path**, triggered merely by other SuperOdom DDS participants being alive. That is
Fast-DDS large-message + multi-participant contention.

**The fix (baked in):**
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # on BOTH the driver and SuperOdom (must match)
```
Result: `/ouster/points` **20.0 Hz steady** (std 0.011 s), sustained, 69.8 °C @ 1984 MHz. Now in
the Dockerfile (`ros-humble-rmw-cyclonedds-cpp` + `ENV RMW_IMPLEMENTATION`) and
`run_superodom_bench.sh`. **This lifts the "not viable live at 1024×20" caveat at the bench level —
a live *motion* test is still needed to confirm tracking** (stationary, the nodes sit in cold-start
"throw laser scan").

> **CPU headroom / power mode (secondary):** under CycloneDDS at full 20 Hz, SuperOdom is
> CPU-bound — cores ~73–84% @ 1984 MHz. Holding 20 Hz at the **stock 25 W cap (1497 MHz)** is
> **untested** (likely OK since the bottleneck was DDS, but verify). Note `nvpmodel -m 0` (MAXN) is
> **broken on this module**: the stock `/etc/nvpmodel.conf` MAXN entry references CORE_6/CORE_7 but
> the **Orin NX 8 GB is a 6-core part** (16 GB = 8-core) → `NVPM ERROR cpu6/online`. Boost instead
> via per-core sysfs `echo 1984000 > scaling_max_freq` + `performance` governor (non-persistent;
> reverts on reboot). For a persistent boost, fix the MAXN conf to 6 cores or add a boot unit.

> **Image fixed (2026-06-09):** `superodom:humble` was **rebuilt** with `libzip-dev` (and now
> `rmw-cyclonedds-cpp`) baked in. BuildKit reused the cached GTSAM/Sophus layers — only the apt
> layer re-ran. A fresh container no longer aborts `os_driver` on `libzip.so.4`. Use
> `DOCKER_BUILDKIT=1` for incremental rebuilds; the classic builder (`DOCKER_BUILDKIT=0`) misses the
> cache and triggers a full ~30-min GTSAM rebuild.

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
