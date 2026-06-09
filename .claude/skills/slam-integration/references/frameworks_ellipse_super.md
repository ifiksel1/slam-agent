# Framework Notes: EllipseLIO & SuperOdom

Ground-truth integration data for two LiDAR-inertial frameworks added 2026-06-08.
Load this when the user selects `ellipse_lio` or `super_odometry`. Both are **ROS 2
only** and both bridge to ArduPilot via `vision_to_mavros` (they output
`nav_msgs/Odometry`). Phase 3 config templates live in `phase3_generation.md`.

---

## EllipseLIO (`ellipse_lio`)

- **Repo:** https://github.com/v4rl-ucy/ellipselio · **Package:** `ellipselio` · **Paper:** arXiv 2605.21150
- **What it is:** "Adaptive LiDAR-Inertial Odometry with an Ellipsoid Representation." Built on **FAST-LIO2 + IKFoM + i-Octree** (all vendored in `include/` — no GTSAM/Ceres/Sophus). Replaces FAST-LIO2's point-to-plane residuals with two-pass **tensor voting** that classifies each map point as plane/line/ball and adaptively reweights the EKF. This is the source of its confined/feature-poor-space advantage and why there are **no `point_filter_num`/`filter_size` knobs** to tune.
- **ROS:** Humble **and** Jazzy. `CMakeLists.txt` auto-detects Jazzy and sets a `ROS_JAZZY` compile define — no manual flag.
- **Sensors:** Ouster (OS64/OS128), Velodyne VLP-16, Hesai QT64, Livox. Ships dataset configs in `config/`.

### Build
```bash
cd ~/colcon_ws/src && git clone https://github.com/v4rl-ucy/ellipselio.git
cd ~/colcon_ws
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```
- Needs `libomp-dev` (`-fopenmp` throughout), Eigen, PCL, OpenCV (camera path only), `pcl_ros`, `cv_bridge`.
- `CMakeLists.txt` forces `-Ofast -fopenmp -fPIC`, C++17. No submodules.

### Run / Topics
```bash
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<ouster>.yaml use_sim_time:=false rviz:=false
```
| I/O | Value |
|-----|-------|
| Input | `lidar.topic` (e.g. `/ouster/points`), `imu.topic` (e.g. `/ouster/imu`) |
| **Odom out (bridge this)** | `/ellipselio_odom` — `nav_msgs/Odometry`, IMU rate, full pose+covariance |
| Map / scan | `/cloud_map` (every `pub_map_n_secs`), `/cloud_scan` |
| TF | `odom_ellipselio` → `imu_prop_ellipselio` (low-latency) / `imu_ellipselio` (LiDAR-corrected) |
| Diagnostics | `/analytics` (`ellipselio/EllipseLioAnalytics`) — incl. `ram_usage` |

### Tuning & gotchas
- `lidar.vertical_fov` is **tune-critical** (OS64=42.4, OS128=90.0) — wrong value silently degrades accuracy.
- `mapping.map_resolution` floor is 0.1 m; raise to 0.15–0.2 if RAM-bound on Orin NX 8GB.
- `r_imu_lidar` accepts a 9-element row-major matrix **or** a 4-element quaternion `[x,y,z,w]` (auto-detected by length).
- `use_sim_time:=false` mandatory for live sensors (default `true` = bags).
- Internal EKF/tensor-voting weights self-adapt; there is **no** `num_match_points`/filter knob.

### ARM64 / Jetson
Low risk: no CUDA, no x86 intrinsics, OpenMP-based. Watchpoints: (1) `-Ofast` implies `-ffast-math` — if EKF diverges on aarch64 only, override to `-O3`; (2) `PCL_NO_PRECOMPILE` makes builds slow/RAM-hungry — cap `-j` on 8 GB; (3) `kMaxMapPoints=10M` preallocates large pools — watch `/analytics.ram_usage`.

---

## SuperOdom (`super_odometry`)

- **Repo:** https://github.com/v4rl-ucy/SuperOdom · **Package:** `super_odometry` · **Paper:** SuperLoc, ICRA 2025
- **What it is:** LiDAR-inertial odometry with bidirectional LiDAR↔IMU fusion, ICP alignment-risk prediction, and **6-DOF degeneracy/uncertainty estimation**. Three nodes: `feature_extraction_node`, `laser_mapping_node`, `imu_preintegration_node`. SLAM and localization modes.
- **ROS:** Humble only (Dockerfile `osrf/ros:humble-desktop-full`; Jazzy untested).
- **Sensors:** Ouster (OS1-128 config; use for OS1-64 with `scan_line: 64`), Velodyne VLP-16, Livox Mid360.

### Build (heavier — source-built deps)
Workspace must contain: `SuperOdom`, `livox_ros_driver2` (**hard dep even for Ouster-only**), `rviz_2d_overlay_plugins`.
```bash
# Sophus  @97e7161:  cmake .. -DBUILD_TESTS=OFF
# GTSAM   @4abef92:  cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..   # MARCH_NATIVE=OFF MANDATORY on aarch64
# Ceres:  apt libceres-dev (2.1.0) OK, or @f68321e
# Livox SDK2 -> cd src/livox_ros_driver2 && ./build.sh humble  (BEFORE colcon)
cd ~/ros2_ws && colcon build
```
apt: `tf2 cv-bridge pcl-conversions xacro robot-state-publisher rviz2 image-transport[-plugins] pcl-ros grid-map[-msgs]` + `libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev libceres-dev libtbb-dev`.

### Run / Topics
```bash
ros2 launch super_odometry os1_128.launch.py \
    config_file:=<pkg>/config/os1_64.yaml \
    calibration_file:=<pkg>/config/ouster/os1_64_calibration.yaml
```
Config is split: a **ROS params YAML** + a **separate OpenCV FileStorage calibration YAML** (`!!opencv-matrix`, passed via `calibration_file`). Calibration rotation is LiDAR→IMU (`imu^R_laser`); translation `imu^T_laser` in meters.

| I/O | Value |
|-----|-------|
| Input | `laser_topic` (`/ouster/points`), `imu_topic` (`/ouster/imu`) |
| **Odom out (bridge this)** | `/state_estimation` — `nav_msgs/Odometry`, high-rate IMU-propagated, frames `map`→`sensor` |
| **Health gate** | `/state_estimation_health` — `std_msgs/Bool`. Use as a HARD gate before fusing |
| Quality | `/super_odometry_stats` — `super_odometry_msgs/OptimizationStats` (per-DoF `uncertainty_x/y/z/roll/pitch/yaw`, match-rejection counts, latency) |
| LiDAR odom / map | `/laser_odometry`, `/laser_cloud_map`, `/registered_scan` |
| Per-axis uncertainty | `/uncertainty_X` … `/uncertainty_yaw` (`std_msgs/Float32`) |

### Modes & gotchas
- `laser_mapping_node.localization_mode`: `false` = mapping; `true` loads `.pcd` prior via `map_dir` (still hardcoded to `/path/to/your/pcd` in launch — override it). `read_pose_file`/`init_*` set the seed pose.
- `scan_line` valid values: 4, 16, 32, 64, 128.
- ⚠️ **Degeneracy caveats (current code):** `nav_msgs/Odometry.pose.covariance` is **not populated**; `pos_degeneracy_threshold`/`ori_degeneracy_threshold` are declared but **never read from YAML** (inert, default 0.0); `isDegenerate` paths are commented out. → For EKF trust, gate on `/state_estimation_health` + watch `/super_odometry_stats`, do **not** rely on odom covariance or `isDegenerate`.
- Default launch has empty `PROJECT_NAME`, so topics sit at root (`/laser_odometry`, etc.).

### ARM64 / Jetson
No CUDA, no x86-only code. Main risk is GTSAM build (20–30 min; cap `-j6` on 8 GB; `gtsam_unstable` must build). `MARCH_NATIVE=OFF` is the critical flag. Calibration loader uses `cv::FileStorage` (standalone OpenCV — not affected by the Noetic cv_bridge ABI issue).

---

## Choosing between them (for the user's Orin NX / OS1-64 / ArduPilot rig)

| | EllipseLIO | SuperOdom | FAST-LIO2 (baseline) |
|---|---|---|---|
| Build weight | **Light** (vendored deps) | Heavy (GTSAM+Ceres+Sophus+Livox) | Light |
| Confined-space claim | Adaptive ellipsoid residuals | Degeneracy-aware fusion | — |
| Health/uncertainty signal | `/analytics` | **Explicit** `/state_estimation_health` + 6-DoF stats | none |
| ROS | Humble + Jazzy | Humble only | Humble + Jazzy |
| Drop-in vs FAST-LIO | Closest (same lineage) | New stack | — |
| First to try | ✅ lower risk | If you need explicit degeneracy gating | current |

Recommended order: validate **EllipseLIO** first (lowest build risk, FAST-LIO lineage, claimed confined-space win), then evaluate **SuperOdom** if you want its explicit degeneracy/health topic for EKF gating.
