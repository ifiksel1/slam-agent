# EllipseLIO Integration — Jetson Orin NX / Ouster OS1-64 / ArduPilot

EllipseLIO ("Adaptive LiDAR-Inertial Odometry with an Ellipsoid Representation",
arXiv 2605.21150) packaged for this rig as a ROS 2 Humble Docker container.

Built on **FAST-LIO2 + IKFoM + i-Octree** (all vendored in `include/` — no
GTSAM/Ceres/Sophus). Replaces FAST-LIO2's point-to-plane residuals with two-pass
tensor voting (plane/line/ball classification) — the source of its claimed
confined / feature-poor-space advantage. Candidate replacement for FAST-LIO2.

> Status: **NOT YET VALIDATED on this rig.** Container + configs staged 2026-06-11.
> Run Phase 2 validation + Phase 5 bench/flight testing before trusting for flight.

## Layout

| Path | What |
|------|------|
| `Dockerfile` | `ros:humble-ros-base` (arm64) + OpenMP/PCL/Eigen + Ouster driver deps. Lightweight — no source-built deps. CycloneDDS default rmw. |
| `ros_entrypoint.sh` | Sources ROS 2 + workspace overlay. |
| `config/os1_64_ouster.yaml` | Live OS1-64 config: `/ouster/points` + `/ouster/imu`, `vertical_fov: 42.4`, rig extrinsics. |
| `config/ouster_os1_64_driver.yaml` | Ouster ROS 2 driver params (reused from validated SuperOdom setup). |
| `scripts/build_ws.sh` | One-shot `colcon build` of `ellipselio` + `ouster_ros` in-container. |
| `scripts/run_ellipselio.sh` | Live bringup: container → Ouster driver → EllipseLIO. |

Host workspace: `~/ellipselio_ws/` (mounted to `/root/ros2_ws`; `build/`+`install/`
persist on host). Sources: `ellipselio` (github.com/v4rl-ucy/ellipselio, main),
`ouster-ros` (reused from `superodom_ws`).

## Quickstart

```bash
# 1. Build the image (once)
cd ~/slam-agent/ellipselio_integration && docker build -t ellipselio:humble .

# 2. Build the workspace (once; ~/ellipselio_ws/install persists)
./scripts/build_ws.sh

# 3. Free the Ouster, then bring up live
docker stop superodom slam_gpu_system fast_livo2_slam_autostart 2>/dev/null
./scripts/run_ellipselio.sh

# 4. Verify odometry
docker exec ellipselio bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /ellipselio_odom'
```

## Topics

| I/O | Topic |
|-----|-------|
| Input | `/ouster/points` (PointCloud2, 20Hz), `/ouster/imu` (~100Hz) |
| **Odom (bridge this)** | `/ellipselio_odom` — `nav_msgs/Odometry`, IMU rate, full pose+cov |
| Map / scan | `/cloud_map` (every `pub_map_n_secs`), `/cloud_scan` |
| TF | `odom_ellipselio` → `imu_prop_ellipselio` (low-latency) / `imu_ellipselio` (corrected) |
| Diagnostics | `/analytics` (`ellipselio/EllipseLioAnalytics`, incl. `ram_usage`) |

ArduPilot bridge: feed `/ellipselio_odom` to `vision_to_mavros` (nav_msgs/Odometry in),
same pattern as FAST-LIO/SuperOdom on this rig.

## Tuning & gotchas

- `lidar.vertical_fov` is **tune-critical** (OS1-64 = 42.4). Wrong value silently degrades accuracy.
- `mapping.map_resolution` floor is 0.1 m; raise to 0.15–0.2 if RAM-bound on 8GB (watch `/analytics.ram_usage`).
- `use_sim_time:=false` MANDATORY for live sensors (launch default is `true` = bag playback).
- No `point_filter_num`/`filter_size`/`num_match_points` knobs — the EKF/tensor-voting weights self-adapt.
- ARM64: if the EKF diverges on aarch64 *only*, rebuild `ellipselio` with `-O3` (override the forced `-Ofast`, which implies `-ffast-math`).
