# Adaptive-LIO integration (Dragunfly rig)

[Adaptive-LIO](https://github.com/chengwei0427/adaptive_lio) — adaptive LiDAR-Inertial Odometry
(adaptive frame segmentation, IMU-fault LIO⇄LO switching, multi-resolution voxel map) —
integrated on the same hardware and transforms as EllipseLIO / FAST-LIVO2:

**Jetson Orin NX 8 GB · Ouster OS1-64 (internal ICM-20948 IMU) · Cube Orange (ArduPilot 4.5) · MAVROS.**

Unlike EllipseLIO (ROS 2 / colcon), Adaptive-LIO is **ROS 1 Noetic / catkin / Ceres**, so it is
built and packaged like the COIN-LIO / FAST-LIO ROS 1 stack — in its own dedicated container
(`adaptive-lio:noetic`), not EllipseLIO's ROS 2 image.

## What was mirrored vs. changed from EllipseLIO

| Contract | Value | Source |
|---|---|---|
| Platform / LiDAR / FC | Orin NX · Ouster OS1-64 · Cube Orange | identical |
| `base_link → os_sensor` | `xyz [0,0,-0.060] rpy [0,π,0]` (upside-down, 6 cm below FCU) | `urdf/drone.urdf` (verbatim) |
| LiDAR↔IMU extrinsic | **identity** (`extrinsic_T [0,0,0]`, `extrinsic_R I`) + online refine | ⚠ **changed** — see note |
| Topics | `/ouster/points`, `/ouster/imu` (1024×20 → 20 Hz lidar, 100 Hz IMU) | identical |
| ArduPilot EKF/VISO | EK3 SRC1 all = ExternalNav(6), `VISO_TYPE 2`, GPS off | `config/ardupilot_params.parm` (verbatim) |
| Odometry out | `/odom` (`nav_msgs/Odometry`, `alio_odom → alio_body`) | Adaptive-LIO native |

**⚠ Extrinsic note:** EllipseLIO used `diag(-1,-1,1)` (180° yaw) because that was a *ROS 2 Ouster
driver* artifact. On the **ROS 1** driver the FAST-LIO family uses the **identity** extrinsic on
this rig (repo `config/ouster64.yaml`), so identity is correct here. `extrinsic_est_en: true`
online-refines it regardless.

## Two integration gotchas handled

1. **Config path is compile-time** — Adaptive-LIO reads `ROOT_DIR + config/mapping_m.yaml`, not a
   ROS param. The build bakes `config/mapping_ouster.yaml` over it, and the run scripts bind-mount
   it so you can tune without recompiling.
2. **Hardcoded frames collide with MAVROS** — Adaptive-LIO broadcasts TF `map→base_link` and
   `world→map` with those literal names. `patches/namespace_frames.sh` renames them to
   `alio_world / alio_odom / alio_body` at build time; `master.launch` then bridges into the real
   `map`/`base_link` with static TFs (same chain as FAST-LIVO2).

## Layout

```
adaptive_lio_ws/
├── Dockerfile                      ROS1 Noetic + Ceres 2.1 + livox_ros_driver + Adaptive-LIO
├── package.xml / CMakeLists.txt    makes this a catkin pkg so $(find adaptive_lio_ws) resolves
├── config/
│   ├── mapping_ouster.yaml         Adaptive-LIO config ported to Ouster OS1-64 (baked as mapping_m.yaml)
│   └── ardupilot_params.parm       EKF3 external-nav params (shared rig, verbatim)
├── urdf/drone.urdf                 base_link → os_sensor (shared rig, verbatim)
├── launch/
│   ├── adaptive_lio.launch         SLAM node only (bag or live)
│   └── master.launch               MAVROS + URDF + SLAM + static TFs + vision_to_mavros
├── patches/namespace_frames.sh     alio_* frame renaming (applied at build)
└── scripts/
    ├── build_image.sh              docker build  (--live also builds the Ouster ROS1 driver)
    ├── run_bag.sh                  N-run distributional bag battery (comparable to run_coinlio_bag.sh)
    ├── run_live.sh                 live Ouster → Adaptive-LIO → MAVROS full stack
    └── adaptive_lio_traj_sampler.py  frame-invariant trajectory metrics on /odom
```

## Quick start

```bash
# 1. Build (bag-replay only; add --live for the Ouster driver too)
./scripts/build_image.sh

# 2. Boost the clock (Adaptive-LIO is CPU-bound; stock 1497 MHz diverges on this Jetson)
sudo ~/superodom_ws/set_cpu_clock.sh boost

# 3. Validate on a converted ROS1 bag (must carry /ouster/points + /ouster/imu)
#    rosbags-convert <ros2_bag_dir> --dst ~/adaptive_lio_bags/3rd_floor.bag
./scripts/run_bag.sh ~/adaptive_lio_bags/3rd_floor.bag 6      # 6-run distribution + runaway rate

# 4. Live full stack (after bag validation)
./scripts/build_image.sh --live
./scripts/run_live.sh
```

## Build notes (two fixes baked into the Dockerfile)

The image **builds clean on ARM64 Noetic** (`adaptive-lio:noetic`, 4.18 GB) with Ceres 2.1.0.
Two upstream gotchas are handled automatically:

1. **Vendored `livox_ros_driver`** — Adaptive-LIO already ships `thirdparty/livox_ros_driver`
   (message-only, pulled by `git clone --recursive`). Do **not** clone it into `src/` as well —
   the two copies collide on the `livox_ros_driver_generate_messages` target. Livox-SDK isn't needed.
2. **C++17, not C++14** — upstream `CMakeLists.txt` hardcodes `-std=c++14`, but `eigen_types.h`
   uses a `constexpr` captureless lambda (`less_vec2i`) that is only a literal type from C++17 on.
   The Dockerfile `sed`s `c++14 → c++17` before `catkin_make`.

## Status / remaining (Phase 5)

- Config + container + harness generated; **image built & smoke-tested** (node, baked config,
  `alio_*` frames, glog/PCL linkage all verified). **Not yet run** on a bag or the live sensor.
- **Verify on hardware before flight:**
  - **Yaw sign** — measure a known +90° CCW turn; if world yaw inverts, negate it (add
    `zero_initial_yaw.py sign:=-1.0`, as FAST-LIVO2 does). Left neutral in `master.launch`.
  - **`alio_body → base_link`** assumes Adaptive-LIO's body frame == `os_imu`; confirm with
    `check_tf_tree` before Loiter/Guided.
  - **`gnorm`** set to 9.805 (Ouster IMU is m/s²); confirm the estimator initializes gravity cleanly.
