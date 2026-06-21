# Phase 2: Validation & Compatibility Check

## Input
`slam_hardware_config.yaml` from Phase 1.

## Compatibility Matrix

### Compute Requirements
| Algorithm | Min RAM | Min Cores | GPU Required | Notes |
|-----------|---------|-----------|-------------|-------|
| FAST-LIO2 | 4GB | 4 | No | Runs on Jetson Nano |
| LIO-SAM | 8GB | 4 | Helpful | GTSAM is memory-heavy |
| COIN-LIO | 6GB | 4 | No | Higher CPU than FAST-LIO |
| Cartographer | 6GB | 4 | No | |
| LeGO-LOAM | 4GB | 4 | No | |
| ORB-SLAM3 | 4GB | 4 | Helpful | GPU speeds feature extraction |
| RTAB-Map | 6GB | 4 | Helpful | Memory grows with map |
| OpenVINS | 2GB | 2 | No | Lightest VIO |
| VINS-Fusion | 4GB | 4 | No | |
| Point-LIO | 4GB | 4 | No | |
| EllipseLIO | 4GB | 4 | No | FAST-LIO2 lineage; vendored IKFoM/i-Octree, no GTSAM/Ceres. Strong in confined/degenerate spaces |
| SuperOdom | 8GB | 4 | No | GTSAM+Ceres source build; 6-DOF degeneracy estimation. Heavier ARM64 build |

### Sensor Support
| Algorithm | LiDAR Types | Camera Types | IMU Required |
|-----------|-------------|-------------|-------------|
| FAST-LIO2 | Any PointCloud2 | N/A | Yes (LiDAR or FC) |
| LIO-SAM | Any PointCloud2 | N/A | Yes (prefers external) |
| COIN-LIO | Any PointCloud2 | N/A | Yes |
| Cartographer | Any PointCloud2, LaserScan | N/A | Optional |
| ORB-SLAM3 | N/A | Mono/Stereo/RGB-D | Optional |
| OpenVINS | N/A | Mono/Stereo | Yes (required) |
| VINS-Fusion | N/A | Mono/Stereo | Yes (required) |
| LVI-SAM | Any PointCloud2 | Mono | Yes |
| R3LIVE | Any PointCloud2 | Mono | Yes |
| EllipseLIO | Any PointCloud2 (Ouster/Velodyne/Livox/Hesai) | N/A | Yes (LiDAR or external) |
| SuperOdom | Any PointCloud2 (Ouster/Velodyne/Livox) | N/A | Yes (LiDAR or external) |

### ROS Distribution Lifecycle
| Distro | Ubuntu | EOL | Status |
|--------|--------|-----|--------|
| Noetic (ROS 1) | 20.04 | May 2025 | Final ROS 1 release |
| Humble (ROS 2) | 22.04 | May 2027 | **Recommended** — widest ecosystem |
| Jazzy (ROS 2) | 24.04 | May 2029 | Recommended for new Ubuntu 24.04 systems |
| Iron (ROS 2) | 22.04 | Nov 2024 | **EOL** — migrate to Humble |
| Foxy (ROS 2) | 20.04 | May 2023 | **EOL** — migrate to Humble or Jazzy |

If user selects an EOL distro, warn them and recommend the active alternative for their Ubuntu version.

### ROS Compatibility
| Algorithm | ROS1 Noetic | ROS2 Humble | ROS2 Jazzy | Notes |
|-----------|-------------|-------------|------------|-------|
| FAST-LIO2 | Yes | Yes | Yes (builds clean) | Same colcon process on all ROS 2 |
| FAST-LIO2 GPU | Yes | Yes (tested) | Untested | CUDA-dependent, not distro-dependent |
| LIO-SAM | Yes | Yes | Yes | GTSAM available on both |
| COIN-LIO | Yes | Partial | Untested | May need C++17 flag adjustments |
| Cartographer | Yes | Yes (binary) | Yes (binary) | `ros-{distro}-cartographer-ros` |
| ORB-SLAM3 | Yes | Community | Untested | Depends on OpenCV/Pangolin version |
| OpenVINS | Yes | Yes | Yes | Pure CMake, distro-independent |
| VINS-Fusion | Yes | Community | Untested | cv_bridge API may differ on Jazzy |
| Point-LIO | Yes | Yes | Yes (builds clean) | Same as FAST-LIO2 |
| EllipseLIO | **No** | Yes | Yes | ROS 2 only; CMake auto-detects Jazzy via `ROS_JAZZY` define. No GTSAM/Ceres |
| SuperOdom | **No** | Yes | Untested | ROS 2 Humble only (Dockerfile is humble). GTSAM `MARCH_NATIVE=OFF` mandatory on ARM64 |

### Communication Method
| ROS + FC | Method | Package |
|----------|--------|---------|
| ROS1 + ArduPilot | MAVROS | `ros-noetic-mavros` |
| ROS1 + PX4 | MAVROS | `ros-noetic-mavros` |
| ROS2 + ArduPilot | MAVROS (default) or DDS | `ros-{distro}-mavros` or `ros-{distro}-micro-ros-agent` |
| ROS2 + PX4 | DDS (native) | `ros-{distro}-micro-ros-agent` |

Replace `{distro}` with `humble` or `jazzy` as appropriate. MAVROS and micro-ROS Agent are available in both Humble and Jazzy repos.

## Validation Checks

Run these checks against the config:

1. **Compute**: config.platform.ram_gb >= algorithm.min_ram? Warn if not.
2. **Sensor**: Does algorithm support config.lidar.model type? (Almost all support PointCloud2)
3. **ROS distro compatibility** (critical):
   - Check the algorithm's row in the ROS Compatibility matrix above
   - Check the sensor driver's distro support (see `references/ros2_distributions.md`)
   - If user chose Noetic but algorithm + drivers support ROS 2 → recommend switching to ROS 2
   - If user chose Jazzy but algorithm is "Untested" or "Community" → recommend Humble instead
   - If user chose Foxy or Iron → warn EOL, recommend Humble or Jazzy
4. **Camera**: If VIO algorithm, is camera present?
5. **IMU**: If LIO algorithm, is IMU source configured?
6. **LiDAR**: If LiDAR algorithm, is LiDAR present?
7. **Raspberry Pi warning**: If Pi + LiDAR SLAM → warn about performance, suggest VIO instead

## Summary Template

Present to user for confirmation:

```
SLAM Integration Configuration Summary
=======================================
Hardware:
  Platform: [model] ([ram]GB RAM, [cores] cores, [gpu])
  OS: [os]
  LiDAR: [model] ([channels] ch, [connection])
  Camera: [model] ([type]) or None
  FC: [model] ([autopilot] [firmware])

Software:
  SLAM: [algorithm]
  ROS: [version]
  Comm: [mavros/dds]
  IMU Source: [lidar/fc] ([imu_model]) - [reasoning]

Physical:
  LiDAR offset: [x, y, z] m, rotation: [r, p, y]°
  Camera offset: [x, y, z] m, rotation: [r, p, y]°
  Transform: [urdf/static_tf]

Environment: [type], [area], [features]
Mission: [speed] m/s, [duration] min, loop closure: [yes/no]
Docker: [yes/no]

Compatibility: [ALL PASS / WARNINGS]
```

## Generate install_config.yaml

After user confirms, generate `scripts/install_config.yaml`:

```yaml
ros_version: "ROS1"  # or "ROS2"
ros_distro: "noetic"  # or "humble", "foxy", etc.
workspace_path: "~/catkin_ws"  # or "~/ros2_ws" for ROS2
flight_controller: "ardupilot"  # or "px4"
use_dds: false  # true if ROS2+PX4 or user chose DDS
lidar_type: "ouster"  # sensor brand
camera_type: "realsense"  # or "zed", "none", etc.
slam_algorithm: "fast_lio"  # algorithm identifier
```

## Output
- Validated config (confirmed by user)
- `scripts/install_config.yaml` generated
- Progress YAML updated with Phase 2 complete
- Ready for Phase 3
