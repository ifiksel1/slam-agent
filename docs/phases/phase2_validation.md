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

### ROS Compatibility
| Algorithm | ROS1 Noetic | ROS2 Humble | ROS2 Foxy |
|-----------|-------------|-------------|-----------|
| FAST-LIO2 | Yes | Yes | Yes |
| LIO-SAM | Yes | Yes | Yes |
| COIN-LIO | Yes | Partial | No |
| Cartographer | Yes | Yes | Yes |
| ORB-SLAM3 | Yes | Community | No |
| OpenVINS | Yes | Yes | Yes |
| VINS-Fusion | Yes | Community | No |

### Communication Method
| ROS + FC | Method | Package |
|----------|--------|---------|
| ROS1 + ArduPilot | MAVROS | ros-noetic-mavros |
| ROS1 + PX4 | MAVROS | ros-noetic-mavros |
| ROS2 + ArduPilot | MAVROS (default) or DDS | ros-humble-mavros or micro-ros-agent |
| ROS2 + PX4 | DDS (native) | micro-ros-agent |

## Validation Checks

Run these checks against the config:

1. **Compute**: config.platform.ram_gb >= algorithm.min_ram? Warn if not.
2. **Sensor**: Does algorithm support config.lidar.model type? (Almost all support PointCloud2)
3. **ROS**: Is algorithm available for config.ros.version?
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
