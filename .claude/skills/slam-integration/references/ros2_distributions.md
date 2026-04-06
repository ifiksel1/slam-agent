# ROS 2 Distribution Reference

## Active Distributions

| Distro | Ubuntu | Release | EOL | LTS | Recommended |
|--------|--------|---------|-----|-----|-------------|
| **Jazzy Jalisco** | 24.04 Noble | May 2024 | May 2029 | Yes | New projects on Ubuntu 24.04 |
| **Humble Hawksbill** | 22.04 Jammy | May 2022 | May 2027 | Yes | Default choice, widest ecosystem |
| **Iron Irwini** | 22.04 Jammy | May 2023 | Nov 2024 | No | **EOL** -- do not use for new projects |
| **Foxy Fitzroy** | 20.04 Focal | Jun 2020 | May 2023 | Yes | **EOL** -- migrate to Humble or Jazzy |
| Rolling | Latest | Continuous | N/A | No | Development/testing only |

## Recommendation Logic

**Always recommend ROS 2 over ROS 1.** Only suggest Noetic if the user has a compelling reason (existing ROS 1 codebase they can't migrate, or a SLAM algorithm with no ROS 2 port).

### Step 1: Check algorithm + driver compatibility (most important)

Cross-reference the user's chosen SLAM algorithm and sensor drivers against the ROS Compatibility matrix in Phase 2. This is the primary constraint:

| Situation | Distro |
|-----------|--------|
| Algorithm has **tested** ROS 2 Humble support | **Humble** (safest) |
| Algorithm has **tested** ROS 2 Jazzy support | **Jazzy** (if on Ubuntu 24.04) |
| Algorithm only has **community** ROS 2 port (e.g., ORB-SLAM3, VINS-Fusion) | **Humble** (most community ports target Humble) |
| Algorithm is **untested** on Jazzy but works on Humble | **Humble** (don't be the first to debug Jazzy compat) |
| Algorithm is ROS 1 only (rare — some legacy forks) | **Noetic** (last resort) |

### Step 2: Verify sensor driver compatibility

| Sensor | ROS 2 Support | Notes |
|--------|---------------|-------|
| Ouster (ouster-ros) | Humble + Jazzy | Official support both |
| Livox (livox_ros_driver2) | Humble, Jazzy (from source) | Humble has binary |
| RealSense (realsense2_camera) | Humble + Jazzy | Official support both |
| Velodyne (velodyne_driver) | Humble + Jazzy | `ros-{distro}-velodyne` |
| ZED (zed-ros2-wrapper) | Humble + Jazzy | Official from Stereolabs |
| Hesai (HesaiLidar_ROS_2.0) | Humble | Jazzy untested |

If the sensor driver isn't available for a distro, that rules it out.

### Step 3: Narrow by Ubuntu version (tiebreaker)

Only after Steps 1-2 confirm multiple valid distros:

| Ubuntu | Best distro |
|--------|-------------|
| 24.04 (Noble) | **Jazzy** (only LTS option) |
| 22.04 (Jammy) | **Humble** (widest ecosystem) |
| JetPack 6.x (Orin) | **Humble** (JetPack 6 = Ubuntu 22.04) |
| JetPack 7+ (future) | **Jazzy** |
| 20.04 (Focal) | **Humble** via Docker, or Noetic native (but recommend upgrading OS) |

### When to recommend Noetic (ROS 1)

Only if ALL of these are true:
1. User already has a working ROS 1 workspace they need to integrate with
2. The SLAM algorithm they chose has mature ROS 1 support
3. They are NOT planning to add path planning (Nav2 is ROS 2 only)
4. They understand Noetic EOL is May 2025 (already past for new projects)

Otherwise, always steer toward ROS 2.

## Key Differences for SLAM Integration

### Package Availability

| Package | Humble | Jazzy | Notes |
|---------|--------|-------|-------|
| MAVROS | `ros-humble-mavros` | `ros-jazzy-mavros` | Full support both |
| Nav2 | `ros-humble-navigation2` | `ros-jazzy-navigation2` | Full support both |
| tf2 | `ros-humble-tf2-ros` | `ros-jazzy-tf2-ros` | Full support both |
| robot_state_publisher | Available | Available | Full support both |
| PCL | `ros-humble-pcl-ros` | `ros-jazzy-pcl-ros` | Full support both |
| Foxglove Bridge | Build from source | Build from source | Same process both |
| micro-ROS Agent | `ros-humble-micro-ros-agent` | `ros-jazzy-micro-ros-agent` | For DDS/PX4 |

### SLAM Algorithm Support

| Algorithm | Humble | Jazzy | Notes |
|-----------|--------|-------|-------|
| FAST-LIO2 | Yes (tested) | Yes (builds clean) | Same CMake/colcon process |
| FAST-LIO2 GPU | Yes (tested) | Untested | CUDA dependency, not distro-dependent |
| LIO-SAM | Yes (tested) | Yes | GTSAM available on both |
| COIN-LIO | Partial | Untested | May need C++17 flag adjustments |
| Cartographer | Yes (binary) | Yes (binary) | `ros-{distro}-cartographer-ros` |
| ORB-SLAM3 | Community port | Untested | Depends on OpenCV/Pangolin, not ROS distro |
| OpenVINS | Yes | Yes | Pure CMake, distro-independent |
| VINS-Fusion | Community port | Untested | cv_bridge API may differ |

### API/Build Differences

| Feature | Humble | Jazzy |
|---------|--------|-------|
| C++ standard | C++17 | C++17 (C++20 allowed) |
| CMake minimum | 3.8 | 3.16 |
| Default middleware | CycloneDDS | CycloneDDS |
| Launch file format | Python / XML | Python / XML |
| QoS defaults | Same | Same |
| colcon | Same workflow | Same workflow |

**In practice**: Most SLAM packages build identically on Humble and Jazzy. The main risk is third-party dependencies (Pangolin, GTSAM, OpenCV) having version conflicts on Ubuntu 24.04.

### Ouster Driver Compatibility

| Driver Version | Humble | Jazzy |
|---------------|--------|-------|
| ouster-ros v0.13.x | Yes | Yes (builds from source) |
| ouster-ros v0.14.x | Yes | Yes |

### Livox Driver Compatibility

| Driver | Humble | Jazzy |
|--------|--------|-------|
| livox_ros_driver2 | Yes | Builds from source |

## Migration Notes

### Foxy -> Humble
- Ubuntu 20.04 -> 22.04 (full OS upgrade required)
- `launch.py` API mostly compatible
- QoS: default changed from SYSTEM_DEFAULT to KEEP_LAST(10) in some packages
- Action API: minor signature changes

### Humble -> Jazzy
- Ubuntu 22.04 -> 24.04
- Minimal API changes for SLAM use cases
- Python 3.10 -> 3.12 (check any custom Python nodes)
- NumPy 1.x -> 2.x on Ubuntu 24.04 (breaking changes in array API)

## When This Matters

Most of the slam-agent workflow (URDF, config generation, ArduPilot params, extrinsics) is **distro-independent**. The distro matters for:
1. `apt install ros-{distro}-*` package names
2. Source `/opt/ros/{distro}/setup.bash` path
3. Third-party SLAM algorithm build compatibility
4. Python version differences in custom nodes
