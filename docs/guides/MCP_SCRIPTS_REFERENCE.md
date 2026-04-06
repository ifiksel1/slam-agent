# SLAM Agent - Available Scripts Reference

**MCP Server**: `slam-tools` (running on Python 3.10)
**Status**: ✅ Ready to run scripts
**Total Scripts**: 18 (8 installation + 8 diagnostic + 2 utility)

---

## 🛠️ Installation Scripts (Phase 4)

These scripts automate SLAM system installation on new hardware.

### 1. `install_slam_integration`
**Purpose**: Complete SLAM integration installation orchestrator
**Usage**: `run_install_script("install_slam_integration", "[config_file.yaml]")`
**When to Use**: First-time setup or major reinstallation
**What it does**:
- Creates workspace
- Clones all packages
- Builds workspace
- Generates config files
- Sets up environment

**Example**:
```bash
run_install_script("install_slam_integration", "~/my_slam_config.yaml")
```

---

### 2. `install_core_ros_packages`
**Purpose**: Install base ROS packages and dependencies
**Usage**: `run_install_script("install_core_ros_packages", "<ROS_VERSION> <ROS_DISTRO>")`
**Parameters**:
- `ROS_VERSION`: "ROS1" or "ROS2"
- `ROS_DISTRO`: "noetic", "humble", "foxy", etc.

**What it does**:
- Installs ROS core packages
- Sets up package manager
- Configures package sources
- Installs build tools (catkin, colcon, etc.)

**Example**:
```bash
run_install_script("install_core_ros_packages", "ROS1 noetic")
```

---

### 3. `install_lidar_driver`
**Purpose**: Install LiDAR sensor driver
**Usage**: `run_install_script("install_lidar_driver", "<LIDAR_TYPE> <WORKSPACE_PATH> <ROS_VERSION>")`
**Parameters**:
- `LIDAR_TYPE`: "ouster", "livox", "velodyne", "sick", etc.
- `WORKSPACE_PATH`: Path to catkin workspace (e.g., `~/slam_ws`)
- `ROS_VERSION`: "ROS1" or "ROS2"

**What it does**:
- Clones LiDAR driver package
- Installs SDK dependencies
- Builds driver
- Configures launch files
- Tests driver initialization

**Example**:
```bash
run_install_script("install_lidar_driver", "ouster ~/slam_ws ROS1")
```

---

### 4. `install_camera_driver`
**Purpose**: Install camera/vision sensor driver
**Usage**: `run_install_script("install_camera_driver", "<CAMERA_TYPE> <WORKSPACE_PATH> <ROS_VERSION>")`
**Parameters**:
- `CAMERA_TYPE`: "realsense", "zed", "flir", etc.
- `WORKSPACE_PATH`: Path to catkin workspace
- `ROS_VERSION`: "ROS1" or "ROS2"

**What it does**:
- Clones camera driver package
- Installs camera SDK
- Builds driver
- Sets up calibration tools
- Tests camera connection

**Example**:
```bash
run_install_script("install_camera_driver", "realsense ~/slam_ws ROS1")
```

---

### 5. `install_slam_algorithm`
**Purpose**: Install SLAM algorithm package
**Usage**: `run_install_script("install_slam_algorithm", "<SLAM_ALGORITHM> <WORKSPACE_PATH> <ROS_VERSION>")`
**Parameters**:
- `SLAM_ALGORITHM`: "fast_lio2", "lio_sam", "orb_slam3", "vins_mono", etc.
- `WORKSPACE_PATH`: Path to catkin workspace
- `ROS_VERSION`: "ROS1" or "ROS2"

**What it does**:
- Clones SLAM repository
- Installs algorithm dependencies (Eigen, Ceres, PCL, etc.)
- Builds SLAM package
- Configures launch files
- Generates example configs

**Example**:
```bash
run_install_script("install_slam_algorithm", "fast_lio2 ~/slam_ws ROS1")
```

---

### 6. `install_mavros`
**Purpose**: Install MAVROS (MAVLink-to-ROS bridge)
**Usage**: `run_install_script("install_mavros", "<ROS_VERSION> <ROS_DISTRO>")`
**Parameters**:
- `ROS_VERSION`: "ROS1" or "ROS2"
- `ROS_DISTRO`: "noetic", "humble", etc.

**What it does**:
- Installs MAVROS package via apt
- Downloads GeographicLib datasets
- Configures MAVLink dialects
- Sets up launch templates
- Tests flight controller communication

**Example**:
```bash
run_install_script("install_mavros", "ROS1 noetic")
```

---

### 7. `install_vision_to_mavros`
**Purpose**: Install vision-to-autopilot bridge (coordinate frame converter)
**Usage**: `run_install_script("install_vision_to_mavros", "<WORKSPACE_PATH> <ROS_VERSION>")`
**Parameters**:
- `WORKSPACE_PATH`: Path to catkin workspace
- `ROS_VERSION`: "ROS1" or "ROS2"

**What it does**:
- Clones vision_to_mavros package
- Compiles ENU↔NED converter
- Sets up Python bridge nodes
- Configures launch files
- Tests coordinate transformation

**Example**:
```bash
run_install_script("install_vision_to_mavros", "~/slam_ws ROS1")
```

---

### 8. `install_dds_bridge`
**Purpose**: Install DDS bridge (for ROS2 ↔ PX4 communication)
**Usage**: `run_install_script("install_dds_bridge", "<ROS_VERSION> <ROS_DISTRO> <FLIGHT_CONTROLLER> <WORKSPACE_PATH>")`
**Parameters**:
- `ROS_VERSION`: "ROS1" or "ROS2"
- `ROS_DISTRO`: "humble", "foxy", etc.
- `FLIGHT_CONTROLLER`: "px4" or "ardupilot"
- `WORKSPACE_PATH`: Path to workspace

**What it does**:
- Installs micro-ROS Agent
- Configures DDS middleware
- Sets up uORB to ROS bridges
- Tests PX4 communication

**Example**:
```bash
run_install_script("install_dds_bridge", "ROS2 humble px4 ~/slam_ws")
```

---

## 🔍 Diagnostic Scripts (Phase 5)

These scripts verify system health and validate integration.

### 1. `verify_installation`
**Purpose**: Comprehensive installation verification
**Usage**: `run_diagnostic("verify_installation", "<ROS_VERSION> <ROS_DISTRO> <WORKSPACE_PATH>")`
**What it checks**:
- ✅ ROS installation and setup.bash sourcing
- ✅ Workspace structure and sourcing
- ✅ All required packages built and loadable
- ✅ Python dependencies available
- ✅ System libraries present
- ✅ Launch files are valid
- ✅ Configuration files exist

**Output**: Pass/fail summary with errors highlighted

**Example**:
```bash
run_diagnostic("verify_installation", "ROS1 noetic ~/slam_ws")
```

---

### 2. `slam_diagnostics`
**Purpose**: Complete SLAM system health check
**Usage**: `run_diagnostic("slam_diagnostics")`
**What it checks**:
- ✅ LiDAR driver running and publishing
- ✅ SLAM algorithm running
- ✅ Odometry topic publishing at correct rate
- ✅ Vision bridge forwarding data
- ✅ MAVROS connected to autopilot
- ✅ MAVLink streams enabled
- ✅ EKF receiving vision poses
- ✅ Local position output publishing
- ✅ IMU-LiDAR time synchronization
- ✅ TF tree valid

**Output**: Status of each component with diagnostics

**Example**:
```bash
run_diagnostic("slam_diagnostics")
```

---

### 3. `check_topic_pipeline`
**Purpose**: Monitor data flow through ROS topic pipeline
**Usage**: `run_diagnostic("check_topic_pipeline", "[--sensor TOPIC --slam TOPIC --vision TOPIC --mavros TOPIC --duration SEC]")`
**Parameters**:
- `--sensor`: Input sensor topic (default: `/ouster/points`)
- `--slam`: SLAM odometry topic (default: `/fast_lio/odom`)
- `--vision`: Vision pose topic (default: `/mavros/vision_pose/pose`)
- `--mavros`: Output position topic (default: `/mavros/local_position/pose`)
- `--duration`: Test duration in seconds (default: 30)

**What it does**:
- Verifies each topic is publishing
- Measures publishing rates (Hz)
- Checks data freshness
- Validates data continuity
- Identifies bottlenecks or drops

**Output**: Topic rates, data freshness, gaps

**Examples**:
```bash
# Quick check (30 seconds, default topics)
run_diagnostic("check_topic_pipeline")

# Custom topics and longer duration
run_diagnostic("check_topic_pipeline", "--sensor /livox/points --slam /lio_sam/mapping/odometry --duration 60")
```

---

### 4. `check_tf_tree`
**Purpose**: Validate TF (Transform) tree structure
**Usage**: `run_diagnostic("check_tf_tree", "[--frames frame1 frame2 --source SRC --target TGT --verbose --visualize]")`
**Parameters**:
- `--frames`: Frames to validate (default: map, odom, base_link)
- `--source`: Source frame for transformation
- `--target`: Target frame for transformation
- `--verbose`: Print all transforms and timings
- `--visualize`: Generate URDF visualization

**What it does**:
- Checks all required frames exist
- Verifies transform chain is valid
- Checks transform freshness
- Tests transforms are invertible
- Validates frame hierarchy

**Output**: TF tree diagram, transform chains, timings

**Examples**:
```bash
# Quick validation
run_diagnostic("check_tf_tree")

# Verbose with chain check
run_diagnostic("check_tf_tree", "--source map --target base_link --verbose")

# Custom frame set
run_diagnostic("check_tf_tree", "--frames map odom base_link camera")
```

---

### 5. `check_autopilot_params`
**Purpose**: Verify flight controller parameters and configuration
**Usage**: `run_diagnostic("check_autopilot_params", "[--autopilot ardupilot|px4 --generate --baseline FILE]")`
**Parameters**:
- `--autopilot`: Flight controller type ("ardupilot" or "px4")
- `--generate`: Generate recommended parameters
- `--baseline`: Compare against baseline parameter file

**What it does**:
- ✅ Connects to flight controller
- ✅ Reads all parameters
- ✅ Validates safety parameters set
- ✅ Checks EKF configuration
- ✅ Verifies vision source settings
- ✅ Checks failsafes configured
- ✅ Compares against known-good baseline

**Output**: Parameter status, recommendations, mismatches

**Examples**:
```bash
# Check ArduPilot configuration
run_diagnostic("check_autopilot_params", "--autopilot ardupilot")

# Generate recommended params for GPS-denied flight
run_diagnostic("check_autopilot_params", "--autopilot ardupilot --generate")

# Compare against known-good baseline
run_diagnostic("check_autopilot_params", "--baseline ~/slam_ws/config/ardupilot_params.parm")
```

---

### 6. `check_sensor_time_sync`
**Purpose**: Verify sensor timestamp synchronization
**Usage**: `run_diagnostic("check_sensor_time_sync", "<topic1> <topic2> [--duration SEC --log FILE]")`
**Parameters**:
- `topic1`: First sensor topic
- `topic2`: Second sensor topic to compare
- `--duration`: Test duration (default: 30s)
- `--log`: Save analysis to file

**What it does**:
- Compares timestamps of two topics
- Detects time drift or jitter
- Identifies sync issues
- Calculates latency
- Recommends corrections

**Output**: Sync statistics, jitter, recommendations

**Examples**:
```bash
# Check LiDAR-IMU sync (built-in)
run_diagnostic("check_sensor_time_sync", "/ouster/points /ouster/imu")

# Check LiDAR-Camera sync (external IMU)
run_diagnostic("check_sensor_time_sync", "/ouster/points /camera/rgb/image_raw --duration 60")

# Save detailed log
run_diagnostic("check_sensor_time_sync", "/ouster/points /ouster/imu --log ~/sync_analysis.txt")
```

---

### 7. `check_urdf`
**Purpose**: Validate URDF (robot description) and TF tree
**Usage**: `run_diagnostic("check_urdf", "[urdf_file] [--from-param --required-frames f1 f2 --verbose --visualize]")`
**Parameters**:
- `urdf_file`: Path to URDF file (optional, loads from param if not specified)
- `--from-param`: Load from ROS parameter `/robot_description`
- `--required-frames`: Frames that must exist (default: base_link, map, odom)
- `--verbose`: Print full URDF tree
- `--visualize`: Generate visualization

**What it does**:
- Parses URDF XML
- Validates joint definitions
- Checks frame references
- Verifies transform chains
- Tests URDF parsing

**Output**: URDF structure, frame hierarchy, visualization

**Examples**:
```bash
# Check URDF from file
run_diagnostic("check_urdf", "~/slam_ws/src/orin_slam_integration/urdf/drone.urdf")

# Check URDF from ROS parameter
run_diagnostic("check_urdf", "--from-param --verbose")

# Verify required frames
run_diagnostic("check_urdf", "--required-frames base_link os1_sensor imu_link")

# Generate visualization
run_diagnostic("check_urdf", "--visualize")
```

---

### 8. `analyze_slam_bag`
**Purpose**: Analyze recorded rosbag SLAM data
**Usage**: `run_diagnostic("analyze_slam_bag", "<bag_file> [--slam-topic TOPIC --start SEC --end SEC --plot --report]")`
**Parameters**:
- `bag_file`: Path to recorded rosbag file
- `--slam-topic`: SLAM odometry topic to analyze
- `--start`: Start time in seconds (offset from bag start)
- `--end`: End time in seconds
- `--plot`: Generate plots (trajectory, error, etc.)
- `--report`: Generate PDF report

**What it does**:
- Reads rosbag SLAM odometry
- Calculates drift and error
- Analyzes trajectory
- Detects loop closure opportunities
- Generates statistics
- Optional plots and report generation

**Output**: Statistics, plots, trajectory analysis, report

**Examples**:
```bash
# Basic analysis
run_diagnostic("analyze_slam_bag", "~/slam_bags/test_flight_1.bag")

# Custom topic and time range
run_diagnostic("analyze_slam_bag", "~/test.bag --slam-topic /lio_sam/mapping/odometry --start 10 --end 60")

# Generate plots and report
run_diagnostic("analyze_slam_bag", "~/flight_log.bag --plot --report")
```

---

## 📊 Profile Management Tools

### `search_profiles`
**Purpose**: Find matching hardware profiles
**Usage**: `search_profiles(platform="", sensor="", algorithm="", autopilot="")`

**Examples**:
```bash
# Find all Jetson Orin systems
search_profiles(platform="jetson_orin")

# Find Ouster + FAST-LIO2 combinations
search_profiles(sensor="ouster", algorithm="fast_lio2")

# Find ArduPilot systems
search_profiles(autopilot="ardupilot")

# Combination search
search_profiles(platform="jetson_orin", sensor="ouster_os1_64", algorithm="fast_lio2")
```

---

### `get_profile`
**Purpose**: Retrieve complete hardware profile
**Usage**: `get_profile(fingerprint)`

**Example**:
```bash
# Get your system's profile
get_profile("jetson_orin_nx-ouster_os1_64-fast_lio2-ardupilot-noetic")
```

---

### `get_known_good_config`
**Purpose**: Get copy-paste-ready configuration files
**Usage**: `get_known_good_config(fingerprint)`

**Example**:
```bash
# Get all config files for your hardware
get_known_good_config("jetson_orin_nx-ouster_os1_64-fast_lio2-ardupilot-noetic")
```

---

## 💾 Learning Tools

### `save_hardware_profile`
**Purpose**: Document a new hardware configuration
**Usage**: `save_hardware_profile(fingerprint, hardware_yaml, phase1_config_yaml)`

---

### `update_profile_status`
**Purpose**: Mark profile as validated or integration_complete
**Usage**: `update_profile_status(fingerprint, validated=True/False, integration_complete=True/False)`

---

### `save_solution`
**Purpose**: Document a troubleshooting solution
**Usage**: `save_solution(phase, hardware_summary, symptom, root_cause, fix, files_changed, tags)`

---

## 📋 Usage Examples

### Example 1: Verify Complete Installation
```bash
# Step 1: Verify ROS and workspace
run_diagnostic("verify_installation", "ROS1 noetic ~/slam_ws")

# Step 2: Check SLAM diagnostics
run_diagnostic("slam_diagnostics")

# Step 3: Monitor data pipeline
run_diagnostic("check_topic_pipeline", "--duration 60")

# Step 4: Validate TF tree
run_diagnostic("check_tf_tree", "--verbose")

# Step 5: Check autopilot configuration
run_diagnostic("check_autopilot_params", "--autopilot ardupilot")
```

### Example 2: Fresh Installation
```bash
# Step 1: Install ROS
run_install_script("install_core_ros_packages", "ROS1 noetic")

# Step 2: Install LiDAR driver
run_install_script("install_lidar_driver", "ouster ~/slam_ws ROS1")

# Step 3: Install SLAM algorithm
run_install_script("install_slam_algorithm", "fast_lio2 ~/slam_ws ROS1")

# Step 4: Install MAVROS
run_install_script("install_mavros", "ROS1 noetic")

# Step 5: Install vision bridge
run_install_script("install_vision_to_mavros", "~/slam_ws ROS1")

# Step 6: Verify everything
run_diagnostic("verify_installation", "ROS1 noetic ~/slam_ws")
```

### Example 3: Troubleshooting Vision Fusion
```bash
# Step 1: Check topic pipeline
run_diagnostic("check_topic_pipeline")

# Step 2: Check TF tree
run_diagnostic("check_tf_tree", "--verbose")

# Step 3: Check autopilot params
run_diagnostic("check_autopilot_params", "--autopilot ardupilot")

# Step 4: Check sensor sync
run_diagnostic("check_sensor_time_sync", "/ouster/points /ouster/imu")

# Step 5: Run SLAM diagnostics
run_diagnostic("slam_diagnostics")
```

---

## 🚀 Script Timeout & Limitations

| Script Type | Timeout | Notes |
|------------|---------|-------|
| Installation | 10 minutes | Some packages take time to compile |
| Diagnostics | 5 minutes | Should complete quickly |
| Analysis | 5 minutes | Analyzes recorded data |
| Profile ops | 30 seconds | File I/O operations |

---

## 📞 Script Status

All 18 scripts are **ready to run** via the MCP server:

✅ 8 Installation scripts (Phase 4)
✅ 8 Diagnostic scripts (Phase 5)
✅ 2 Utility scripts (verification)
✅ Profile management tools
✅ Learning/knowledge capture tools

**MCP Server Status**: Running on Python 3.10
**Location**: `mcp/slam_mcp_server.py`
**Configuration**: `.mcp.json` (auto-configured)

---

## 💡 Tips

1. **Run diagnostics before installation** - Understand current state
2. **Use profiles to avoid re-tuning** - Copy known-good configs
3. **Save solutions after troubleshooting** - Help future integrations
4. **Monitor topics during development** - Use check_topic_pipeline
5. **Validate URDF early** - Prevents TF tree issues later

---

**Ready to run!** Use these scripts through the SLAM agent for automated installation, verification, and troubleshooting.
