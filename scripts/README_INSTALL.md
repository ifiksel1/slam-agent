# SLAM Integration Installation Scripts

Automated installation scripts for Phase 4: Workspace Setup & Package Installation.

## Quick Start

### Option 1: Interactive Installation (Recommended)

```bash
cd scripts
./install_slam_integration.sh
```

The script will prompt you for:
- ROS version (ROS1/ROS2 and distro)
- Workspace path
- Flight controller (ArduPilot/PX4)
- Sensor types (LiDAR, camera)
- SLAM algorithm

### Option 2: Configuration File

1. Copy the template:
```bash
cp scripts/install_config_template.yaml my_config.yaml
```

2. Edit `my_config.yaml` with your hardware details

3. Run installation:
```bash
./install_slam_integration.sh my_config.yaml
```

## What Gets Installed

The installation script automates:

1. **Workspace Creation**
   - Creates ROS workspace if it doesn't exist
   - Initializes catkin (ROS1) or colcon (ROS2)

2. **Core ROS Packages**
   - TF2, robot_state_publisher
   - RViz, rqt tools
   - PCL, cv_bridge, common libraries

3. **Flight Controller Bridge**
   - MAVROS (for ROS1 or ROS2+ArduPilot)
   - micro-ROS Agent/DDS (for ROS2+PX4 or ROS2+ArduPilot with DDS)

4. **Sensor Drivers**
   - LiDAR drivers (Ouster, Velodyne, Livox, Hesai)
   - Camera drivers (RealSense, ZED)

5. **SLAM Algorithm**
   - FAST-LIO2, LIO-SAM, LVI-SAM, OpenVINS, etc.

6. **Integration Bridge**
   - vision_to_mavros (for MAVROS setups)

7. **Workspace Build**
   - Builds all packages
   - Verifies installation

## Script Components

- `install_slam_integration.sh` - Main orchestrator script
- `install_core_ros_packages.sh` - Core ROS packages
- `install_mavros.sh` - MAVROS installation
- `install_dds_bridge.sh` - micro-ROS Agent/DDS installation
- `install_lidar_driver.sh` - LiDAR driver installation
- `install_camera_driver.sh` - Camera driver installation
- `install_slam_algorithm.sh` - SLAM algorithm installation
- `install_vision_to_mavros.sh` - vision_to_mavros bridge
- `verify_installation.sh` - Post-installation verification

## Features

✅ **Smart Checking**: Skips already-installed packages  
✅ **Error Handling**: Stops on errors, provides clear messages  
✅ **Progress Logging**: Color-coded output for easy reading  
✅ **Resumable**: Can be run multiple times safely  
✅ **Configurable**: Supports both interactive and config file modes  

## Example Output

```
[INFO] Configuration:
[INFO]   ROS Version: ROS1 (noetic)
[INFO]   Flight Controller: ArduPilot
[INFO]   Use DDS: false
[INFO]   Workspace: ~/catkin_ws

[INFO] Step 1: Checking/Creating ROS workspace...
[✓] Workspace already exists: ~/catkin_ws

[INFO] Step 2: Installing core ROS packages...
[✓] Build tools already installed
[✓] All core ROS packages already installed

[INFO] Step 3: Installing flight controller bridge...
[✓] MAVROS already installed (version: 1.16.0)
[✓] GeographicLib datasets installed

[INFO] Step 4: Installing sensor drivers...
[INFO] Installing Ouster driver...
[✓] Ouster driver cloned

[INFO] Step 5: Installing SLAM algorithm...
[INFO] Installing FAST-LIO2...
[✓] FAST-LIO cloned

[INFO] Step 6: Installing vision_to_mavros bridge...
[✓] vision_to_mavros cloned

[INFO] Step 7: Building workspace...
[✓] Workspace built successfully

[INFO] Step 8: Verifying installation...
[✓] Workspace exists: ~/catkin_ws
[✓] tf2_ros installed
[✓] robot_state_publisher installed
[✓] MAVROS installed
[✓] Verification complete! All critical components installed.
```

## Troubleshooting

### Script Fails on Dependency Installation

Some packages may require manual intervention. The script will continue and warn you. Check the specific package's README for additional dependencies.

### SLAM Algorithm Build Fails

Some SLAM algorithms have specific requirements:
- Check: https://github.com/engcang/SLAM-application for installation guides
- Verify you have all dependencies (Ceres, PCL, Eigen, etc.)
- Some algorithms may need GPU support (CUDA)

### Permission Errors

If you get permission errors:
```bash
sudo chmod +x scripts/*.sh
```

### Workspace Already Exists

The script will detect and use existing workspaces. If you want a fresh workspace, specify a different path in the config.

## Integration with AI Guide

These scripts are designed to work with the AI-guided installation process:

1. **Phase 1**: User answers questions about hardware
2. **Phase 2**: Validation and summary
3. **Phase 3**: File generation (configs, launch files, etc.)
4. **Phase 4**: **Run these installation scripts** (or follow manual guide)
5. **Phase 5**: Testing and validation

The AI assistant can:
- Generate a config file from Phase 1 answers
- Run the installation script automatically
- Verify installation and continue to Phase 5

## Manual Installation

If you prefer manual installation or the scripts don't work for your setup, see:
- `docs/AI_SYSTEM_BUILDER_GUIDE.md` Phase 4 (lines 3455-5057)
- Step-by-step manual installation instructions

## Next Steps After Installation

1. **Configure LiDAR Network** (if using Ethernet LiDAR):
   ```bash
   # See Phase 4, Step 7 in the guide
   ```

2. **Run Diagnostics**:
   ```bash
   ./slam_diagnostics.sh
   ```

3. **Follow Testing Protocol**:
   - Bench test (no props)
   - Ground test (tethered)
   - Flight test (GPS available)
   - GPS-denied test (indoor)

## Contributing

To add support for new sensors or SLAM algorithms, edit the respective install script:
- `install_lidar_driver.sh` - Add new LiDAR types
- `install_camera_driver.sh` - Add new camera types
- `install_slam_algorithm.sh` - Add new SLAM algorithms

Follow the existing pattern and test thoroughly before submitting.
