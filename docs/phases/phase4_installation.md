# Phase 4: Workspace Setup & Installation

## Input
- Validated config from Phase 2
- Generated files from Phase 3
- `scripts/install_config.yaml`

## Installation Options

Offer user three choices:
- **Option A (Recommended)**: AI runs `scripts/install_slam_integration.sh install_config.yaml` automatically
- **Option B**: User runs the script manually
- **Option C**: Manual step-by-step (fallback if scripts fail)

## Pre-Installation Check

Before installing anything, check what already exists:
```bash
# Workspace
ls ~/catkin_ws/devel/setup.bash 2>/dev/null && echo "workspace exists"
# ROS packages
rospack find mavros 2>/dev/null && echo "mavros installed"
rospack find tf2_ros 2>/dev/null && echo "tf2 installed"
# Git repos
ls ~/catkin_ws/src/ouster-ros 2>/dev/null && echo "ouster driver exists"
```

Only install what's missing. Skip with "Already installed: [package]".

## Installation Steps (Manual Fallback)

### Step 1: Create ROS Workspace
```bash
# ROS1
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
catkin init  # or catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# ROS2
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: Core ROS Packages
```bash
sudo apt install ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-rviz ros-$ROS_DISTRO-rqt
```

### Step 3: MAVROS or DDS

**MAVROS** (ROS1, or ROS2+ArduPilot):
```bash
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
```

**DDS/micro-ROS** (ROS2+PX4 or ROS2+ArduPilot optional):
```bash
sudo apt install ros-$ROS_DISTRO-micro-ros-agent
# Or build from source:
cd ~/ros2_ws/src
git clone https://github.com/micro-ROS/micro_ros_setup.git
cd ~/ros2_ws && colcon build --packages-select micro_ros_setup
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### Step 4: Sensor Drivers

**Ouster**: `git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git`
**Velodyne**: `sudo apt install ros-$ROS_DISTRO-velodyne`
**Livox**: Install Livox SDK first, then `git clone https://github.com/Livox-SDK/livox_ros_driver.git`
**RealSense**: `sudo apt install ros-$ROS_DISTRO-realsense2-camera`
**ZED**: Follow Stereolabs installation guide
**Hesai**: `git clone https://github.com/HesaiTechnology/HesaiLidar_General_ROS.git`

### Step 5: SLAM Algorithm

Reference: https://github.com/engcang/SLAM-application for tested installation guides.

**FAST-LIO2**:
```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO && git submodule update --init
cd ~/catkin_ws && catkin build fast_lio
```

**LIO-SAM** (needs GTSAM):
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update && sudo apt install libgtsam-dev libgtsam-unstable-dev
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LIO-SAM.git
cd ~/catkin_ws && catkin build lio_sam
```

**OpenVINS**:
```bash
cd ~/catkin_ws/src
git clone https://github.com/rpng/open_vins.git
cd ~/catkin_ws && catkin build
```

### Step 6: Vision Bridge
Install vision_to_mavros or create DDS publisher from Phase 3 generated code.

### Step 7: LiDAR Network (Ethernet only)
```yaml
# /etc/netplan/01-lidar-network.yaml
network:
  version: 2
  ethernets:
    INTERFACE:
      addresses: [COMPUTER_IP/24]
      dhcp4: no
```
```bash
sudo netplan apply
ping LIDAR_IP
```

### Step 8: Copy Integration Package
Copy all Phase 3 generated files to workspace.

### Step 9: Build
```bash
cd ~/catkin_ws && catkin build
source devel/setup.bash
```

### Step 10: Verify
```bash
rospack find PACKAGE_NAME
roslaunch PACKAGE_NAME master.launch --screen
```

## Output
- All packages installed
- Workspace builds successfully
- Update progress YAML with installation_status
