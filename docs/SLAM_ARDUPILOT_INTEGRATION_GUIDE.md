# SLAM to ArduPilot Integration Guide

**Version**: 1.0  
**Date**: November 22, 2025  
**System**: Ultra-onboard ROS1 Noetic  
**Platform**: NVIDIA Jetson Orin NX

---

## Overview

This document explains how LIO-SAM integrates with ArduPilot in the Ultra-onboard system to enable GPS-denied autonomous navigation, and provides a structured template to repeat this process with any SLAM algorithm (FAST-LIO, COIN-LIO, Cartographer, RTAB-Map, etc.).

**Scope**: This guide covers the complete integration pipeline from LiDAR/camera sensors through SLAM algorithms to ArduPilot's EKF3, including TF tree management, coordinate transformations, and parameter configuration.

---

## Table of Contents

1. [How LIO-SAM Integrates with ArduPilot](#part-1-how-lio-sam-integrates-with-ardupilot)
2. [Generic SLAM Integration Template](#part-2-generic-slam-integration-template)
3. [URDF Guidance](#part-3-urdf-guidance)
4. [Algorithm-Specific Notes](#part-4-algorithm-specific-notes)
5. [Troubleshooting](#part-5-troubleshooting)
6. [Example Configurations](#part-6-example-configurations)

---

## Part 1: How LIO-SAM Integrates with ArduPilot

### 1.1 Architecture Overview

The GPS-denied navigation pipeline consists of 6 key components:

```
Ouster LiDAR → LIO-SAM → TF Tree → vision_to_mavros → MAVROS → ArduPilot EKF3
     ↓                                                              ↑
  IMU (from ArduPilot via MAVROS) ←──────────────────────────────┘
```

**Data Flow**:

1. **Sensor Input**: Ouster LiDAR publishes point clouds (`/ouster/points_aligned`) at 10-20 Hz
2. **IMU Integration**: LIO-SAM subscribes to `/mavros/imu/data` from ArduPilot's onboard IMU
3. **SLAM Processing**: LIO-SAM fuses LiDAR + IMU to produce odometry (`/lio_sam/mapping/odometry`)
4. **TF Publishing**: LIO-SAM publishes `map→odom→base_link` transform tree
5. **Coordinate Conversion**: `vision_to_mavros` node converts transforms to MAVROS format
6. **Pose Feedback**: MAVROS sends vision pose to ArduPilot at `/mavros/vision_pose/pose` (30-40 Hz)
7. **EKF Fusion**: ArduPilot EKF3 fuses vision pose with IMU for state estimation

### 1.2 Key Files and Their Roles

**LIO-SAM Configuration** (`lio_sam_onb/config/params.yaml`):

- IMU topic: `/mavros/imu/data` (uses ArduPilot's IMU, not LiDAR's)
- Point cloud: `/ouster/points_aligned`
- Frames: `lidarFrame: base_link`, `mapFrame: map`, `odometryFrame: odom`
- IMU noise parameters calibrated for ARK V6X flight controller
- Extrinsics: LiDAR-to-IMU transform (identity if co-located)

**Launch Integration** (`onboard_main_bringup/dragunfly_bringup_onb/launch/onboard_main.launch`):

- Line 169: Launches LIO-SAM with `run_no_rviz.launch`
- Lines 217-223: Launches `vision_to_mavros` node (TF → PoseStamped converter)
- Line 188: Launches `set_origin2.py` to initialize ArduPilot's EKF origin

**TF Bridge** (`onboard_utilities/vision_to_mavros_onb/src/vision_to_mavros.cpp`):

- Looks up `map→base_link` transform from TF tree (or subscribes to odometry topic)
- Converts to `geometry_msgs/PoseStamped` at 30 Hz
- Publishes to `/mavros/vision_pose/pose`
- Handles coordinate frame rotations (ENU ↔ NED conversions via `gamma_world` parameter)

**EKF Origin Setup** (`onboard_utilities/vision_to_mavros_onb/scripts/set_origin2.py`):

- Sets global GPS origin for local coordinate system
- Sends `SET_GPS_GLOBAL_ORIGIN` MAVLink message
- Resets EKF with `MAV_CMD_DO_SET_HOME` command (param1=1)
- Critical for indoor/GPS-denied flight initialization

### 1.3 ArduPilot EKF3 Configuration

**Key Parameters** (from `onboard_flight_ops/loc_nav/ego_swarm_onb/configs/ardupilot_ego_swarm.parm`):

```
# GPS mode (default - outdoor with GPS)
EK3_SRC1_POSXY = 3    # GPS for XY position
EK3_SRC1_VELXY = 3    # GPS for XY velocity
EK3_SRC1_POSZ = 1     # Barometer for altitude
EK3_SRC1_VELZ = 3     # GPS for vertical velocity

# GPS-denied mode (indoor - uncomment for LIO-SAM)
# EK3_SRC1_POSXY = 6  # External nav (vision) for XY position
# EK3_SRC1_VELXY = 6  # External nav for XY velocity
# EK3_SRC1_POSZ = 1   # Barometer for altitude (or 6 for vision Z)
# EK3_SRC1_VELZ = 6   # External nav for vertical velocity

# Required settings
AHRS_EKF_TYPE = 3     # Use EKF3
VISO_TYPE = 2         # MAVLink vision pose
ARMING_CHECK = 388598 # Disable GPS checks for indoor
```

**Switching to GPS-denied mode**:

```bash
rosrun mavros mavparam set EK3_SRC1_POSXY 6
rosrun mavros mavparam set EK3_SRC1_VELXY 6
rosrun mavros mavparam set ARMING_CHECK 388598
rosservice call /mavros/cmd/reboot "{}"
```

### 1.4 URDF Role

The URDF (`onboard_utilities/dragunfly_urdf_onb/urdf/dragunfly_tf.urdf`) defines:

1. **TF Tree Structure**: `base_link → os1_sensor`, `base_link → fcu`, etc.
2. **Sensor Mounts**: Physical offsets (LiDAR at -6cm below base_link, rotated π)
3. **Robot State Publisher**: Publishes static transforms for all sensors

**Launch** (`onboard_utilities/dragunfly_urdf_onb/launch/dragunfly_publish.launch`):

```xml
<param name="robot_description" textfile="dragunfly_tf.urdf" />
<node name="dragunfly_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

**Critical Transforms**:

- `base_link → os1_sensor`: `xyz="0 0 -0.060"` `rpy="π 0 0"` (LiDAR flipped upside-down)
- `base_link → fcu`: Flight controller location (EKF reference frame)

**Why It Matters**: LIO-SAM publishes `map→base_link`. The URDF provides `base_link→sensors`. Together they give `map→all_sensors`, enabling proper sensor fusion.

---

## Part 2: Generic SLAM Integration Template

### 2.1 Prerequisites Checklist

Before integrating a new SLAM algorithm:

- [ ] SLAM outputs odometry as `nav_msgs/Odometry` or publishes TF (`map→odom→base_link`)
- [ ] SLAM can accept IMU data from `/mavros/imu/data`
- [ ] LiDAR driver publishes point clouds in correct frame
- [ ] Robot has URDF or static TF publishers configured
- [ ] MAVROS connected to ArduPilot (`rostopic echo /mavros/state`)
- [ ] ArduPilot parameters backed up (`rosrun mavros mavparam dump backup.parm`)

### 2.2 Integration Steps

#### Step 1: Configure SLAM Algorithm

**Config File** (`config/slam_params.yaml`):

```yaml
# Topics (ROS1) or topic names (ROS2)
imu_topic: "/mavros/imu/data"              # Use ArduPilot's IMU
pointcloud_topic: "/ouster/points_aligned" # Or /velodyne_points, /camera/depth/points
odometry_topic: "/slam/odometry"           # SLAM output

# Frames
map_frame: "map"          # Global fixed frame
odom_frame: "odom"        # Local odometry frame
base_frame: "base_link"   # Robot body frame
lidar_frame: "os1_sensor" # LiDAR sensor frame (from URDF)

# IMU Parameters (calibrate for your FC - see Allan variance analysis)
imu_acc_noise: 9.83e-03   # m/s²/√Hz
imu_gyr_noise: 2.38e-03   # rad/s/√Hz
imu_acc_bias: 4.99e-04    # m/s²
imu_gyr_bias: 3.23e-05    # rad/s
imu_gravity: 9.8          # m/s²

# Extrinsics (LiDAR to IMU)
# If using URDF: leave as identity, TF handles it
# If no URDF: manually specify transform
extrinsic_T: [0.0, 0.0, 0.0]               # Translation [x, y, z]
extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]   # Rotation matrix (row-major)
```

**Key Tuning**:

- IMU noise parameters MUST match your flight controller (ARK V6X, Pixhawk, Cube, etc.)
- Run Allan variance analysis or use manufacturer specs
- See `lio_sam_onb/config/params.yaml` lines 50-64 for ARK V6X example

#### Step 1.5: Configure Ethernet LiDAR Network (if applicable)

**Skip this if**: Using USB LiDAR, camera-only SLAM, or LiDAR already configured

Most 3D LiDARs use Ethernet/UDP and require network configuration before the ROS driver will work.

**Quick Network Setup Guide**:

**1. Identify your LiDAR's default IP**:

| Manufacturer | Default Sensor IP | Default Destination | UDP Ports |
|--------------|-------------------|---------------------|-----------|
| Ouster | 192.168.1.201 | 192.168.1.1 (configurable) | 7502 (lidar), 7503 (IMU) |
| Velodyne | 192.168.1.201 | 192.168.1.255 (broadcast) | 2368 (data), 8308 (telemetry) |
| RoboSense | 192.168.1.200 | 192.168.1.102 | 6699 (MSOP), 7788 (DIFOP) |
| Hesai | 192.168.1.201 | 192.168.1.100 | 2368 (data) |
| Livox | 192.168.1.1xx (varies) | N/A (broadcasts) | 50000-51000 range |

**2. Configure your computer's network interface**:

```bash
# Find your network interface name
ip link show
# Look for: eth0, eth1, eno1, enp3s0, etc. (NOT wlan0/WiFi!)

# Configure static IP on Ubuntu (netplan)
sudo nano /etc/netplan/01-lidar-network.yaml
```

Add this configuration (adjust interface name and IPs to match your LiDAR):

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # Replace with your interface
      addresses:
        - 192.168.1.100/24  # Your computer's IP (same subnet as LiDAR)
      dhcp4: no
      optional: true  # Don't wait for this at boot
```

Apply the configuration:

```bash
sudo netplan apply

# Verify
ip addr show eth0
# Should show: inet 192.168.1.100/24
```

**3. Test connectivity**:

```bash
# Ping the LiDAR
ping 192.168.1.201 -c 5

# Expected output:
# 64 bytes from 192.168.1.201: icmp_seq=1 ttl=64 time=0.5 ms
```

**If ping fails**:
- Check physical connection (cable, power)
- Verify LiDAR is powered on and spinning
- Try different host IP in same subnet (e.g., 192.168.1.50)
- Check firewall: `sudo ufw allow from 192.168.1.0/24`

**4. Verify UDP data streaming**:

```bash
# For Ouster (port 7502):
sudo tcpdump -i eth0 udp port 7502 -c 10

# For Velodyne (port 2368):
sudo tcpdump -i eth0 udp port 2368 -c 10

# Should see packets like:
# 12:34:56.789 IP 192.168.1.201.7502 > 192.168.1.100.7502: UDP, length 1248
```

**If no UDP packets**:
- Firewall blocking: `sudo ufw allow from 192.168.1.0/24 to any`
- LiDAR sending to wrong IP (configure LiDAR, see step 5)
- LiDAR not transmitting (check power, initialization)

**5. Configure LiDAR destination IP** (if needed):

**Ouster** (web interface or curl):

```bash
# Web UI (recommended):
firefox http://192.168.1.201

# Or command line:
curl -X POST http://192.168.1.201/api/v1/system/network/ipv4/override \
     -H "Content-Type: application/json" \
     -d '{"udp_dest":"192.168.1.100"}'

# Set lidar mode (resolution):
curl -X POST http://192.168.1.201/api/v1/sensor/config \
     -H "Content-Type: application/json" \
     -d '{"lidar_mode":"1024x10"}'  # Options: 512x10, 1024x10, 2048x10

# Restart sensor to apply
curl -X POST http://192.168.1.201/api/v1/system/reboot
```

**Velodyne** (web interface):

```bash
firefox http://192.168.1.201
# Change "Host (Destination) IP" to your computer's IP (192.168.1.100)
# Save and power-cycle the sensor
```

**RoboSense** (RSView software or web interface):

Download RSView from manufacturer website, or use web UI at sensor IP.

**6. Test with ROS driver**:

```bash
# ROS1 Ouster example:
roslaunch ouster_ros sensor.launch \
  sensor_hostname:=192.168.1.201 \
  metadata:=/tmp/ouster_metadata.json

# Check for point cloud data:
rostopic hz /ouster/points
# Expected: ~10-20 Hz

# Check for IMU data:
rostopic hz /ouster/imu
# Expected: ~100 Hz

# ROS1 Velodyne example:
roslaunch velodyne_pointcloud VLP16_points.launch
rostopic hz /velodyne_points
```

**Docker Considerations**:

If running ROS in Docker, use host networking to access LiDAR:

```bash
docker run --network=host --privileged \
  -v /dev:/dev \
  your_ros_image
```

Or create a macvlan network for isolated access:

```bash
docker network create -d macvlan \
  --subnet=192.168.1.0/24 \
  -o parent=eth0 lidar_net

docker run --network=lidar_net ...
```

**Common Issues**:

| Issue | Cause | Solution |
|-------|-------|----------|
| Ping timeout | Wrong subnet/IP | Ensure both IPs in 192.168.1.x/24 |
| Ping OK, no UDP | Firewall | `sudo ufw allow from 192.168.1.0/24` |
| UDP packets seen, no ROS topic | Wrong ROS driver config | Check sensor_hostname parameter |
| Intermittent dropouts | Slow network | Check cable quality, switch, MTU |
| Works, then stops after reboot | IP config not persistent | Use netplan (permanent) not ifconfig (temporary) |

**Jetson-Specific Notes**:

On NVIDIA Jetson, the Ethernet interface may be `eth0` or `l4tbr0`:

```bash
# Check available interfaces
ip link show

# Common Jetson interfaces: eth0, eth1, l4tbr0

# May need to disable NetworkManager for static IP:
sudo systemctl disable NetworkManager
sudo systemctl stop NetworkManager
```

**Verification Checklist**:

- [ ] LiDAR IP reachable via ping
- [ ] UDP packets visible with tcpdump
- [ ] ROS driver launches without errors
- [ ] Point cloud topic publishing at expected rate (10-30 Hz)
- [ ] IMU topic publishing (if LiDAR has IMU)
- [ ] No dropped packets or latency warnings

**Once network is working**, proceed to SLAM integration.

---

#### Step 2: Set Up TF Tree

**Option A: Using URDF** (Recommended)

Your URDF must define:

```xml
<robot name="my_robot">
  <link name="base_link"/>
  <link name="lidar_sensor"/>
  <link name="imu_link"/>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <origin xyz="0 0 -0.06" rpy="3.14159 0 0"/>
  </joint>
  
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```

Launch robot state publisher:

```xml
<launch>
  <param name="robot_description" textfile="$(find my_robot_description)/urdf/robot.urdf" />
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>
</launch>
```

**Option B: Without URDF** (Manual Static Transforms)

If you don't have a URDF, publish static transforms manually:

```xml
<launch>
  <!-- base_link → lidar_sensor -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_lidar"
        args="0 0 -0.06 3.14159 0 0 base_link os1_sensor 100"/>
  
  <!-- base_link → IMU (if separate from FC) -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_imu"
        args="0 0 0 0 0 0 base_link imu_link 100"/>
  
  <!-- base_link → fcu (flight controller reference) -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_fcu"
        args="0 0 0 0 0 0 base_link fcu 100"/>
</launch>
```

**ROS2 Equivalent**:

```xml
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="0 0 -0.06 3.14159 0 0 base_link os1_sensor"/>
```

**Critical**: Measure physical sensor positions with calipers. Errors of even 1-2 cm can degrade SLAM accuracy.

#### Step 3: Create SLAM Launch File

```xml
<launch>
  <!-- Parameters -->
  <rosparam file="$(find my_slam)/config/slam_params.yaml" command="load"/>
  
  <!-- SLAM Node -->
  <node pkg="my_slam" type="slam_node" name="slam_node" output="screen">
    <remap from="~/imu" to="/mavros/imu/data"/>
    <remap from="~/pointcloud" to="/ouster/points_aligned"/>
    <remap from="~/odometry" to="/slam/odometry"/>
  </node>
</launch>
```

#### Step 4: Integrate vision_to_mavros Bridge

**Reuse Existing Node**:

```xml
<node pkg="vision_to_mavros" type="vision_to_mavros_node" name="slam_to_mavros" output="screen">
  <param name="target_frame_id" value="map" />
  <param name="source_frame_id" value="base_link" />
  <param name="output_rate" value="30" />        <!-- Hz -->
  <param name="gamma_world" value="0.0" />       <!-- Rotation offset (usually 0 for LiDAR) -->
  <param name="use_tf" value="true" />           <!-- Use TF tree -->
  <remap from="~/vision_pose" to="/mavros/vision_pose/pose" />
</node>
```

**Alternative: Subscribe to Odometry**:

If SLAM doesn't publish TF:

```xml
<node pkg="vision_to_mavros" type="vision_to_mavros_node" name="slam_to_mavros" output="screen">
  <param name="use_tf" value="false" />          <!-- Use odometry topic -->
  <remap from="~/input_odom" to="/slam/odometry"/>
  <remap from="~/vision_pose" to="/mavros/vision_pose/pose" />
</node>
```

#### Step 5: Set EKF Origin

**Launch set_origin script**:

```xml
<node pkg="vision_to_mavros" name="set_origin_node" type="set_origin2.py" output="screen"/>
```

**Manual Setup** (one-time):

```bash
# Set origin coordinates (lat/lon/alt in script)
rosrun vision_to_mavros set_origin2.py
```

This initializes ArduPilot's EKF to use the current position as (0,0,0) in local frame.

#### Step 6: Configure ArduPilot Parameters

**Create parameter file** (`my_slam_ardupilot.parm`):

```
# EKF3 Configuration for GPS-denied navigation
AHRS_EKF_TYPE,3              # Use EKF3
VISO_TYPE,2                  # MAVLink vision pose

# Use vision for position/velocity
EK3_SRC1_POSXY,6             # External nav for XY position
EK3_SRC1_VELXY,6             # External nav for XY velocity
EK3_SRC1_POSZ,1              # Barometer for altitude (or 6 for vision Z)
EK3_SRC1_VELZ,6              # External nav for vertical velocity
EK3_SRC1_YAW,1               # Compass for yaw (or 6 for vision)

# Disable GPS arming checks
ARMING_CHECK,388598          # Removes GPS, fence, RC checks (be careful!)

# Safety parameters
FENCE_ENABLE,1               # Enable geofence
FENCE_TYPE,7                 # Alt + circle + polygon
FENCE_ACTION,1               # RTL on breach
FENCE_RADIUS,50              # meters
FENCE_ALT_MAX,25             # meters (adjust per environment)

# Velocity limits (must match planner)
WPNAV_SPEED,200              # cm/s (2.0 m/s)
WPNAV_ACCEL,300              # cm/s² (3.0 m/s²)
PSC_POSXY_P,1.5              # Position controller P gain (tune for tracking)
```

**Load parameters**:

```bash
# Backup first!
rosrun mavros mavparam dump ~/backup_$(date +%Y%m%d).parm

# Load new parameters
rosrun mavros mavparam load my_slam_ardupilot.parm

# Verify critical params
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6
rosrun mavros mavparam get VISO_TYPE       # Should be 2

# Reboot FC
rosservice call /mavros/cmd/reboot "{}"
```

#### Step 7: Create Master Launch File

```xml
<launch>
  <!-- 1. MAVROS -->
  <include file="$(find mavros)/launch/apm.launch"/>
  
  <!-- 2. Robot description (URDF) or static TFs -->
  <include file="$(find my_robot_description)/launch/publish.launch"/>
  
  <!-- 3. Sensor drivers -->
  <include file="$(find ouster_ros)/launch/sensor.launch"/>
  
  <!-- 4. SLAM algorithm -->
  <include file="$(find my_slam)/launch/slam.launch"/>
  
  <!-- 5. TF to MAVROS bridge -->
  <node pkg="vision_to_mavros" type="vision_to_mavros_node" name="slam_to_mavros" output="screen">
    <param name="target_frame_id" value="map"/>
    <param name="source_frame_id" value="base_link"/>
    <param name="output_rate" value="30"/>
    <remap from="~/vision_pose" to="/mavros/vision_pose/pose"/>
  </node>
  
  <!-- 6. Set EKF origin -->
  <node pkg="vision_to_mavros" type="set_origin2.py" name="set_origin_node" output="screen"/>
</launch>
```

### 2.3 Testing Protocol

Follow this sequence (never skip steps):

**Phase 1: Bench Test (No Props)**

```bash
# Terminal 1: Launch system
roslaunch my_integration master.launch

# Terminal 2: Monitor topics
rostopic hz /mavros/vision_pose/pose      # Should be ~30 Hz
rostopic hz /slam/odometry                # SLAM output rate
rostopic echo /mavros/state               # Check connected: True

# Terminal 3: Check TF tree
rosrun tf view_frames
evince frames.pdf  # Verify map→odom→base_link→sensors

# Terminal 4: Verify EKF
rostopic echo /mavros/local_position/pose  # ArduPilot's fused estimate
```

**Expected**: Vision pose publishing, MAVROS connected, TF tree complete

**Phase 2: Ground Test (Props On, Tethered)**

```bash
# Manually move drone (tethered), verify:
rostopic echo /slam/odometry -n 1           # Position changes
rostopic echo /mavros/local_position/pose -n 1  # EKF tracks movement

# Test mode switching
rosservice call /mavros/set_mode "custom_mode: 'GUIDED'"
rosservice call /mavros/set_mode "custom_mode: 'LOITER'"
```

**Phase 3: Controlled Flight Test**

- Start in LOITER (GPS mode outdoors)
- Switch to GUIDED mode
- Send waypoint via `/mavros/setpoint_position/local`
- Monitor position error
- Test RTL failsafe

**Phase 4: GPS-Denied Indoor Test**

- Set `EK3_SRC1_POSXY=6` (vision mode)
- Verify EKF origin set
- Hover test (30 seconds)
- Small waypoint navigation
- Monitor drift over time

---

## Part 3: URDF Guidance

### 3.1 When You HAVE a URDF

**Advantages**:

- Single source of truth for robot geometry
- Automatic TF tree publishing
- Easy to update sensor positions
- Works with RViz, Gazebo, MoveIt

**Usage**:

1. Define all sensor frames in URDF
2. Launch `robot_state_publisher` to publish static TFs
3. SLAM algorithm uses these frames automatically
4. `vision_to_mavros` reads from TF tree

**Example** (from `dragunfly_tf.urdf`):

```xml
<link name="base_link"/>
<link name="os1_sensor"/>
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="os1_sensor"/>
  <origin xyz="0 0 -0.060" rpy="3.14159 0 0"/>
</joint>
```

### 3.2 When You DON'T HAVE a URDF

**Workarounds**:

**Option 1: Create Minimal URDF**

```xml
<?xml version="1.0"?>
<robot name="minimal_robot">
  <link name="base_link"/>
  <link name="lidar_sensor"/>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <!-- MEASURE these values with calipers! -->
    <origin xyz="0.0 0.0 -0.06" rpy="3.14159 0 0"/>
  </joint>
</robot>
```

Save as `minimal.urdf`, launch with:

```xml
<param name="robot_description" textfile="$(find my_pkg)/urdf/minimal.urdf"/>
<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>
```

**Option 2: Static Transform Publishers**

```xml
<!-- Launch file -->
<launch>
  <node pkg="tf" type="static_transform_publisher" name="base_to_lidar"
        args="0 0 -0.06 3.14159 0 0 base_link os1_sensor 100"/>
  
  <node pkg="tf" type="static_transform_publisher" name="base_to_imu"
        args="0 0 0 0 0 0 base_link imu_link 100"/>
</launch>
```

**ROS2**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.06', '3.14159', '0', '0', 'base_link', 'os1_sensor']
        )
    ])
```

**Option 3: Hardcode Extrinsics in SLAM Config**

Some SLAM algorithms allow manual extrinsics:

```yaml
# LIO-SAM example
extrinsicTrans: [0.0, 0.0, -0.06]  # x, y, z offset
extrinsicRot: [-1, 0, 0,           # Rotation matrix (180° flip)
                0, -1, 0,
                0, 0, 1]
```

**Trade-offs**:

- ✅ Works without URDF
- ❌ Harder to visualize in RViz
- ❌ Must update config if sensor moves
- ❌ No automatic TF publishing

### 3.3 Measuring Sensor Transforms

**Tools**:

- Digital calipers (±0.01mm accuracy)
- Laser distance meter
- 3D CAD model of robot (if available)

**Process**:

1. Define `base_link` as vehicle center of mass or flight controller
2. Measure XYZ offset from `base_link` to sensor center
3. Determine rotation (use right-hand rule):
   - LiDAR often mounted upside-down: `rpy="π 0 0"`
   - Cameras looking forward: `rpy="0 0 0"`
   - IMU rotated 180°: `rpy="π 0 0"`

**Validation**:

```bash
rosrun tf tf_echo base_link os1_sensor
# Check translation/rotation match physical measurements
```

---

## Part 4: Algorithm-Specific Notes

### 4.1 LIO-SAM

- **Odometry topic**: `/lio_sam/mapping/odometry`
- **Publishes TF**: Yes (`map→odom→base_link`)
- **IMU required**: Yes (tightly coupled)
- **Loop closure**: Yes (optional, enable with `loopClosureEnableFlag: true`)
- **Implementation**: `onboard_flight_ops/loc_nav/lio_sam_onb/`

### 4.2 FAST-LIO / FAST-LIO2

- **Odometry topic**: `/Odometry` (default)
- **Publishes TF**: Yes
- **IMU required**: Yes (ESKF fusion)
- **Advantages**: Faster than LIO-SAM, simpler config
- **Config**: `config/ouster64.yaml`
- **Implementation**: `onboard_flight_ops/loc_nav/fast-lio_onb/`

### 4.3 COIN-LIO

- **Odometry topic**: `/coinlio_odom`
- **Publishes TF**: Yes
- **IMU required**: Yes
- **Special**: Image feature fusion (requires camera)
- **Implementation**: `onboard_flight_ops/loc_nav/coin_lio_onb/`

### 4.4 Cartographer

- **Odometry topic**: `/tracked_pose` (publishes `map→base_link` directly)
- **Publishes TF**: Yes
- **IMU required**: Optional (but recommended)
- **Config**: LUA configuration files

### 4.5 RTAB-Map

- **Odometry topic**: `/rtabmap/odom`
- **Publishes TF**: Yes (`map→odom`)
- **IMU required**: Optional
- **Special**: RGB-D or stereo camera + LiDAR fusion

### 4.6 OpenVINS

- **Odometry topic**: `/ov_msckf/odometry`
- **Publishes TF**: Yes
- **IMU required**: Yes (VIO requires IMU)
- **Special**: Visual-inertial only (no LiDAR)
- **Implementation**: `onboard_flight_ops/loc_nav/openvins_onb/`

---

## Part 5: Troubleshooting

### Issue: Vision pose not reaching ArduPilot

```bash
# Check MAVROS connection
rostopic echo /mavros/state  # connected should be True

# Check vision_to_mavros publishing
rostopic hz /mavros/vision_pose/pose  # Should be 20-40 Hz

# Check MAVROS plugin loaded
rosservice list | grep vision  # Should show vision_pose services

# Verify ArduPilot parameter
rosrun mavros mavparam get VISO_TYPE  # Should be 2
```

### Issue: EKF not using vision data

```bash
# Check EKF source parameters
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6 for vision
rosrun mavros mavparam get AHRS_EKF_TYPE   # Should be 3

# Check EKF origin set
rostopic echo /mavros/global_position/global  # Should have valid lat/lon

# Rerun set_origin if needed
rosrun vision_to_mavros set_origin2.py
```

### Issue: TF tree incomplete

```bash
# List available frames
rosrun tf tf_monitor

# Check for missing transforms
rosrun tf view_frames
evince frames.pdf  # Look for disconnected branches

# Verify robot_state_publisher running
rosnode list | grep robot_state_publisher

# Check URDF loaded
rosparam get /robot_description  # Should return URDF XML
```

### Issue: Large position drift

**Possible causes**:

1. Incorrect IMU noise parameters → Re-calibrate
2. Wrong extrinsics (LiDAR-to-IMU) → Measure and update URDF
3. Insufficient features in environment → Add visual markers
4. Loop closure disabled → Enable in SLAM config
5. IMU/LiDAR misalignment in time → Check synchronization

**Diagnostics**:

```bash
# Check SLAM covariance
rostopic echo /slam/odometry/pose/covariance  # Should be small and stable

# Monitor drift over time
rostopic echo /mavros/local_position/pose  # Watch for creep when stationary
```

### Issue: SLAM initialization failure

**Possible causes**:

1. Insufficient motion during startup → Move robot to build initial map
2. Poor feature environment → Test in textured area
3. IMU not calibrated → Ensure drone is level on startup
4. Wrong point cloud frame → Check `lidar_frame` parameter

**Diagnostics**:

```bash
# Check SLAM status
rostopic echo /slam/status  # Implementation-specific

# Verify point cloud
rostopic hz /ouster/points_aligned  # Should be 10-20 Hz
rviz  # Visualize point cloud quality
```

---

## Part 6: Example Configurations

### Example 1: LIO-SAM + Ouster OS1-64

See existing implementation:

- **Config**: `onboard_flight_ops/loc_nav/lio_sam_onb/config/params.yaml`
- **Launch**: `onboard_flight_ops/loc_nav/lio_sam_onb/launch/run_no_rviz.launch`
- **Integration**: `onboard_main_bringup/dragunfly_bringup_onb/launch/onboard_main.launch` lines 169, 217-223

**Key Configuration**:

```yaml
lio_sam:
  pointCloudTopic: "/ouster/points_aligned"
  imuTopic: "/mavros/imu/data"
  lidarFrame: "base_link"
  mapFrame: "map"
  
  imuAccNoise: 9.8339587228516070e-03
  imuGyrNoise: 2.3797672263922514e-03
  imuAccBiasN: 4.9935238366353052e-04
  imuGyrBiasN: 3.2341720308535394e-05
```

### Example 2: FAST-LIO + Ouster (Hypothetical)

**Config** (`fast_lio_onb/config/ouster64.yaml`):

```yaml
common:
  lid_topic: "/ouster/points_aligned"
  imu_topic: "/mavros/imu/data"
  
mapping:
  map_frame: "map"
  odom_frame: "odom"
  body_frame: "base_link"
  
imu:
  acc_n: 0.00983
  gyr_n: 0.00238
  acc_w: 0.000499
  gyr_w: 0.0000323
```

**Launch**:

```xml
<launch>
  <rosparam file="$(find fast_lio)/config/ouster64.yaml" command="load"/>
  <node pkg="fast_lio" type="fastlio_mapping" name="fastlio_node" output="screen"/>
  
  <node pkg="vision_to_mavros" type="vision_to_mavros_node" name="fastlio_to_mavros">
    <param name="target_frame_id" value="map"/>
    <param name="source_frame_id" value="base_link"/>
    <param name="output_rate" value="30"/>
    <remap from="~/vision_pose" to="/mavros/vision_pose/pose"/>
  </node>
</launch>
```

### Example 3: Visual SLAM (No LiDAR)

For ORB-SLAM3, VINS-Fusion, etc.:

**Launch**:

```xml
<launch>
  <!-- Camera driver -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch"/>
  
  <!-- Visual SLAM -->
  <node pkg="orb_slam3" type="stereo_node" name="orb_slam3">
    <remap from="/camera/left" to="/camera/infra1/image_rect_raw"/>
    <remap from="/camera/right" to="/camera/infra2/image_rect_raw"/>
  </node>
  
  <!-- TF → MAVROS -->
  <node pkg="vision_to_mavros" type="vision_to_mavros_node" name="vslam_to_mavros">
    <param name="target_frame_id" value="map"/>
    <param name="source_frame_id" value="base_link"/>
    <remap from="~/vision_pose" to="/mavros/vision_pose/pose"/>
  </node>
</launch>
```

**ArduPilot Config**: Same EKF3 parameters as LiDAR SLAM

---

## Summary Checklist

When integrating a new SLAM algorithm:

- [ ] Configure SLAM to use `/mavros/imu/data`
- [ ] Set frame IDs: `map`, `odom`, `base_link`
- [ ] Create/update URDF with sensor positions (or use static TF publishers)
- [ ] Launch `robot_state_publisher` or static transform publishers
- [ ] Configure `vision_to_mavros` to bridge SLAM → MAVROS
- [ ] Run `set_origin2.py` to initialize EKF origin
- [ ] Set ArduPilot `EK3_SRC1_POSXY=6`, `EK3_SRC1_VELXY=6`
- [ ] Set `VISO_TYPE=2`, `AHRS_EKF_TYPE=3`
- [ ] Disable GPS arming checks: `ARMING_CHECK=388598`
- [ ] Test bench → ground → flight progressively
- [ ] Monitor `/mavros/vision_pose/pose` publishing at 20-40 Hz
- [ ] Verify EKF fuses vision data in `/mavros/local_position/pose`

---

## Key Insights

**The integration pattern is identical across SLAM algorithms**. Only the SLAM-specific config and topics change. The TF tree, `vision_to_mavros` bridge, and ArduPilot EKF setup remain the same.

**Critical Components**:

1. **TF Tree**: `map→odom→base_link→sensors` (URDF or static publishers)
2. **Bridge Node**: `vision_to_mavros` converts SLAM output to MAVROS format
3. **EKF Origin**: `set_origin2.py` initializes local coordinate system
4. **ArduPilot Config**: EKF3 sources set to external nav (source 6)

**Success Criteria**:

- Vision pose publishes at 20-40 Hz
- ArduPilot EKF converges (<1m position error stationary)
- Drone tracks commanded positions in GUIDED mode
- Minimal drift over time (<0.1 m/min stationary)

---

## References

- **LIO-SAM Paper**: https://github.com/TixiaoShan/LIO-SAM
- **ArduPilot EKF3**: https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html
- **MAVROS Vision Pose**: https://docs.px4.io/main/en/ros/external_position_estimation.html
- **ROS TF2**: http://wiki.ros.org/tf2
- **Ultra-onboard Testing Guide**: `onboard_flight_ops/loc_nav/ego_swarm_onb/docs/TESTING_GUIDE.md`

---

**Document Version**: 1.0  
**Last Updated**: November 22, 2025  
**Maintainer**: Ultra-onboard Development Team

