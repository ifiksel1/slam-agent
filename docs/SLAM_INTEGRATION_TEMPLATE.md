# SLAM Integration Quick-Start Template

This template provides a step-by-step checklist for integrating any SLAM algorithm with ArduPilot for GPS-denied navigation in the Ultra-onboard system.

**Copy this file and fill in the placeholders for your specific SLAM algorithm.**

---

## Algorithm Information

- **SLAM Name**: `_______________` (e.g., LIO-SAM, FAST-LIO, Cartographer)
- **Package Name**: `_______________` (e.g., `my_slam_onb`)
- **ROS Version**: ROS1 Noetic / ROS2 Humble
- **Sensor Type**: LiDAR / Camera / RGB-D / Multi-modal
- **IMU Required**: Yes / No
- **Loop Closure**: Yes / No

---

## Pre-Integration Checklist

- [ ] SLAM algorithm compiled and tested standalone
- [ ] Sensor drivers working (LiDAR/camera)
- [ ] MAVROS connected to ArduPilot
- [ ] URDF created or static TF publishers configured
- [ ] ArduPilot parameters backed up
- [ ] Jetson Orin NX resources verified (CPU/GPU/RAM)

---

## Step 1: SLAM Configuration

### Topics Configuration

Fill in your SLAM algorithm's topic names:

```yaml
# Input topics
imu_topic: "/mavros/imu/data"              # ✓ Use ArduPilot's IMU
pointcloud_topic: "_______________"        # Your LiDAR topic
image_topic: "_______________"             # If using camera
depth_topic: "_______________"             # If using RGB-D

# Output topics
odometry_topic: "_______________"          # SLAM odometry output
map_topic: "_______________"               # Map output (if applicable)
```

### Frame IDs

```yaml
map_frame: "map"                           # ✓ Keep as "map"
odom_frame: "odom"                         # ✓ Keep as "odom"
base_frame: "base_link"                    # ✓ Keep as "base_link"
lidar_frame: "_______________"             # Your LiDAR frame (e.g., "os1_sensor")
camera_frame: "_______________"            # If using camera
```

### IMU Parameters

Use these calibrated values for ARK V6X flight controller, or calibrate your own:

```yaml
imu_acc_noise: 9.83e-03    # m/s²/√Hz (ARK V6X calibrated)
imu_gyr_noise: 2.38e-03    # rad/s/√Hz (ARK V6X calibrated)
imu_acc_bias: 4.99e-04     # m/s² (ARK V6X calibrated)
imu_gyr_bias: 3.23e-05     # rad/s (ARK V6X calibrated)
imu_gravity: 9.8           # m/s²
```

**Note**: If using a different flight controller, run Allan variance analysis to get accurate values.

### Extrinsics (LiDAR-IMU Transform)

**Option A: Using URDF** (Recommended)

Set to identity, let URDF/TF handle transformations:

```yaml
extrinsic_T: [0.0, 0.0, 0.0]
extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

**Option B: Without URDF**

Measure and enter LiDAR-to-IMU transform:

```yaml
extrinsic_T: [_____, _____, _____]  # [x, y, z] in meters (body frame: fwd, left, up)
extrinsic_R: [_____, _____, _____,  # Rotation matrix (row-major)
              _____, _____, _____,
              _____, _____, _____]
```

**Option C: Non-Standard LiDAR Mounting**

For LiDARs with different native frame (e.g., Hesai JT128 dome-forward):

```yaml
# Hesai JT128 dome pointing FORWARD:
# LiDAR frame: X=left, Y=up, Z=forward
# Body frame:  X=forward, Y=left, Z=up
extrinsic_T: [0.0, 0.0, 0.10]      # Adjust based on physical mounting
extrinsic_R: [0, 0, 1,             # Body X = LiDAR Z
              1, 0, 0,             # Body Y = LiDAR X
              0, 1, 0]             # Body Z = LiDAR Y
```

**Common LiDAR Frame Conventions**:
| LiDAR | Native Frame | extrinsic_R (if dome/sensor forward) |
|-------|--------------|--------------------------------------|
| Ouster | X=fwd, Y=left, Z=up | Identity [1,0,0,0,1,0,0,0,1] |
| Velodyne | X=fwd, Y=left, Z=up | Identity [1,0,0,0,1,0,0,0,1] |
| Hesai JT128 (dome fwd) | X=left, Y=up, Z=fwd | [0,0,1,1,0,0,0,1,0] |
| Livox Avia | Check datasheet | Varies by mounting |

**For calibration**: See `AI_SYSTEM_BUILDER_GUIDE.md` Section 8.5 (LiDAR-IMU Calibration)

---

## Step 2: TF Tree Setup

### Option A: Using URDF

Create or update URDF at: `onboard_utilities/_______________/urdf/robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link"/>
  <link name="_____lidar_frame_____"/>
  
  <joint name="_____lidar_joint_____" type="fixed">
    <parent link="base_link"/>
    <child link="_____lidar_frame_____"/>
    <!-- MEASURE these values! -->
    <origin xyz="_____ _____ _____" rpy="_____ _____ _____"/>
  </joint>
</robot>
```

Create launch file at: `onboard_utilities/_______________/launch/publish.launch`

```xml
<launch>
  <param name="robot_description" textfile="$(find _____)/urdf/robot.urdf" />
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>
</launch>
```

### Option B: Static Transform Publishers

Create launch file at: `onboard_utilities/_______________/launch/static_tfs.launch`

```xml
<launch>
  <!-- base_link → lidar -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_lidar"
        args="_____ _____ _____ _____ _____ _____ base_link _____lidar_frame_____ 100"/>
</launch>
```

---

## Step 3: SLAM Launch File

Create at: `onboard_flight_ops/loc_nav/_______________/launch/slam.launch`

```xml
<launch>
  <!-- Load parameters -->
  <rosparam file="$(find _____)/config/params.yaml" command="load"/>
  
  <!-- SLAM Node -->
  <node pkg="_____" type="_____" name="_____slam_node" output="screen">
    <!-- Topic remaps -->
    <remap from="~/_____imu_topic_____" to="/mavros/imu/data"/>
    <remap from="~/_____cloud_topic_____" to="/ouster/points_aligned"/>
    <remap from="~/_____odom_topic_____" to="/_____/odometry"/>
  </node>
</launch>
```

---

## Step 4: vision_to_mavros Integration

Add to your main launch file:

```xml
<!-- TF to MAVROS bridge -->
<node pkg="vision_to_mavros" type="vision_to_mavros_node" name="_____slam_to_mavros" output="screen">
  <param name="target_frame_id" value="map"/>
  <param name="source_frame_id" value="base_link"/>
  <param name="output_rate" value="30"/>
  <param name="gamma_world" value="0.0"/>
  
  <!-- Choose one: -->
  <!-- Option A: Use TF tree (if SLAM publishes transforms) -->
  <param name="use_tf" value="true"/>
  
  <!-- Option B: Use odometry topic (if SLAM doesn't publish TF) -->
  <!-- <param name="use_tf" value="false"/> -->
  <!-- <remap from="~/input_odom" to="/_____/odometry"/> -->
  
  <remap from="~/vision_pose" to="/mavros/vision_pose/pose"/>
</node>

<!-- Set EKF origin -->
<node pkg="vision_to_mavros" type="set_origin2.py" name="set_origin_node" output="screen"/>
```

---

## Step 5: ArduPilot Configuration

Create parameter file at: `onboard_flight_ops/loc_nav/_______________/configs/ardupilot_params.parm`

```
# EKF3 Configuration for GPS-denied navigation
AHRS_EKF_TYPE,3
VISO_TYPE,2

# Vision as position source
EK3_SRC1_POSXY,6
EK3_SRC1_VELXY,6
EK3_SRC1_POSZ,1              # Barometer (or 6 for vision Z)
EK3_SRC1_VELZ,6
EK3_SRC1_YAW,1               # Compass (or 6 for vision yaw)

# Disable GPS arming checks
ARMING_CHECK,388598

# Safety (adjust for your environment)
FENCE_ENABLE,1
FENCE_TYPE,7
FENCE_ACTION,1
FENCE_RADIUS,50
FENCE_ALT_MAX,25

# Velocity limits
WPNAV_SPEED,200
WPNAV_ACCEL,300
PSC_POSXY_P,1.5
```

**Load commands**:

```bash
# Backup
rosrun mavros mavparam dump ~/backup_$(date +%Y%m%d).parm

# Load
rosrun mavros mavparam load ~/onboard_ws/src/Ultra-onboard/onboard_flight_ops/loc_nav/_____/configs/ardupilot_params.parm

# Verify
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6
rosrun mavros mavparam get VISO_TYPE       # Should be 2

# Reboot
rosservice call /mavros/cmd/reboot "{}"
```

---

## Step 6: Master Launch File Integration

Update `onboard_main_bringup/dragunfly_bringup_onb/launch/onboard_main.launch`:

```xml
<!-- Add argument -->
<arg name="enable_____" default="true"/>

<!-- Add SLAM launch -->
<include if="$(arg enable_____)" file="$(find _____)/launch/slam.launch"/>

<!-- Add vision_to_mavros bridge -->
<node if="$(arg enable_____)" pkg="vision_to_mavros" type="vision_to_mavros_node" 
      name="_____to_mavros" output="screen">
  <param name="target_frame_id" value="map"/>
  <param name="source_frame_id" value="base_link"/>
  <param name="output_rate" value="30"/>
  <remap from="~/vision_pose" to="/mavros/vision_pose/pose"/>
</node>
```

---

## Step 7: Testing Checklist

### Phase 1: Bench Test (No Props)

```bash
# Launch system
roslaunch dragunfly_bringup onboard_main.launch enable_____:=true

# Verify topics
rostopic hz /mavros/vision_pose/pose      # Should be ~30 Hz ✓
rostopic hz /_____/odometry               # SLAM output rate ✓
rostopic echo /mavros/state               # connected: True ✓

# Check TF tree
rosrun tf view_frames
evince frames.pdf  # Verify complete tree ✓

# Monitor EKF
rostopic echo /mavros/local_position/pose  # Should update ✓
```

**Results**:
- [ ] Vision pose publishing at target rate
- [ ] MAVROS connected
- [ ] TF tree complete (no disconnected frames)
- [ ] EKF position updating

### Phase 2: Ground Test (Props On, Tethered)

```bash
# Move drone manually, verify odometry tracks
rostopic echo /_____/odometry -n 1
rostopic echo /mavros/local_position/pose -n 1

# Test mode switching
rosservice call /mavros/set_mode "custom_mode: 'GUIDED'"
rosservice call /mavros/set_mode "custom_mode: 'STABILIZE'"
```

**Results**:
- [ ] SLAM odometry tracks physical movement
- [ ] ArduPilot EKF tracks SLAM odometry
- [ ] Mode switching works
- [ ] No unusual errors in logs

### Phase 3: Flight Test

**Environment**: Open outdoor area with GPS

**Procedure**:
1. [ ] ARM in LOITER mode (GPS)
2. [ ] Takeoff to 2m
3. [ ] Switch to GUIDED mode
4. [ ] Send test waypoint via `/mavros/setpoint_position/local`
5. [ ] Monitor position tracking
6. [ ] Test RTL
7. [ ] Land and disarm

**Metrics to Record**:
- Position error: _____ cm (target: <50 cm)
- Velocity tracking: _____ m/s error (target: <0.2 m/s)
- Waypoint arrival accuracy: _____ cm (target: <100 cm)
- Drift rate stationary: _____ cm/min (target: <10 cm/min)

### Phase 4: GPS-Denied Indoor Test

**Prerequisites**:
- [ ] `EK3_SRC1_POSXY=6` set
- [ ] `EK3_SRC1_VELXY=6` set
- [ ] `ARMING_CHECK=388598` set
- [ ] EKF origin initialized
- [ ] Geofence configured for indoor space

**Procedure**:
1. [ ] Verify vision pose publishing
2. [ ] ARM in GUIDED mode (no GPS)
3. [ ] Hover test (30 seconds)
4. [ ] Small waypoint (1m forward)
5. [ ] Return to start position
6. [ ] Monitor drift
7. [ ] Land and disarm

**Metrics to Record**:
- Hover drift: _____ cm over 30s (target: <20 cm)
- Waypoint accuracy: _____ cm (target: <50 cm)
- Position covariance: _____ (check `/slam/odometry`)
- EKF health: Check `/mavros/state`

---

## Troubleshooting

### SLAM not initializing

**Check**:
- [ ] Sufficient motion during startup
- [ ] Point cloud publishing (`rostopic hz /ouster/points_aligned`)
- [ ] IMU data arriving (`rostopic hz /mavros/imu/data`)
- [ ] Environment has sufficient features

**Fix**:
```bash
# Verify sensor data
rostopic echo /ouster/points_aligned -n 1
rviz  # Visualize point cloud quality
```

### Vision pose not reaching ArduPilot

**Check**:
- [ ] `vision_to_mavros` node running
- [ ] `/mavros/vision_pose/pose` publishing
- [ ] `VISO_TYPE=2` parameter set
- [ ] MAVROS connected

**Fix**:
```bash
rosnode list | grep vision_to_mavros
rostopic hz /mavros/vision_pose/pose
rosrun mavros mavparam get VISO_TYPE
```

### EKF not using vision data

**Check**:
- [ ] `EK3_SRC1_POSXY=6`
- [ ] `AHRS_EKF_TYPE=3`
- [ ] EKF origin set (check `/mavros/global_position/global`)

**Fix**:
```bash
rosrun mavros mavparam get EK3_SRC1_POSXY
rosrun vision_to_mavros set_origin2.py
```

### Large position drift

**Possible Causes**:
1. Incorrect IMU noise parameters
2. Wrong extrinsics
3. Insufficient features
4. Loop closure disabled
5. Time synchronization issues

**Diagnostics**:
```bash
# Check SLAM covariance
rostopic echo /_____/odometry/pose/covariance

# Monitor stationary drift
rostopic echo /mavros/local_position/pose
```

---

## Documentation

Create README at: `onboard_flight_ops/loc_nav/_______________/README.md`

Include:
- [ ] Algorithm overview
- [ ] Dependencies and installation
- [ ] Configuration parameters
- [ ] Launch instructions
- [ ] Known issues and limitations
- [ ] Performance metrics (update rate, accuracy, CPU/GPU usage)

---

## Integration Verification

Final checklist before production deployment:

- [ ] All configuration files created
- [ ] Launch files tested
- [ ] ArduPilot parameters loaded and verified
- [ ] Bench tests passed
- [ ] Ground tests passed
- [ ] Flight tests passed (GPS mode)
- [ ] GPS-denied tests passed
- [ ] Documentation complete
- [ ] Code committed to repository
- [ ] Integration added to main launch file
- [ ] Team members trained on new system

---

## Performance Targets

| Metric | Target | Actual |
|--------|--------|--------|
| SLAM update rate | 10-20 Hz | _____ Hz |
| Vision pose rate | 30 Hz | _____ Hz |
| Position accuracy (hover) | <20 cm | _____ cm |
| Position accuracy (waypoint) | <50 cm | _____ cm |
| Drift rate (stationary) | <10 cm/min | _____ cm/min |
| CPU usage (Jetson) | <60% | _____ % |
| GPU usage (Jetson) | <70% | _____ % |
| RAM usage | <10 GB | _____ GB |

---

## Notes

Use this section to document algorithm-specific quirks, lessons learned, or optimization tips:

- 
- 
- 

---

**Template Version**: 1.0  
**Date**: November 22, 2025  
**Reference**: See `docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` for detailed explanations

