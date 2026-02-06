# SLAM Integration Diagnostics & Troubleshooting

**Purpose**: Systematic diagnostic guide for troubleshooting SLAM to ArduPilot integration issues.

**Use Case**: When your SLAM integration isn't working, follow this flowchart to identify and fix the problem.

---

## Quick Diagnostic Command Reference

Copy and paste these commands to quickly diagnose issues:

```bash
# === System Status ===
rostopic list | grep -E "(mavros|slam|lio|odom|vision)"
rosnode list | grep -E "(mavros|slam|lio|vision|robot_state)"
rostopic hz /mavros/vision_pose/pose /mavros/imu/data /ouster/points_aligned

# === MAVROS Connection ===
rostopic echo /mavros/state -n 1

# === ArduPilot Parameters ===
rosrun mavros mavparam get EK3_SRC1_POSXY
rosrun mavros mavparam get EK3_SRC1_VELXY
rosrun mavros mavparam get VISO_TYPE
rosrun mavros mavparam get AHRS_EKF_TYPE

# === TF Tree ===
rosrun tf view_frames && evince frames.pdf
rosrun tf tf_monitor

# === EKF Status ===
rostopic echo /mavros/local_position/pose -n 1
rostopic echo /mavros/global_position/global -n 1

# === SLAM Output ===
rostopic info /lio_sam/mapping/odometry  # Replace with your SLAM odometry topic
rostopic echo /lio_sam/mapping/odometry -n 1
```

---

## Diagnostic Flowchart

### Level 1: System Connectivity

```
START → Is MAVROS connected to ArduPilot?
        │
        ├─ NO → Check /mavros/state
        │       ├─ "connected: False"
        │       │   → Check serial/UDP connection
        │       │   → Verify FCU powered on
        │       │   → Check /dev/ttyACM0 or fcu_url
        │       └─ Topic doesn't exist
        │           → MAVROS not running
        │           → Launch MAVROS: roslaunch mavros apm.launch
        │
        └─ YES → Continue to Level 2
```

**Commands**:
```bash
# Check MAVROS state
rostopic echo /mavros/state

# Expected output:
# connected: True
# armed: False
# mode: "STABILIZE"

# If not connected, check connection
ls /dev/ttyACM*  # Should show /dev/ttyACM0
# OR
netstat -an | grep 14550  # For UDP connection

# Check MAVROS running
rosnode list | grep mavros
```

**Common Fixes**:
- Reconnect USB cable to flight controller
- Check `fcu_url` parameter in `apm.launch`
- Verify ArduPilot firmware version compatible with MAVROS
- Check serial port permissions: `sudo usermod -a -G dialout $USER`

---

### Level 2: Sensor Data Flow

```
Are sensors publishing data?
│
├─ LiDAR point cloud?
│  ├─ NO → rostopic hz /ouster/points_aligned
│  │       → Check Ouster driver launched
│  │       → Verify network connection (ping 169.254.9.99)
│  └─ YES → Continue
│
├─ IMU data from MAVROS?
│  ├─ NO → rostopic hz /mavros/imu/data
│  │       → Check MAVROS connected
│  │       → Verify ArduPilot IMU working
│  └─ YES → Continue
│
└─ All sensors OK → Continue to Level 3
```

**Commands**:
```bash
# Check LiDAR
rostopic hz /ouster/points_aligned
# Expected: ~10-20 Hz

rostopic echo /ouster/points_aligned -n 1
# Should show point cloud data

# Check LiDAR driver node
rosnode list | grep ouster

# Check Ouster network
ping 169.254.9.99
# Should respond if LiDAR powered and connected

# Check IMU
rostopic hz /mavros/imu/data
# Expected: ~100-400 Hz

rostopic echo /mavros/imu/data -n 1
# Should show IMU readings

# Visualize sensors
rviz
# Add PointCloud2 → Topic: /ouster/points_aligned
# Add Imu → Topic: /mavros/imu/data
```

**Common Fixes**:
- LiDAR: Check power, network cable, IP configuration
- LiDAR: Verify `ouster_ros` package installed
- IMU: Check MAVROS parameter `conn/system_id` and `conn/component_id`
- IMU: Verify ArduPilot sensors initialized (check GCS)

---

### Level 3: TF Tree Completeness

```
Is TF tree complete?
│
├─ Check: rosrun tf view_frames
│  
├─ Does map→odom→base_link exist?
│  ├─ NO → SLAM not publishing transforms
│  │       → Check SLAM node running
│  │       → Verify SLAM initialized
│  │       → Check SLAM config (TF publishing enabled)
│  └─ YES → Continue
│
├─ Does base_link→sensors exist?
│  ├─ NO → URDF not loaded OR robot_state_publisher not running
│  │       → Check robot_description parameter
│  │       → Launch robot_state_publisher
│  │       → OR: Add static_transform_publishers
│  └─ YES → Continue
│
└─ TF tree complete → Continue to Level 4
```

**Commands**:
```bash
# Generate TF tree diagram
rosrun tf view_frames
evince frames.pdf

# Expected frames:
# map → odom → base_link → os1_sensor
#                        → fcu
#                        → other sensors

# Check specific transform
rosrun tf tf_echo map base_link
# Should show transform updating

# List all frames
rosrun tf tf_monitor

# Check robot_state_publisher
rosnode list | grep robot_state_publisher

# Check URDF loaded
rosparam get /robot_description | head -n 10
# Should show XML

# Check static transforms
rosnode list | grep static_transform
```

**Common Fixes**:
- SLAM not running: `rosnode list | grep slam`
- SLAM not initialized: Move robot to build initial map
- URDF not loaded: Check `robot_description` parameter exists
- robot_state_publisher not running: Launch it
- Missing static transforms: Add `static_transform_publisher` nodes

---

### Level 4: SLAM Operation

```
Is SLAM running and producing output?
│
├─ Is SLAM node running?
│  ├─ NO → rosnode list | grep slam
│  │       → Launch SLAM
│  └─ YES → Continue
│
├─ Is SLAM publishing odometry?
│  ├─ NO → rostopic hz /<slam>/odometry
│  │       → Check SLAM logs for errors
│  │       → Verify SLAM subscribed to correct topics
│  └─ YES → Continue
│
├─ Is odometry data valid?
│  ├─ Check covariance (should be small)
│  ├─ Check position updates (should change with motion)
│  └─ Check for NaN or inf values
│
└─ SLAM OK → Continue to Level 5
```

**Commands**:
```bash
# Check SLAM node
rosnode list | grep -i slam
rosnode info /<slam_node_name>

# Check SLAM subscriptions
rosnode info /<slam_node_name> | grep -A 20 Subscriptions
# Should show:
#  - /mavros/imu/data
#  - /ouster/points_aligned

# Check SLAM publications
rosnode info /<slam_node_name> | grep -A 20 Publications
# Should show odometry topic

# Check odometry publishing
rostopic hz /lio_sam/mapping/odometry  # Replace with your topic
# Expected: 10-100 Hz depending on algorithm

# Check odometry data
rostopic echo /lio_sam/mapping/odometry -n 1

# Check for errors
rostopic echo /rosout | grep -i error
rostopic echo /rosout | grep -i warn

# Check SLAM-specific status topics
rostopic list | grep -i status
```

**Common Fixes**:
- SLAM not subscribing to IMU: Check IMU topic remap in launch file
- SLAM not subscribing to point cloud: Check point cloud topic/frame
- SLAM initialization failure: Ensure sufficient motion during startup
- Poor odometry: Check environment has sufficient features
- High covariance: Tune SLAM parameters (IMU noise, etc.)

---

### Level 5: vision_to_mavros Bridge

```
Is vision_to_mavros converting and publishing pose?
│
├─ Is vision_to_mavros node running?
│  ├─ NO → rosnode list | grep vision_to_mavros
│  │       → Launch vision_to_mavros
│  └─ YES → Continue
│
├─ Is it receiving SLAM data?
│  ├─ Check mode (TF or odometry topic)
│  ├─ If TF mode: Can it lookup map→base_link?
│  ├─ If topic mode: Is odometry topic correct?
│  └─ Check logs for transform errors
│
├─ Is it publishing /mavros/vision_pose/pose?
│  ├─ NO → rostopic hz /mavros/vision_pose/pose
│  │       → Check vision_to_mavros logs
│  └─ YES → Continue
│
├─ Is pose data valid?
│  └─ Check position matches SLAM odometry
│
└─ Bridge OK → Continue to Level 6
```

**Commands**:
```bash
# Check bridge node
rosnode list | grep vision_to_mavros
rosnode info /slam_to_mavros  # Or your node name

# Check vision pose publishing
rostopic hz /mavros/vision_pose/pose
# Expected: 20-40 Hz

rostopic echo /mavros/vision_pose/pose -n 1

# Check TF lookup (if using TF mode)
rosrun tf tf_echo map base_link
# Should update at SLAM rate

# Check odometry subscription (if using topic mode)
rosnode info /slam_to_mavros | grep Subscriptions
# Should show SLAM odometry topic

# Check for transform errors
rosout | grep "vision_to_mavros"
rosout | grep "TransformException"

# Verify position matches
# Terminal 1:
rostopic echo /lio_sam/mapping/odometry/pose/pose/position
# Terminal 2:
rostopic echo /mavros/vision_pose/pose/pose/position
# Should be similar (accounting for frame transforms)
```

**Common Fixes**:
- Node not running: Check launch file includes vision_to_mavros
- TF mode errors: Verify TF tree complete (Level 3)
- Topic mode errors: Check odometry topic name in remap
- Wrong frame: Check `target_frame_id` and `source_frame_id` parameters
- No output: Check `output_rate` parameter
- Position mismatch: Check `gamma_world` rotation parameter

---

### Level 6: ArduPilot EKF Configuration

```
Is ArduPilot configured to use vision data?
│
├─ Check EKF3 enabled
│  ├─ AHRS_EKF_TYPE == 3?
│  │  ├─ NO → rosrun mavros mavparam set AHRS_EKF_TYPE 3
│  │  └─ YES → Continue
│
├─ Check vision source configured
│  ├─ VISO_TYPE == 2?
│  │  ├─ NO → rosrun mavros mavparam set VISO_TYPE 2
│  │  └─ YES → Continue
│  │
│  ├─ EK3_SRC1_POSXY == 6?
│  │  ├─ NO → rosrun mavros mavparam set EK3_SRC1_POSXY 6
│  │  └─ YES → Continue
│  │
│  └─ EK3_SRC1_VELXY == 6?
│     ├─ NO → rosrun mavros mavparam set EK3_SRC1_VELXY 6
│     └─ YES → Continue
│
├─ Check EKF origin set
│  └─ rostopic echo /mavros/global_position/global
│      ├─ Valid lat/lon? → OK
│      └─ 0,0 or invalid? → Run set_origin2.py
│
└─ Configuration OK → Continue to Level 7
```

**Commands**:
```bash
# Check all critical parameters
rosrun mavros mavparam get AHRS_EKF_TYPE    # Should be 3
rosrun mavros mavparam get VISO_TYPE        # Should be 2
rosrun mavros mavparam get EK3_SRC1_POSXY   # Should be 6
rosrun mavros mavparam get EK3_SRC1_VELXY   # Should be 6
rosrun mavros mavparam get EK3_SRC1_POSZ    # Should be 1 or 6
rosrun mavros mavparam get EK3_SRC1_VELZ    # Should be 6
rosrun mavros mavparam get ARMING_CHECK     # Should be 388598 for indoor

# Check EKF origin
rostopic echo /mavros/global_position/global -n 1
# Should show valid latitude/longitude

# Set origin if needed
rosrun vision_to_mavros set_origin2.py

# Load all parameters from file
rosrun mavros mavparam load ~/onboard_ws/src/Ultra-onboard/onboard_flight_ops/loc_nav/<slam>/configs/ardupilot_params.parm

# Reboot after parameter changes
rosservice call /mavros/cmd/reboot "{}"
```

**Common Fixes**:
- Wrong AHRS type: Set to 3 (EKF3)
- Vision not enabled: Set VISO_TYPE to 2
- GPS still primary: Change EK3_SRC1_POSXY/VELXY to 6
- Origin not set: Run set_origin2.py
- Can't arm indoors: Set ARMING_CHECK to 388598 (disables GPS check)

---

### Level 7: EKF Fusion Verification

```
Is ArduPilot EKF using vision data?
│
├─ Check /mavros/local_position/pose updating
│  ├─ NO → EKF not fusing vision
│  │       → Check all previous levels
│  │       → Check EKF status messages
│  └─ YES → Continue
│
├─ Does position track SLAM output?
│  ├─ Move robot manually
│  ├─ Compare SLAM odom vs MAVROS local_position
│  └─ Should match within ~10-20cm
│
├─ Check EKF covariance
│  └─ /mavros/local_position/pose/covariance
│      └─ Should be small and stable
│
└─ EKF Fusion OK → System Working!
```

**Commands**:
```bash
# Check EKF output
rostopic hz /mavros/local_position/pose
# Should update at high rate (50-100 Hz)

rostopic echo /mavros/local_position/pose -n 1

# Compare SLAM vs EKF positions
# Terminal 1 - SLAM:
rostopic echo /lio_sam/mapping/odometry/pose/pose/position
# Terminal 2 - EKF:
rostopic echo /mavros/local_position/pose/pose/position
# Should track each other

# Check EKF covariance
rostopic echo /mavros/local_position/pose/covariance

# Move robot and verify tracking
# Physically move drone (props off!) and watch positions update

# Check EKF status
rostopic echo /mavros/state
# ekf_ok should be True
```

**Common Fixes**:
- EKF not updating: Vision data not reaching ArduPilot (check Level 5 & 6)
- Position mismatch: Frame transformation issue (check gamma_world)
- Large covariance: SLAM uncertainty high (improve environment/tuning)
- EKF divergence: Reset by rebooting ArduPilot
- Position jumps: Loop closure causing corrections (normal, but tune if excessive)

---

## Common Error Messages and Solutions

### Error: "Lookup would require extrapolation into the past"

**Cause**: Time synchronization issue between nodes

**Solution**:
```bash
# Check system time
date

# Check ROS time
rostopic echo /clock

# Ensure all nodes use same time source
# Check launch files for use_sim_time parameter

# For hardware: Ensure NTP sync
sudo timedatectl set-ntp true
```

### Error: "Could not find a connection between 'map' and 'base_link'"

**Cause**: TF tree incomplete - SLAM not publishing transforms

**Solution**:
```bash
# Check SLAM running
rosnode list | grep slam

# Check TF tree
rosrun tf view_frames

# Verify SLAM initialized (needs motion)
# Move robot to build initial map

# Check SLAM config enables TF publishing
```

### Error: "Vision pose message too old"

**Cause**: vision_to_mavros publishing too slowly

**Solution**:
```bash
# Check publish rate
rostopic hz /mavros/vision_pose/pose

# Increase output_rate parameter
# In launch file: <param name="output_rate" value="40"/>

# Check SLAM not dropping frames
rostopic hz /slam/odometry
```

### Warning: "VISO: NO VISION DATA"

**Cause**: ArduPilot not receiving vision messages

**Solution**:
```bash
# Verify vision_pose publishing
rostopic echo /mavros/vision_pose/pose -n 1

# Check MAVROS vision plugin loaded
rosnode info /mavros | grep vision

# Verify VISO_TYPE=2
rosrun mavros mavparam get VISO_TYPE

# Restart MAVROS
rosnode kill /mavros
roslaunch mavros apm.launch
```

### Error: "GPS: NO GPS"

**Cause**: GPS disabled for indoor flight (expected)

**Solution**:
```bash
# This is normal for GPS-denied operation
# Verify EK3 using vision instead:
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6

# Disable GPS arming checks
rosrun mavros mavparam set ARMING_CHECK 388598

# If need GPS outdoors, change back:
# rosrun mavros mavparam set EK3_SRC1_POSXY 3
```

---

## Performance Diagnostic Checklist

Use this checklist to verify system performance:

### Latency

```bash
# Measure SLAM latency
# Manually move robot and observe delay

# Check processing times in logs
rostopic echo /rosout | grep -i time

# Monitor system resources
htop  # CPU usage
nvidia-smi  # GPU usage (if applicable)
```

**Targets**:
- SLAM latency: <100ms
- vision_to_mavros latency: <30ms
- Total latency (sensor → EKF): <150ms

### Update Rates

```bash
# Check all critical rates
rostopic hz /ouster/points_aligned        # Target: 10-20 Hz
rostopic hz /mavros/imu/data              # Target: 100-400 Hz
rostopic hz /slam/odometry                # Target: 10-100 Hz
rostopic hz /mavros/vision_pose/pose      # Target: 30-40 Hz
rostopic hz /mavros/local_position/pose   # Target: 50-100 Hz
```

### Accuracy

```bash
# Stationary drift test
# Place robot stationary for 1 minute
rostopic echo /mavros/local_position/pose/pose/position

# Record start and end position
# Calculate drift: sqrt(dx^2 + dy^2 + dz^2)
# Target: <10 cm/minute
```

### Resource Usage

```bash
# CPU per node
rosrun rqt_top rqt_top

# Memory usage
free -h

# GPU usage (Jetson)
tegrastats

# Disk I/O
iostat -x 1
```

**Targets** (Jetson Orin NX):
- Total CPU: <60%
- Per-core CPU: <90%
- RAM: <10 GB
- GPU: <70%

---

## Log File Locations

```bash
# ROS logs
~/.ros/log/latest/

# MAVROS logs
rosnode log /mavros

# SLAM-specific logs
rosnode log /<slam_node>

# System logs
journalctl -u roslaunch

# ArduPilot logs (download via MAVROS)
rosrun mavros mavlogdump
```

---

## Emergency Recovery Procedures

### SLAM Lost Tracking

```bash
# Stop sending commands
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "{}" -1

# Land immediately (if flying)
rosservice call /mavros/cmd/land "{}"

# OR: RTL (Return to Launch)
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'RTL'}"

# After landing:
# 1. Check logs for error
# 2. Restart SLAM
# 3. Re-verify TF tree
# 4. Re-test on ground before flying
```

### EKF Divergence

```bash
# Land immediately
rosservice call /mavros/cmd/land "{}"

# Reboot flight controller
rosservice call /mavros/cmd/reboot "{}"

# After reboot:
# 1. Re-run set_origin2.py
# 2. Verify vision pose publishing
# 3. Check EKF converges on ground
# 4. Re-test hover before mission
```

### Complete System Restart

```bash
# 1. Kill all ROS nodes
rosnode kill -a

# 2. Kill roscore
killall roscore rosmaster

# 3. Wait 5 seconds
sleep 5

# 4. Restart system
roslaunch dragunfly_bringup onboard_main.launch

# 5. Verify all nodes started
rosnode list

# 6. Run full diagnostic (Level 1-7)
```

---

## Diagnostic Script

Save this as `~/onboard_ws/src/Ultra-onboard/scripts/slam_diagnostics.sh`:

```bash
#!/bin/bash

echo "===== SLAM Integration Diagnostics ====="
echo ""

echo "[1/7] Checking MAVROS Connection..."
timeout 2 rostopic echo /mavros/state -n 1 | grep connected && echo "✓ MAVROS Connected" || echo "✗ MAVROS NOT Connected"

echo ""
echo "[2/7] Checking Sensors..."
timeout 2 rostopic hz /ouster/points_aligned --window 10 | grep "average rate" && echo "✓ LiDAR OK" || echo "✗ LiDAR NOT Publishing"
timeout 2 rostopic hz /mavros/imu/data --window 10 | grep "average rate" && echo "✓ IMU OK" || echo "✗ IMU NOT Publishing"

echo ""
echo "[3/7] Checking TF Tree..."
timeout 5 rosrun tf tf_echo map base_link 2>&1 | grep "At time" && echo "✓ TF Tree Complete" || echo "✗ TF Tree INCOMPLETE"

echo ""
echo "[4/7] Checking SLAM..."
timeout 2 rostopic hz /lio_sam/mapping/odometry --window 10 | grep "average rate" && echo "✓ SLAM OK" || echo "✗ SLAM NOT Publishing"

echo ""
echo "[5/7] Checking vision_to_mavros..."
timeout 2 rostopic hz /mavros/vision_pose/pose --window 10 | grep "average rate" && echo "✓ Bridge OK" || echo "✗ Bridge NOT Publishing"

echo ""
echo "[6/7] Checking ArduPilot Parameters..."
EK3=$(rosrun mavros mavparam get EK3_SRC1_POSXY 2>&1 | grep "value:" | awk '{print $2}')
VISO=$(rosrun mavros mavparam get VISO_TYPE 2>&1 | grep "value:" | awk '{print $2}')
[[ "$EK3" == "6.0" ]] && echo "✓ EK3_SRC1_POSXY = 6" || echo "✗ EK3_SRC1_POSXY = $EK3 (should be 6)"
[[ "$VISO" == "2.0" ]] && echo "✓ VISO_TYPE = 2" || echo "✗ VISO_TYPE = $VISO (should be 2)"

echo ""
echo "[7/7] Checking EKF Fusion..."
timeout 2 rostopic hz /mavros/local_position/pose --window 10 | grep "average rate" && echo "✓ EKF Fusing" || echo "✗ EKF NOT Fusing"

echo ""
echo "===== Diagnostic Complete ====="
echo "See SLAM_INTEGRATION_DIAGNOSTICS.md for detailed troubleshooting"
```

Make executable:
```bash
chmod +x ~/onboard_ws/src/Ultra-onboard/scripts/slam_diagnostics.sh
```

Run:
```bash
~/onboard_ws/src/Ultra-onboard/scripts/slam_diagnostics.sh
```

---

### Level 8: Calibration Verification

```
Is SLAM tracking correctly in all directions?
│
├─ Move FORWARD → X increases in /Odometry?
│  ├─ NO → LiDAR-IMU extrinsic_R wrong
│  │       → See AI_SYSTEM_BUILDER_GUIDE.md Section 8.5 (LiDAR-IMU calibration)
│  └─ YES → Continue
│
├─ Move LEFT → Y increases in /Odometry?
│  ├─ NO → Y-axis sign error in extrinsics
│  │       → Check LiDAR frame convention (may need rotation)
│  └─ YES → Continue
│
├─ Move UP → Z increases in /Odometry?
│  ├─ NO → Z-axis sign error
│  │       → Check mounting orientation
│  └─ YES → Continue
│
├─ /mavros/vision_pose matches /Odometry direction?
│  ├─ NO → vision_to_mavros gamma_world or yaw_cam parameter wrong
│  │       → Check ENU/NED conversion
│  └─ YES → Continue
│
└─ All directions OK → Calibration verified! ✓
```

**Commands**:
```bash
# Step 1: Monitor SLAM output
rostopic echo /Odometry/pose/pose/position

# Step 2: Test each axis (move sensor by hand)
# Move FORWARD → X should INCREASE
# Move LEFT    → Y should INCREASE
# Move UP      → Z should INCREASE

# Step 3: Compare SLAM and vision_pose
rostopic echo /mavros/vision_pose/pose/pose/position
# Should match /Odometry direction

# Step 4: Check point cloud quality in RViz
rviz
# Add PointCloud2 display for /cloud_registered or similar
# Walls should be straight, not curved
# No smearing during motion
```

**Common Calibration Issues**:

| Symptom | Cause | Fix |
|---------|-------|-----|
| X correct, Y inverted | 180° yaw error in extrinsics | Fix extrinsic_R rotation |
| Both axes wrong | Major frame mismatch | Recalculate extrinsic_R for your LiDAR |
| Axes swapped (X↔Y) | LiDAR frame different from body | Check LiDAR manufacturer frame convention |
| Works in RViz, wrong on drone | ENU/NED mismatch | Check vision_to_mavros gamma_world |
| Point cloud smeared | Time offset or bad extrinsics | Run LI-Calib for temporal calibration |

**LiDAR Frame Reference**:
```
ROS Body Frame (FLU): X=forward, Y=left, Z=up
ArduPilot (NED):      X=north, Y=east, Z=down
MAVROS topics:        Always in ENU (East-North-Up)

Common LiDAR conventions:
- Ouster/Velodyne: X=fwd, Y=left, Z=up (standard, identity)
- Hesai JT128 dome-forward: X=left, Y=up, Z=fwd (needs rotation!)
```

**For LiDAR-IMU calibration guidance**: See `AI_SYSTEM_BUILDER_GUIDE.md` Section 8.5

---

## Summary

**Follow this diagnostic flow**:

1. **Level 1**: MAVROS connected to ArduPilot?
2. **Level 2**: Sensors publishing data?
3. **Level 3**: TF tree complete?
4. **Level 4**: SLAM running and publishing?
5. **Level 5**: vision_to_mavros bridging correctly?
6. **Level 6**: ArduPilot configured for vision?
7. **Level 7**: EKF fusing vision data?
8. **Level 8**: Calibration verified? (directions match, no smearing)

**Each level builds on the previous**. Don't skip ahead if an earlier level fails.

**Quick health check**:
```bash
rostopic hz /mavros/vision_pose/pose /slam/odometry /mavros/local_position/pose
```
All three should publish at reasonable rates (10-100 Hz).

---

**Document Version**: 1.0  
**Last Updated**: November 22, 2025  
**Reference**: See `SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` for integration instructions

