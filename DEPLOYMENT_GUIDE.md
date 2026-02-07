# SLAM System Deployment & Testing Guide

**System:** Jetson Orin NX with FAST-LIO2 + Ouster OS1-64 + ArduPilot
**Status:** Ready for Hardware Integration & Testing

---

## 🎯 Pre-Flight Checklist

### Hardware Verification
- [ ] Ouster OS1-64 LiDAR physically mounted
- [ ] Jetson Orin NX powered and booted
- [ ] USB/Network cables connected
- [ ] ArduPilot flight controller connected
- [ ] Batteries charged

### Software Verification
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/slam_ws/devel/setup.bash

# Verify installation
bash ~/slam-agent/scripts/verify_installation.sh ROS1 noetic ~/slam_ws
# Expected: All components installed ✓
```

---

## 🚀 Test Sequence

### Test 1: SLAM Alone (No Flight Controller)
**Duration:** 5-10 minutes
**Goal:** Verify SLAM is tracking properly

```bash
# Terminal 1: Launch SLAM
roslaunch fast_lio mapping_ouster64.launch

# Terminal 2: Check odometry
rostopic hz /Odometry

# Terminal 3: Record test data
rosbag record /Odometry /cloud_registered /os_cloud_node/points -o slam_test_1.bag

# Terminal 4: Monitor diagnostics
watch -n 1 'rostopic list | wc -l'
```

**Success Criteria:**
- ✓ `/Odometry` publishing at ~10 Hz
- ✓ `/cloud_registered` has point data
- ✓ `/path` topic shows trajectory
- ✓ System CPU < 10%, Memory < 500MB

---

### Test 2: Topic Pipeline Validation
**Duration:** 5 minutes
**Goal:** Verify all data is flowing correctly

```bash
# With SLAM running, check pipeline
python3 ~/slam-agent/scripts/check_topic_pipeline.py --duration 15

# Expected Output:
# Sensor: ACTIVE
# SLAM: ACTIVE
# Vision: NOT CONFIGURED (if not using vision)
# MAVROS: NOT RUNNING (until connected)
```

**Debug if Needed:**
```bash
# Check specific topic rates
rostopic hz /os_cloud_node/points
rostopic hz /os_cloud_node/imu
rostopic hz /Odometry
```

---

### Test 3: SLAM + MAVROS Integration
**Duration:** 5 minutes
**Goal:** Verify flight controller communication

```bash
# Terminal 1: SLAM (if not running)
roslaunch fast_lio mapping_ouster64.launch

# Terminal 2: MAVROS (assuming ArduPilot on USB port)
roslaunch mavros apm.launch fcu_url:=/dev/ttyUSB0:115200

# Terminal 3: Vision-to-MAVROS
roslaunch vision_to_mavros vision_to_mavros.launch

# Terminal 4: Check FC status
rostopic echo /mavros/state -n 5
```

**Success Criteria:**
- ✓ `/mavros/state` shows `connected: true`
- ✓ No latency warnings
- ✓ SLAM odometry flowing to FC

---

### Test 4: Complete System Test
**Duration:** 10-15 minutes
**Goal:** End-to-end validation before flight

```bash
# Automated test suite
bash ~/slam-agent/RUN_FULL_SYSTEM_TEST.sh

# This will:
# 1. Launch SLAM system
# 2. Run all 8 diagnostic scripts
# 3. Generate comprehensive report
# 4. Display results

# View results
cat /tmp/slam_test_*/diagnostic_report.txt
```

---

## 📊 Expected Performance

### Odometry Characteristics
| Parameter | Expected | Threshold |
|-----------|----------|-----------|
| Update Rate | 10 Hz | ±1 Hz |
| Position Jitter | <5 mm | <20 mm |
| Velocity Noise | <0.05 m/s | <0.2 m/s |
| Latency | <100 ms | <500 ms |

### System Resources
| Resource | Expected | Limit |
|----------|----------|-------|
| CPU | 3-5% | <25% |
| Memory | 200-300 MB | <2 GB |
| Disk I/O | <10 MB/s | <50 MB/s |

---

## 🔍 Troubleshooting

### Issue: Odometry Not Publishing
```bash
# Check if SLAM node is running
rosnode list | grep laserMapping

# If missing, check for errors
cat ~/.ros/log/latest/laserMapping-*.log

# Common causes:
# 1. No LiDAR data coming in
#    → Check /os_cloud_node/points
# 2. SLAM initialization timeout
#    → Provide more static features (walls, objects)
# 3. Configuration mismatch
#    → Verify lidar_type=3 in launch file
```

### Issue: High Latency
```bash
# Check processing time
rosbag play slam_test.bag --pause
# Publish rate should be 10 Hz with no lag

# If slow:
# 1. CPU bottleneck → Close other applications
# 2. Dense point cloud → Reduce point_filter_num in config
# 3. Feature extraction → Disable if not needed
```

### Issue: TF Tree Empty
```bash
# Expected: SLAM should publish tf at ~10 Hz
rostopic hz /tf

# If no output:
# 1. Launch robot_state_publisher
# 2. Verify URDF is loaded
# 3. Check FAST-LIO publishes transforms in config

# Temporary workaround: Use /Odometry directly for EKF3
```

---

## 📈 Performance Optimization

### For GPS-Denied Indoor Flight
```yaml
# mapping_ouster64.launch optimizations:
scan_line: 64              # Use all LiDAR lines
filter_size_surf: 0.5      # Aggressive filtering
feature_extract_enable: false  # Disable if slow
dense_publish_enable: true # Send full point cloud
```

### For Outdoor Use (if available)
```yaml
det_range: 200             # Increase detection range
filter_size_map: 1.0       # Larger map resolution
pcd_save_enable: false     # Disable map saving (storage)
```

---

## 🎓 Flight Testing Progression

### Phase 1: Tethered Flight (0-5 min)
- Launch in GPS-denied indoor area
- Monitor SLAM odometry in real-time
- Verify no dropout or jumps
- Confidence: Position within ±0.5 m

### Phase 2: Short Autonomous Flight (5-15 min)
- Pre-program simple waypoint mission
- Allow EKF3 to fuse SLAM with IMU
- Monitor telemetry for any errors
- Confidence: Stable hover ±1 m

### Phase 3: Extended Mission (15+ min)
- Complex waypoint missions
- Return-to-launch capability
- Monitor SLAM consistency over time
- Map accumulation check

### Phase 4: Production Deployment
- Full autonomous missions
- Real-world GPS-denied environments
- Production-grade logging & redundancy

---

## 📝 Data Collection

### Recording for Post-Analysis
```bash
# Full sensor suite recording
rosbag record \
  /Odometry \
  /cloud_registered \
  /cloud_registered_body \
  /os_cloud_node/points \
  /os_cloud_node/imu \
  /path \
  /tf \
  /tf_static \
  -o flight_mission_001.bag

# Verify recording
rosbag info flight_mission_001.bag
```

### Offline Analysis
```bash
# Analyze recorded flight
python3 ~/slam-agent/scripts/analyze_slam_bag.py \
  flight_mission_001.bag

# Expected output:
# - Trajectory length
# - Point cloud statistics
# - Loop closures detected (if enabled)
```

---

## 🔐 Safety Considerations

### Pre-Flight
- [ ] SLAM publishing odometry reliably
- [ ] No error messages in ROS logs
- [ ] Flight controller responding correctly
- [ ] Geofence configured (if available)
- [ ] Manual override tested

### During Flight
- [ ] Monitor RC control responsiveness
- [ ] Watch for SLAM tracking loss (odometry dropout)
- [ ] Be ready to take manual control
- [ ] Log all data for post-analysis

### Post-Flight
- [ ] Download flight logs and rosbags
- [ ] Analyze trajectory accuracy
- [ ] Check for any system warnings
- [ ] Review SLAM convergence

---

## 📞 Support Resources

### Built-In Diagnostics
```bash
# Complete system check
bash ~/slam-agent/scripts/verify_installation.sh ROS1 noetic ~/slam_ws
bash ~/slam-agent/scripts/slam_diagnostics.sh
python3 ~/slam-agent/scripts/check_tf_tree.py --verbose
```

### Log Analysis
```bash
# Review ROS logs
cat ~/.ros/log/latest/*/laserMapping-*.log

# Check system performance
top -b -n 1 | head -20
free -h
df -h ~/slam_ws
```

---

## ✅ Completion Checklist

Before first flight:
- [ ] SYSTEM_BUILD_COMPLETE.md reviewed
- [ ] All diagnostic tests passing
- [ ] Odometry test recorded and analyzed
- [ ] MAVROS integration tested
- [ ] TF transforms verified
- [ ] Safety procedures understood
- [ ] Backup power available
- [ ] Recovery procedure documented

**You're ready to fly! 🚀**

---

**Next:** Connect hardware and run Test 1 (SLAM Alone)
