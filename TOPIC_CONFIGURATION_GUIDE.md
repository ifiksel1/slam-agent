# Topic Configuration Guide
**Ouster OS1-64 + FAST-LIO2 + ROS Noetic**

---

## ⚠️ Critical Issue: Topic Name Mismatches

### What Happened (2026-02-07 Incident)
The SLAM configuration file expected LiDAR data on `/os_cloud_node/points` but the Ouster driver was publishing to `/ouster/points`. This caused complete silent failure - no error messages, just no data flowing.

**Symptom:** System appears to work but SLAM never generates odometry.

---

## Correct Topic Configuration

### Ouster Driver Topics
These are **published by** the Ouster ROS driver:

```
/ouster/points              ← Point clouds (65536 pts @ 10 Hz)
/ouster/imu                 ← IMU data (@ 100 Hz)
/ouster/range_image         ← Range image
/ouster/reflec_image        ← Reflectance image
/ouster/lidar_packets       ← Raw lidar packets
/ouster/imu_packets         ← Raw IMU packets
/ouster/metadata            ← Sensor metadata
```

### FAST-LIO2 SLAM Configuration
These are **expected by** SLAM (configured in `ouster64.yaml`):

```yaml
# CORRECT (current):
common:
    lid_topic:  "/ouster/points"
    imu_topic:  "/ouster/imu"
```

### What Went Wrong
The config file had outdated topic names:

```yaml
# WRONG (old default):
common:
    lid_topic:  "/os_cloud_node/points"     ← doesn't exist!
    imu_topic:  "/os_cloud_node/imu"        ← doesn't exist!
```

---

## How to Verify Configuration

### Step 1: Check Available Topics
```bash
# List all Ouster-related topics
rostopic list | grep -i ouster

# Expected output:
# /ouster/imu
# /ouster/imu_packets
# /ouster/lidar_packets
# /ouster/metadata
# /ouster/nearir_image
# /ouster/points              ← THIS ONE (LiDAR data)
# /ouster/points2
# ... etc
```

### Step 2: Verify SLAM Config File
```bash
# Check config has correct topic names
cat /home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml | head -5

# Should show:
# common:
#     lid_topic:  "/ouster/points"
#     imu_topic:  "/ouster/imu"
```

### Step 3: Check Data is Flowing
```bash
# Monitor LiDAR topic
rostopic hz /ouster/points
# Should show: average rate: 10.00 Hz

# Check one message
rostopic echo /ouster/points -n 1 | head -20
# Should show PointCloud2 message with data

# Check SLAM is getting it
rostopic echo /Odometry -n 1
# Should show position data (x, y, z)
```

### Step 4: Run Pre-Flight Check
```bash
chmod +x /home/dev/slam-agent/preflight_check.sh
/home/dev/slam-agent/preflight_check.sh
```

---

## Configuration Files Location

### SLAM Config
```
File: /home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml
Lines 2-3: Topic name configuration
```

### Ouster Driver Launch
```
File: /home/dev/slam_ws/src/ouster-ros/launch/driver.launch
Publishes to: /ouster/* topics
Parameters: sensor_hostname, lidar_port, imu_port
```

### SLAM Launch
```
File: /home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/launch/mapping_ouster64.launch
Reads config: ouster64.yaml
Publishes to: /Odometry, /cloud_registered, /path
```

---

## Troubleshooting Checklist

### Issue: SLAM not publishing odometry

```
1. Check topics exist:
   rostopic list | grep ouster

2. Verify SLAM sees them:
   cat /home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml

3. Confirm data flowing:
   rostopic hz /ouster/points

4. Check SLAM is running:
   rosnode list | grep laserMapping

5. Run pre-flight check:
   /home/dev/slam-agent/preflight_check.sh
```

### Issue: Wrong topics in config

**Symptom:** Config points to `/os_cloud_node/points` instead of `/ouster/points`

**Fix:**
```bash
# Edit the config file
nano /home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml

# Change line 2: "/os_cloud_node/points" → "/ouster/points"
# Change line 3: "/os_cloud_node/imu" → "/ouster/imu"

# Save and restart SLAM
rosnode kill /laserMapping
roslaunch fast_lio mapping_ouster64.launch
```

---

## Working Configuration Summary

```
Ouster Hardware
    ↓ (Ethernet: 7502 LiDAR, 7503 IMU)
Ouster ROS Driver
    ↓ (publishes to /ouster/*)
ROS Topics
    /ouster/points          ← Point clouds
    /ouster/imu             ← IMU data
    ↓
SLAM (FAST-LIO2)
    [reads config: ouster64.yaml]
    [subscribes to /ouster/points and /ouster/imu]
    ↓
SLAM Output
    /Odometry               ← Position (x,y,z) @ 10 Hz
    /cloud_registered      ← Registered clouds
    /path                   ← Trajectory
```

---

## Key Files to Monitor

| File | Purpose | Critical Settings |
|------|---------|------------------|
| `/home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml` | SLAM config | Topic names (lines 2-3) |
| `/home/dev/slam_ws/src/ouster-ros/launch/driver.launch` | Driver launch | Port configuration (7502/7503) |
| `/home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/launch/mapping_ouster64.launch` | SLAM launch | rviz flag |

---

## Prevention

### Before Starting Any Test
```bash
/home/dev/slam-agent/preflight_check.sh
```

### Add to System Documentation
- [x] Topic configuration documented (this file)
- [x] Config files use correct topic names
- [x] Pre-flight check script created
- [x] Lesson learned in memory file

### If Adding New Sensors
1. Verify sensor publishes to correct topics: `rostopic list`
2. Update config file with sensor's topic names
3. Test data flow before running SLAM
4. Run pre-flight check

---

## Quick Reference

**Correct Topic Names:**
- LiDAR: `/ouster/points`
- IMU: `/ouster/imu`

**Correct Ports:**
- LiDAR: 7502
- IMU: 7503

**Verify Before Flight:**
```bash
/home/dev/slam-agent/preflight_check.sh
```

---

**Last Updated:** 2026-02-07
**Status:** ✅ Production Verified
