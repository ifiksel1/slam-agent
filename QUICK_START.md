# SLAM Integration Quick Start Guide

**One-page reference for GPS-denied navigation setup**

---

## 🎯 Goal
Integrate SLAM algorithm with ArduPilot/PX4 for autonomous indoor/GPS-denied flight

---

## Setup (One-Time)

```bash
pip3 install -r mcp/requirements.txt   # Enables MCP tools for Claude/Cursor
```

## 3-Minute Decision Tree

```
START: What's your experience level?

├─ BEGINNER (first time doing this)
│  └─> Use Claude Code or Cursor with the SLAM Integration skill
│      ├─ Say "Help me integrate SLAM with my drone"
│      ├─ Answer questions about your hardware
│      └─ The agent handles installation, testing, and learning
│
├─ INTERMEDIATE (done some ROS/SLAM)
│  └─> Use: docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md Part 2
│      ├─ Read 7-step integration process
│      ├─ Copy templates from docs/SLAM_INTEGRATION_TEMPLATE.md
│      └─ Test systematically
│
└─ EXPERT (know what you're doing)
   └─> Use: docs/SLAM_INTEGRATION_TEMPLATE.md
       ├─ Copy configs and modify
       ├─ Run scripts/slam_diagnostics.sh to verify
       └─ Reference main guide if needed
```

---

## 📋 Essential Checklist (Before You Start)

- [ ] Hardware
  - [ ] Onboard computer (4GB+ RAM recommended)
  - [ ] LiDAR or camera sensor
  - [ ] Flight controller (ArduPilot 4.0+ or PX4 v1.10+)
  - [ ] Telemetry connection (USB, UART, or UDP)
  - [ ] **If Ethernet LiDAR**: Network cable + configured static IP (see below)
  
- [ ] Software  
  - [ ] ROS1 (Noetic) or ROS2 (Humble) installed
  - [ ] MAVROS installed and tested
  - [ ] SLAM algorithm installed (from source or apt)
  - [ ] Sensor driver installed (ouster_ros, velodyne_driver, etc.)

- [ ] Knowledge
  - [ ] Basic ROS concepts (topics, nodes, launch files)
  - [ ] How to measure sensor positions (calipers, ruler)
  - [ ] Basic Linux command line
  - [ ] How to arm/disarm your drone safely

---

## 🚀 15-Minute Express Setup (If Everything Works)

**Assumptions**: You have Ultra-onboard-like setup with compatible hardware

### Step 1: Configure SLAM (5 min)
```yaml
# Edit config/slam_params.yaml
pointCloudTopic: "/your_lidar_topic"  # e.g., /ouster/points
imuTopic: "/mavros/imu/data"           # Standard
lidarFrame: "your_lidar_frame"         # e.g., os1_sensor
baselinkFrame: "base_link"
```

### Step 2: Set Up TF Tree (3 min)
```bash
# Option A: Create minimal URDF (see docs/AI_SYSTEM_BUILDER_GUIDE.md line 740+)
# Option B: Use static transforms
rosrun tf static_transform_publisher 0 0 -0.06 3.14159 0 0 base_link lidar_link 100
```

### Step 3: Configure ArduPilot (5 min)
```bash
# Load parameter file
rosrun mavros mavparam load config/autopilot.parm

# Or set manually:
rosrun mavros mavparam set EK3_SRC1_POSXY 6    # Vision for position
rosrun mavros mavparam set EK3_SRC1_VELXY 6    # Vision for velocity  
rosrun mavros mavparam set VISO_TYPE 2          # MAVLink vision
rosrun mavros mavparam set ARMING_CHECK 388598  # Disable GPS checks (indoor only!)
```

### Step 4: Test (2 min)
```bash
# Terminal 1: Launch system
roslaunch your_package master.launch

# Terminal 2: Check topics
rostopic hz /mavros/vision_pose/pose    # Should be 20-40 Hz
rostopic echo /mavros/state             # Check connected: True

# Terminal 3: Auto-diagnostic
./scripts/slam_diagnostics.sh
```

**If all green**: You're ready for ground tests!  
**If any red**: See troubleshooting below

---

## 🌐 Ethernet LiDAR Quick Setup (3 min)

**Most LiDARs use Ethernet** - configure before starting:

```bash
# 1. Configure your computer's network (Ubuntu):
sudo nano /etc/netplan/01-lidar.yaml

# Add (replace eth0 with your interface):
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]  # Your computer
      dhcp4: no

# 2. Apply and test:
sudo netplan apply
ping 192.168.1.201  # LiDAR's default IP

# 3. Check UDP data:
sudo tcpdump -i eth0 udp port 7502 -c 5  # Ouster
# OR
sudo tcpdump -i eth0 udp port 2368 -c 5  # Velodyne

# 4. If no data, allow firewall:
sudo ufw allow from 192.168.1.0/24
```

**Common LiDAR IPs**:
- Ouster: 192.168.1.201 (port 7502)
- Velodyne: 192.168.1.201 (port 2368)
- RoboSense: 192.168.1.200 (port 6699)

**Detailed guide**: See `docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` Step 1.5

---

## 🔧 Critical Parameters Reference

### ArduPilot EKF3 (GPS-denied indoor)
```
AHRS_EKF_TYPE = 3                    # Use EKF3
VISO_TYPE = 2                        # MAVLink vision pose
EK3_SRC1_POSXY = 6                   # External nav XY position
EK3_SRC1_VELXY = 6                   # External nav XY velocity
EK3_SRC1_POSZ = 1                    # Barometer altitude
EK3_SRC1_VELZ = 6                    # External nav Z velocity
ARMING_CHECK = 388598                # ⚠️ Disables GPS checks (use geofence!)
FENCE_ENABLE = 1                     # ✅ MUST enable geofence for safety
```

### PX4 EKF2 (GPS-denied indoor)
```
EKF2_AID_MASK = 24                   # Enable vision position + yaw
EKF2_HGT_MODE = 0                    # Barometric altitude
CBRK_GPSFAIL = 240024                # ⚠️ Disable GPS failure check
```

### Common SLAM Topics
| Algorithm | Odometry Output | Map Output | Input Topics |
|-----------|----------------|------------|--------------|
| LIO-SAM | /lio_sam/mapping/odometry | /lio_sam/mapping/cloud_registered | /points, /imu |
| FAST-LIO | /Odometry | /cloud_registered | /livox/lidar, /livox/imu |
| Cartographer | /tracked_pose | /map | /scan or /points, /imu |
| ORB-SLAM3 | /orb_slam3/camera_pose | /orb_slam3/map_points | /camera/image |
| RTAB-Map | /rtabmap/odom | /rtabmap/mapData | /camera/rgb, /camera/depth |

---

## 🐛 5-Minute Troubleshooting

### Issue: No vision pose reaching ArduPilot
```bash
# Check 1: MAVROS connected?
rostopic echo /mavros/state  # connected should be True

# Check 2: vision_to_mavros running?
rosnode list | grep vision

# Check 3: SLAM publishing?
rostopic hz /your_slam/odometry  # Should be >10 Hz

# Fix: Verify frame IDs match
rostopic echo /mavros/vision_pose/pose -n 1  # Check frame_id
```

### Issue: EKF not using vision data
```bash
# Check parameters
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6
rosrun mavros mavparam get VISO_TYPE       # Should be 2

# Check origin set
rostopic echo /mavros/global_position/global  # Should have valid lat/lon

# Fix: Rerun origin script
rosrun vision_to_mavros set_origin2.py
```

### Issue: TF tree incomplete
```bash
# View tree
rosrun tf view_frames
evince frames.pdf

# Check for missing links
# Should show: map -> odom -> base_link -> sensors

# Fix: Launch robot_state_publisher or static_transform_publishers
```

### Issue: Sensor time synchronization problems
```bash
# Check time offset between sensors (e.g., LiDAR vs FC IMU):
./scripts/check_sensor_time_sync.py /ouster/points /mavros/imu/data

# For VIO (camera vs IMU):
./scripts/check_sensor_time_sync.py /camera/image_raw /mavros/imu/data

# Good: <10ms offset | Warning: 10-50ms | Poor: >50ms
```

**Symptoms**: VIO initialization failures, scale drift, tracking loss  
**Causes**: Cross-sensor timestamp misalignment (LiDAR vs FC, Camera vs FC)  
**Quick fixes**:
- Use hardware-synchronized sensors when possible
- Check sensor driver timestamp configuration
- For VIO: Use Kalibr to estimate/compensate time offset

**Note**: Don't check sensors from same device (e.g., `/ouster/points` vs `/ouster/imu`) - they're hardware-synced already!

### Issue: Large drift
**Causes**: Wrong IMU parameters, incorrect extrinsics, no loop closure  
**Quick fixes**:
- Recalibrate IMU noise parameters (Allan variance)
- Verify sensor position measurements (±1cm accuracy needed)
- Enable loop closure in SLAM config
- Add visual features to environment

### Issue: No LiDAR data / Can't connect to LiDAR
**Causes**: Network configuration, wrong IP, firewall  
**Quick fixes**:
```bash
# Check connection
ping 192.168.1.201  # LiDAR default IP

# If ping fails - configure network:
sudo nano /etc/netplan/01-lidar.yaml
# Set IP to 192.168.1.100/24, apply: sudo netplan apply

# Check UDP packets
sudo tcpdump -i eth0 udp port 7502 -c 10  # Ouster
sudo tcpdump -i eth0 udp port 2368 -c 10  # Velodyne

# If ping works but no UDP - firewall:
sudo ufw allow from 192.168.1.0/24
```

---

## 🎓 Common Mistakes (Learn from Others!)

1. **❌ Forgot to set EKF origin** → Drone can't arm or drifts immediately  
   ✅ Run `set_origin2.py` at startup

2. **❌ Wrong frame IDs** → TF errors, no position estimate  
   ✅ Ensure lidarFrame matches your URDF/static TF

3. **❌ Disabled all arming checks without geofence** → DANGEROUS!  
   ✅ ALWAYS enable FENCE_ENABLE=1 when using ARMING_CHECK=388598

4. **❌ IMU topic wrong** → SLAM doesn't fuse properly  
   ✅ Use `/mavros/imu/data` for ArduPilot, verify with `rostopic echo`

5. **❌ Measured sensor position in mm, put in config as meters** → 1000x error!  
   ✅ Double-check units (config uses METERS)

6. **❌ LiDAR upside-down but didn't rotate in URDF** → Inverted map  
   ✅ Use `rpy="3.14159 0 0"` for 180° roll

7. **❌ Tested indoors with no features** → SLAM fails, pure drift  
   ✅ Add cardboard boxes, posters, structures for features

---

## 📞 Where to Get More Help

| Problem Type | Resource | Location |
|--------------|----------|----------|
| "How do I...?" | AI Assistant Guide | `docs/AI_SYSTEM_BUILDER_GUIDE.md` |
| Understanding how it works | Technical Reference | `docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` |
| Need config templates | Template File | `docs/SLAM_INTEGRATION_TEMPLATE.md` |
| Something's broken | Diagnostics Guide | `docs/SLAM_INTEGRATION_DIAGNOSTICS.md` |
| Automated check | Diagnostic Script | `scripts/slam_diagnostics.sh` |
| Check sensor time sync | Time Sync Checker | `scripts/check_sensor_time_sync.py` |
| Analyze flight data | Bag File Analyzer | `scripts/analyze_slam_bag.py` |
| Install SLAM algorithm | Installation Guides | https://github.com/engcang/SLAM-application |
| Working examples | Example Configs | `examples/README.md` |

---

## ⏱️ Realistic Timeline Expectations

- **Complete integration (first time)**: 4-8 hours
  - Hardware assembly/mounting: 1-2 hours
  - Software installation: 1-2 hours
  - Configuration/tuning: 1-2 hours
  - Testing/debugging: 1-2 hours

- **Adding new SLAM algorithm (to existing system)**: 1-2 hours
  - Config modification: 30 min
  - Testing/tuning: 30-90 min

- **Troubleshooting unexpected issues**: 30 min - 4 hours
  - Simple (topic remap): 5-30 min
  - Medium (TF tree): 30-60 min
  - Complex (drift/performance): 1-4 hours

**Pro tip**: Budget 2x your estimate for first integration!

---

## ✅ Ready to Start?

**Choose your path**:
- [ ] **Guided setup**: Open `docs/AI_SYSTEM_BUILDER_GUIDE.md` in an AI assistant
- [ ] **DIY setup**: Follow `docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` Part 2
- [ ] **Troubleshoot**: Run `scripts/slam_diagnostics.sh`

**Remember**: GPS-denied autonomous flight is inherently risky. Always:
- Test progressively (bench → tethered → small movements → full flight)
- Keep RC manual override ready
- Use geofence limits
- Have clear emergency procedures
- Never skip safety checks

Good luck! 🚁

