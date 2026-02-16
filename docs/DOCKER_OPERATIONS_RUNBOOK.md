# Docker SLAM Operations Runbook

**Version**: 1.0
**Status**: Production Ready
**Last Updated**: 2026-02-08
**System**: FAST-LIO + STD + Ouster OS1-64 + Cube Orange (ArduPilot)

---

## Quick Reference

### Start System
```bash
cd ~/slam_ws
docker compose up -d slam_launch
docker compose logs -f slam_launch
```

### Stop System
```bash
cd ~/slam_ws
docker compose down
```

### Verify Health
```bash
cd ~/slam_ws
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosnode list"
```

### Run Diagnostics
```bash
python3 ~/slam-agent/scripts/docker_diagnostics.py
```

---

## 1. Pre-Flight Operations

### 1.1 System Prerequisites

**Host Machine**:
- Docker: v20.10+
- docker-compose: v1.29+
- Available RAM: 2+ GB free (container uses 1-2 GB)
- Available Disk: 6+ GB for image
- Kernel: Linux 5.0+

**Check Prerequisites**:
```bash
docker --version
docker compose --version
free -h
df -h /
```

### 1.2 Hardware Connections

**Verify Hardware Before Launch**:
```bash
# Check USB devices
ls -la /dev/ttyACM*

# Check Ethernet for Ouster
ip link show | grep eth0
ping -c 1 169.254.56.220

# Check flight controller connectivity
lsusb | grep -i "FTDI\|Silicon Labs"
```

### 1.3 Network Configuration

**For Ouster LiDAR Access**:
```bash
# Set Ouster Ethernet to static IP
sudo ip addr add 169.254.56.1/24 dev eth0

# Or configure via NetworkManager
nmcli connection modify "Ouster" ipv4.method manual \
  ipv4.addresses 169.254.56.1/24
```

---

## 2. Starting the System

### 2.1 Quick Start (All-in-One)

```bash
cd ~/slam_ws
docker compose up -d slam_launch
```

**What This Does**:
1. Starts MAVROS bridge (flight controller interface)
2. Starts Ouster LiDAR driver
3. Starts FAST-LIO SLAM engine
4. Starts vision odometry to MAVROS converter
5. Publishes transforms via robot_state_publisher

**Expected Time**: 10-15 seconds until all nodes ready

### 2.2 Monitor Startup

```bash
# Watch logs in real-time
docker compose logs -f slam_launch

# Check specific node status
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosnode info /laserMapping"
```

### 2.3 Startup Sequence in Logs

```
=== ROS Noetic SLAM System ===
Starting SLAM container...
Running preflight checks...

[1] NETWORK CONNECTIVITY
  ✓ Ouster sensor reachable (169.254.56.220)
Environment ready. ROS_MASTER_URI=http://localhost:11311
Starting complete SLAM system (MAVROS + Ouster + FAST-LIO + Vision Bridge)...

[Nodes spawning...]
[/mavros-1] spawning /opt/ros/noetic/lib/mavros/mavros_node /mavros
[/ouster/os_node-5] spawning /catkin_ws/devel/lib/ouster_ros/...
[/fastlio_mapping-9] spawning /catkin_ws/devel/lib/fast_lio/...
[/slam_to_mavros-11] spawning /catkin_ws/devel/lib/vision_to_mavros/...
```

---

## 3. System Monitoring

### 3.1 Health Status Checks

```bash
#!/bin/bash
# Quick health check script

echo "=== Container Status ==="
docker compose ps

echo "=== Active Nodes ==="
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosnode list | sort"

echo "=== Publishing Topics ==="
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic list -b | wc -l"

echo "=== SLAM Odometry Rate ==="
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && timeout 5 rostopic hz /Odometry"
```

### 3.2 Resource Usage

```bash
# Monitor container resource usage in real-time
docker stats slam_launch

# Check memory pressure
docker compose exec -T slam_launch bash -c "free -h"

# Check CPU load
docker compose exec -T slam_launch bash -c "top -bn1 | head -10"
```

### 3.3 Running Diagnostics

```bash
# Comprehensive diagnostic suite
python3 ~/slam-agent/scripts/docker_diagnostics.py

# Output example:
# ✓ Docker Image Check: Image exists (Size: 5.11GB)
# ✓ Container Status: Container is running
# ✓ ROS Environment: ROS environment correctly configured
# ✓ ROS Nodes: ROS nodes running (7 total, 3 critical)
# ✓ ROS Topics: ROS topics publishing (12 total, 3 critical)
# ✓ ROS Packages: All 6 required packages found
# ✓ Launch Files: 26 launch files found
# ✓ SLAM Odometry: SLAM odometry topic is publishing
```

---

## 4. Common Operations

### 4.1 Access Container Shell

```bash
# Interactive bash shell
docker compose exec slam_launch bash

# Or non-interactive for scripts
docker compose exec -T slam_launch bash -c "command here"
```

### 4.2 View ROS Topics

```bash
# List all topics
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic list"

# Monitor specific topic
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic echo /Odometry"

# Topic publishing rate
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic hz /ouster/points"
```

### 4.3 Inspect Transform Tree

```bash
# View TF structure
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosrun tf view_frames"

# Check specific transform
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   rosrun tf tf_echo map camera_init"
```

### 4.4 Check Flight Controller Connection

```bash
# MAVROS system status
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   rostopic echo /mavros/state -n 1"

# Vision odometry input to FC
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   rostopic info /mavros/vision_pose/pose"
```

### 4.5 Record Flight Data (rosbag)

```bash
# Start recording all topics to bag file
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   cd /catkin_ws/bags && \
   rosbag record -a &"

# Or record specific topics
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   cd /catkin_ws/bags && \
   rosbag record /Odometry /ouster/points /mavros/vision_pose/pose"
```

---

## 5. Configuration Management

### 5.1 Modify SLAM Parameters

**File**: `~/slam_ws/src/orin_slam_integration/config/fast_lio_config.yaml`

Changes apply immediately on next node restart:

```bash
# Edit config
nano ~/slam_ws/src/orin_slam_integration/config/fast_lio_config.yaml

# Restart just the SLAM node
docker compose restart slam_launch  # or stop/start
```

**Key Parameters**:
- `lid_topic`: LiDAR point cloud topic (default: `/ouster/points`)
- `imu_topic`: IMU data topic (default: `/ouster/imu`)
- `lidar_type`: 3 for Ouster
- `scan_line`: 64 for OS1-64
- `scan_rate`: 10 Hz

### 5.2 Modify STD Loop Closure

**File**: `~/slam_ws/src/orin_slam_integration/config/std_config.yaml`

```yaml
max_keyframe_num: 1000  # Reduce if RAM limited
```

### 5.3 Modify Flight Controller Parameters

**File**: `~/slam_ws/src/orin_slam_integration/config/ardupilot_params.parm`

Deploy to flight controller:

```bash
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   rosrun mavros mavparam load \
   /catkin_ws/src/orin_slam_integration/config/ardupilot_params.parm"
```

**Key Parameters**:
- `EK3_SRC1_POSXY`: Source for position (6 = vision)
- `EK3_SRC1_VELXY`: Source for velocity (6 = vision)
- `VISO_TYPE`: Vision odometry type (1 = vision)
- `FENCE_ENABLE`: Geofence enable (1 = enabled)

---

## 6. Troubleshooting

### 6.1 Container Won't Start

```bash
# Check logs
docker compose logs slam_launch | tail -50

# If device issues:
docker compose down
sudo systemctl restart docker
docker compose up -d slam_launch

# If port conflicts:
docker ps  # Find conflicting containers
docker rm <container_id>
docker compose up -d slam_launch
```

### 6.2 No ROS Topics Publishing

```bash
# Check if ROS master is running
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosmaster 2>&1 | head -5"

# Check node status
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rosnode list"

# If no nodes, restart
docker compose restart slam_launch
```

### 6.3 SLAM Odometry Not Publishing

```bash
# Check if LiDAR data arriving
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   timeout 5 rostopic hz /ouster/points"

# If no LiDAR data:
# 1. Check network: ping 169.254.56.220
# 2. Check Ouster driver: rostopic info /ouster/os_node
# 3. Restart LiDAR driver: rosnode kill /ouster/os_node

# Check SLAM node logs
docker compose logs slam_launch | grep fastlio_mapping
```

### 6.4 High Memory Usage

```bash
# Check memory
docker compose exec -T slam_launch bash -c "free -h"

# If >6GB used, reduce STD keyframes
# Edit ~/slam_ws/src/orin_slam_integration/config/std_config.yaml
# Change: max_keyframe_num: 200  (from 1000)

# Restart
docker compose restart slam_launch
```

### 6.5 High CPU Usage

```bash
# Monitor CPU per process
docker compose exec -T slam_launch bash -c "top -b -n 1 | head -15"

# If SLAM CPU high: could be processing heavy point clouds
# Options:
# 1. Reduce LiDAR frame rate in Ouster config
# 2. Use faster host hardware
# 3. Reduce SLAM feature point count

# If STD CPU high:
# 1. Disable STD: roslaunch orin_slam_integration master.launch enable_std:=false
# 2. Or reduce keyframes in std_config.yaml
```

---

## 7. Stopping & Cleanup

### 7.1 Normal Shutdown

```bash
# Graceful shutdown
docker compose down

# This:
# 1. Stops all nodes cleanly
# 2. Closes rosbag recordings
# 3. Stops the container
# 4. Preserves data volumes
```

### 7.2 Emergency Stop

```bash
# Force kill (if stuck)
docker compose kill slam_launch

# Or hard stop
docker stop slam_launch
```

### 7.3 Clean Up Data

```bash
# Clear logs
rm ~/slam_ws/logs/*

# Clear recorded bags
rm ~/slam_ws/bags/*.bag

# Clear maps
rm ~/slam_ws/maps/*.pcd

# Prune Docker (careful!)
docker system prune -a  # Removes unused images
```

---

## 8. Pre-Flight Checklist

Before autonomous flight:

- [ ] Container started and all nodes running: `docker compose ps`
- [ ] LiDAR data flowing: `rostopic hz /ouster/points`
- [ ] SLAM odometry publishing: `rostopic hz /Odometry`
- [ ] Vision odometry to FC: `rostopic info /mavros/vision_pose/pose`
- [ ] Flight controller connected: `/mavros/state connected=true`
- [ ] Geofence enabled: `EK3_ENABLE=1, FENCE_ENABLE=1`
- [ ] RC transmitter paired and working
- [ ] Battery fully charged
- [ ] Propellers secure and balanced
- [ ] Preflight check passed: All green lights
- [ ] GPS disabled (GPS-denied mode): `GPS_TYPE=0`
- [ ] EKF3 configured for vision: `EK3_SRC1_POSXY=6`

### Run Automated Pre-Flight

```bash
cd ~/slam_ws
docker compose exec slam_launch /usr/local/bin/preflight_check.sh
```

---

## 9. Flight Operations

### 9.1 During Flight - Monitor

**Terminal 1: Logs**
```bash
docker compose logs -f slam_launch | grep -E "error|ERROR|fail|FAIL"
```

**Terminal 2: Diagnostics Loop**
```bash
while true; do
  docker compose exec -T slam_launch bash -c \
    "source /catkin_ws/devel/setup.bash && \
     echo 'Odom:' && rostopic info /Odometry | head -1 && \
     echo 'FC:' && rostopic info /mavros/vision_pose/pose | head -1"
  sleep 5
done
```

**Terminal 3: Record Data**
```bash
cd ~/slam_ws/bags
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   rosbag record -a"
```

### 9.2 After Flight - Analysis

```bash
# Copy flight data from container
docker compose cp slam_launch:/catkin_ws/bags/flight_2026-02-08.bag .

# Analyze bag file
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && \
   python3 ~/slam-agent/scripts/analyze_slam_bag.py \
   /catkin_ws/bags/flight_2026-02-08.bag"

# Generate metrics:
# - Position drift over time
# - Loop closure effectiveness
# - Processing latency
# - Memory usage trends
```

---

## 10. Performance Tuning

### 10.1 SLAM Tuning

For better accuracy/faster processing:

```yaml
# fast_lio_config.yaml
down_sample_rate: 1     # Reduce if CPU limited (1-4)
feature_extract_enable: 1
cube_side_length: 1000  # Larger = more memory, better accuracy
runtime_pos_log: 1      # Enable position logging
```

### 10.2 STD Tuning

For better loop closure:

```yaml
# std_config.yaml
max_keyframe_num: 500   # Reduce if RAM limited
descriptor_near_num: 20
descriptor_far_num: 30
```

### 10.3 Flight Controller Tuning

For better position hold:

```yaml
# ardupilot_params.parm
VISO_POS_M_NSE: 0.1     # Vision position noise (lower = trust more)
EK3_ENABLE: 1           # Enable EKF3
EK3_MAG_CAL: 3          # Mag calibration on ground
```

---

## 11. Reference

### Image Info
- **Size**: 5.11 GB
- **Base**: Ubuntu 20.04 (focal)
- **ROS**: Noetic
- **Ceres**: 2.1.0
- **GTSAM**: 4.0.3

### Key Ports (Host Networking)
- **ROS Master**: 11311 (localhost)
- **Ouster LiDAR**: 169.254.56.220:8192
- **Flight Controller**: /dev/ttyACM0 (USB)

### Key Directories (In Container)
- `/catkin_ws/devel` - Built binaries & scripts
- `/catkin_ws/src` - Source code
- `/catkin_ws/data` - Flight data (mounted from ~/slam_ws/data)
- `/catkin_ws/bags` - rosbag recordings (mounted from ~/slam_ws/bags)

### Useful Commands

```bash
# All at once
docker compose up -d slam_launch && sleep 10 && python3 ~/slam-agent/scripts/docker_diagnostics.py

# Auto-restart on failure
docker compose up -d --force-recreate slam_launch

# Watch for errors
docker compose logs -f slam_launch 2>&1 | grep -i "error\|fail\|warn"

# Full system rebuild (slow)
cd ~/slam_ws && docker build -t slam_integration:latest . --no-cache
```

---

**Status**: ✅ Production Ready
**Last Tested**: 2026-02-08
**Next Update**: After next test flight
