# Docker SLAM Operations Runbook - ROS2 Humble

**Version**: 1.0
**Status**: Production Ready
**ROS Version**: ROS2 Humble
**Last Updated**: 2026-02-08
**System**: FAST-LIO + STD + Ouster OS1-64 + Cube Orange (ArduPilot)

---

## Quick Reference

### Start System
```bash
cd ~/slam_ws
ROS_VERSION=humble docker compose up -d slam_launch
docker compose logs -f slam_launch
```

### Stop System
```bash
docker compose down
```

### Verify Health
```bash
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Run Diagnostics
```bash
python3 ~/slam-agent/scripts/docker_diagnostics_ros2.py
```

---

## 1. Pre-Flight Operations

### 1.1 System Prerequisites (ROS2 Specific)

**ROS2 Requirements**:
- ROS2 Humble (Ubuntu 22.04 Jammy)
- colcon build system
- Python 3.10+
- DDS middleware (FastDDS by default)

**Verify Prerequisites**:
```bash
# Check ROS2 installation
ros2 --version

# Check colcon
colcon --version

# Check Python
python3 --version  # Should be 3.10+
```

### 1.2 Hardware Connections

Same as ROS1:
```bash
# Check USB devices
ls -la /dev/ttyACM*

# Check Ethernet for Ouster
ping -c 1 169.254.56.220
```

---

## 2. Starting the System

### 2.1 Quick Start (All-in-One)

**Option 1: Using version selector script**
```bash
cd ~/slam_ws
./select_ros_version.sh switch humble
./select_ros_version.sh start
```

**Option 2: Using docker compose directly**
```bash
cd ~/slam_ws
ROS_VERSION=humble docker compose -f docker-compose.multi.yml up -d slam_launch
```

**Option 3: Using .env file**
```bash
cd ~/slam_ws
echo "ROS_VERSION=humble" > .env
docker compose -f docker-compose.multi.yml up -d slam_launch
```

### 2.2 Monitor Startup

```bash
# Watch logs in real-time
docker compose logs -f slam_launch

# Check specific node status
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node info /laserMapping"
```

---

## 3. System Monitoring (ROS2 Commands)

### 3.1 Key Differences from ROS1

| Operation | ROS1 (Noetic) | ROS2 (Humble) |
|-----------|--------------|--------------|
| List nodes | `rosnode list` | `ros2 node list` |
| List topics | `rostopic list` | `ros2 topic list` |
| Echo topic | `rostopic echo /topic` | `ros2 topic echo /topic` |
| Topic rate | `rostopic hz /topic` | `ros2 topic hz /topic` |
| Topic info | `rostopic info /topic` | `ros2 topic info /topic` |
| View frames | `rosrun tf view_frames` | `ros2 run tf2_tools view_frames.py` |
| Record bag | `rosbag record -a` | `ros2 bag record -a` |
| Play bag | `rosbag play file.bag` | `ros2 bag play file.bag` |
| Launch | `roslaunch pkg launch.launch` | `ros2 launch pkg launch.py` |

### 3.2 Health Status Checks (ROS2)

```bash
#!/bin/bash
# Quick health check script for ROS2

echo "=== Container Status ==="
docker compose ps

echo "=== Active Nodes ==="
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list | sort"

echo "=== Publishing Topics ==="
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list | wc -l"

echo "=== SLAM Odometry Rate ==="
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /Odometry"

echo "=== ROS Domain ID ==="
docker compose exec -T slam_launch bash -c \
  "echo ROS_DOMAIN_ID=\$ROS_DOMAIN_ID"
```

### 3.3 Running Diagnostics (ROS2)

```bash
# Comprehensive diagnostic suite for ROS2
python3 ~/slam-agent/scripts/docker_diagnostics_ros2.py
```

---

## 4. Common ROS2 Operations

### 4.1 Access Container Shell

```bash
# Interactive bash shell
docker compose exec slam_launch bash

# Or non-interactive for scripts
docker compose exec -T slam_launch bash -c "command here"
```

### 4.2 View ROS Topics (ROS2)

```bash
# List all topics with type
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list -t"

# Monitor specific topic (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /Odometry"

# Topic publishing rate (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /ouster/points"

# Topic info (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic info /Odometry"
```

### 4.3 Inspect Transform Tree (ROS2)

```bash
# View TF structure (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 run tf2_tools view_frames.py"

# Check specific transform (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 run tf2_ros tf2_echo map camera_init"
```

### 4.4 Check Flight Controller Connection (ROS2)

```bash
# MAVROS system status (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic echo /mavros/state --once"

# Vision odometry input to FC (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 topic info /mavros/vision_pose/pose"
```

### 4.5 Record Flight Data (ROS2 Bag)

```bash
# Start recording all topics (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   cd /colcon_ws/bags && \
   ros2 bag record -a &"

# Or record specific topics (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   cd /colcon_ws/bags && \
   ros2 bag record /Odometry /ouster/points /mavros/vision_pose/pose"

# Play bag file (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   cd /colcon_ws/bags && \
   ros2 bag play flight_2026-02-08"
```

---

## 5. Configuration Management (ROS2)

### 5.1 ROS Domain ID

ROS2 uses Domain IDs instead of ROS Master:

```bash
# Check current Domain ID
docker compose exec -T slam_launch bash -c "echo \$ROS_DOMAIN_ID"

# Set different Domain ID (in .env)
echo "ROS_DOMAIN_ID=1" >> .env
docker compose restart slam_launch

# Multiple systems on same network:
# System 1: ROS_DOMAIN_ID=0
# System 2: ROS_DOMAIN_ID=1
# (They won't communicate across domains)
```

### 5.2 Modify SLAM Parameters (ROS2)

Parameters are now in YAML files:

```bash
# Edit config (same as ROS1)
nano ~/slam_ws/src/orin_slam_integration/config/fast_lio_config.yaml

# Restart just the SLAM node (ROS2)
docker compose restart slam_launch
```

### 5.3 Deploy Flight Controller Parameters (ROS2)

```bash
# Note: MAVROS is ROS2 compatible
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 service call /mavros/param/set \
   mavros_msgs/srv/ParamSet \
   '{param_id: EK3_ENABLE, value: {real: 1.0}}'"
```

---

## 6. Troubleshooting (ROS2 Specific)

### 6.1 Nodes Not Communicating

ROS2 uses DDS. Check Domain ID:

```bash
# Verify all nodes use same Domain ID
docker compose exec -T slam_launch bash -c \
  "for pid in \$(pgrep -f 'ros2'); do grep -h ROS_DOMAIN_ID /proc/\$pid/environ 2>/dev/null; done | sort -u"
```

### 6.2 Topic Not Visible

```bash
# List all topics with details (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list -a"

# Check topic info
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic info /topic_name"
```

### 6.3 Launch File Errors

ROS2 launch files are Python, not XML:

```bash
# Check launch file syntax
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   python3 -m py_compile /colcon_ws/src/orin_slam_integration/launch/master.py"
```

---

## 7. Comparison: ROS1 vs ROS2 Runbook

### Topics Command Differences

**ROS1**:
```bash
rostopic list
rostopic echo /topic
rostopic hz /topic
rostopic info /topic
```

**ROS2**:
```bash
ros2 topic list
ros2 topic echo /topic
ros2 topic hz /topic
ros2 topic info /topic
```

### Nodes Command Differences

**ROS1**:
```bash
rosnode list
rosnode info /node_name
rosnode kill /node_name
```

**ROS2**:
```bash
ros2 node list
ros2 node info /node_name
# (No direct kill command - use systemctl or docker)
```

### Launch Differences

**ROS1**:
```bash
roslaunch package_name launch_file.launch arg:=value
```

**ROS2**:
```bash
ros2 launch package_name launch_file.py arg:=value
```

---

## 8. Pre-Flight Checklist (ROS2)

Before autonomous flight:

- [ ] Container started: `docker compose ps`
- [ ] ROS Domain ID correct: `echo $ROS_DOMAIN_ID`
- [ ] LiDAR data flowing: `ros2 topic hz /ouster/points`
- [ ] SLAM odometry publishing: `ros2 topic hz /Odometry`
- [ ] Vision odometry to FC: `ros2 topic info /mavros/vision_pose/pose`
- [ ] Flight controller connected: `ros2 topic echo /mavros/state --once`
- [ ] Geofence enabled: `EK3_ENABLE=1`
- [ ] RC transmitter paired
- [ ] Battery fully charged
- [ ] Propellers secure
- [ ] GPS disabled (GPS-denied mode)
- [ ] EKF3 configured for vision

---

## 9. Flight Operations (ROS2)

### 9.1 During Flight - Monitor

**Terminal 1: Logs**
```bash
docker compose logs -f slam_launch 2>&1 | grep -E "error|ERROR"
```

**Terminal 2: Diagnostics Loop (ROS2)**
```bash
while true; do
  docker compose exec -T slam_launch bash -c \
    "source /opt/ros/humble/setup.bash && \
     echo 'Odom:' && ros2 topic info /Odometry 2>&1 | head -1"
  sleep 5
done
```

**Terminal 3: Record Data (ROS2)**
```bash
cd ~/slam_ws/bags
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 bag record -a -o flight_2026-02-08"
```

### 9.2 After Flight - Analysis (ROS2)

```bash
# Copy flight data from container
docker compose cp slam_launch:/colcon_ws/bags/flight_2026-02-08 .

# Analyze bag file (ROS2)
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 bag info flight_2026-02-08"
```

---

## 10. Switching Between ROS Versions

### Quick Switch Script

```bash
# Switch to ROS2 Humble
./select_ros_version.sh switch humble
./select_ros_version.sh start

# Switch back to ROS1 Noetic
./select_ros_version.sh switch noetic
./select_ros_version.sh start
```

### Manual Switch

```bash
# Edit .env
echo "ROS_VERSION=humble" > .env

# Rebuild if needed
docker compose -f docker-compose.multi.yml build

# Start
docker compose -f docker-compose.multi.yml up -d slam_launch
```

---

## 11. Reference

### Image Info (ROS2)
- **Size**: ~4.5 GB (estimated)
- **Base**: Ubuntu 22.04 (jammy)
- **ROS**: Humble
- **Build System**: colcon
- **Ceres**: 2.1.0
- **GTSAM**: 4.0.3

### Key Directories (In Container)
- `/colcon_ws/install` - Built binaries & scripts (ROS2)
- `/colcon_ws/src` - Source code
- `/colcon_ws/data` - Flight data
- `/colcon_ws/bags` - ROS2 bag recordings

### ROS2 Specific Settings

```bash
# Check ROS2 middleware (DDS)
docker compose exec -T slam_launch bash -c \
  "ros2 daemon status"

# View ROS2 graph
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/humble/setup.bash && rqt_graph"
```

---

**Status**: ✅ Production Ready (ROS2 Humble)
**Tested**: 2026-02-08
**Comparison Guide**: See DOCKER_OPERATIONS_RUNBOOK.md for ROS1 version
