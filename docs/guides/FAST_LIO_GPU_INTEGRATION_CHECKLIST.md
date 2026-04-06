# FAST_LIO_GPU Integration Checklist

**System:** Jetson Orin NX + Ouster OS1-64 + FAST_LIO_GPU + ArduPilot Cube Orange
**ROS Version:** ROS 2 Humble
**Status:** Phase 2 Validated ✅

---

## Phase 3: Configuration Generation

### URDF/Robot Description
- [ ] Create base_link → ouster_link transform
- [ ] Add Ouster OS1-64 physical offset (measure from drone CG)
- [ ] Define IMU frame (ouster_imu_link)
- [ ] Verify right-hand coordinate system (X forward, Z up)

### FAST_LIO_GPU Config YAML
- [ ] Set `lid_topic: "/ouster/points"`
- [ ] Set `imu_topic: "/ouster/imu"`
- [ ] Configure point cloud dimensions (64 channels)
- [ ] Set scan rate (10 Hz or 20 Hz - match Ouster config)
- [ ] Enable CUDA acceleration: `use_cuda: true`
- [ ] Define extrinsic calibration (IMU → LiDAR)
- [ ] Set map parameters (voxel size, max range)

### ROS 2 Launch File
- [ ] Use Python launch file format (ROS 2 standard)
- [ ] Start ouster-ros driver node
- [ ] Start fast_lio node with config path
- [ ] Start static_transform_publisher for URDF
- [ ] Include MAVROS launch file
- [ ] Set namespace (optional): `/slam`

### MAVROS Configuration
- [ ] Set FCU URL: `/dev/ttyUSB0:921600` (Cube Orange serial)
- [ ] Configure vision pose topic: `/mavros/vision_pose/pose`
- [ ] Create odometry → vision_pose bridge node
- [ ] Enable EKF3 vision messages in ArduPilot params

---

## Phase 4: Installation (Critical Tests)

### Pre-Installation
- [ ] Verify JetPack 6.2+ installed (`jetson_release`)
- [ ] Confirm CUDA 12.6 available (`nvcc --version`)
- [ ] Check nvidia-docker runtime (`docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi`)
- [ ] Verify Ouster connectivity (ping 169.254.56.220)

### Docker Setup
- [ ] Use base image: `dustynv/ros:humble-desktop-l4t-r35.4.1` (or latest)
- [ ] Enable GPU in docker-compose:
  ```yaml
  runtime: nvidia
  environment:
    - NVIDIA_VISIBLE_DEVICES=all
  ```
- [ ] Mount workspace: `/home/dev/slam_ws:/workspace`
- [ ] Host network mode: `network_mode: host`
- [ ] Device passthrough: `/dev/ttyUSB0:/dev/ttyUSB0`

### Build Process (Inside Container)

**Step 1: Workspace Setup**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Clone Repositories**
```bash
# FAST_LIO_GPU
git clone https://github.com/OmerMersin/FAST_LIO_GPU.git

# Ouster ROS (check if pre-installed in dustynv image)
sudo apt install ros-humble-ouster-ros

# MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

**Step 3: Install Dependencies**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Step 4: Build with CUDA**
```bash
colcon build --packages-select fast_lio \
  --cmake-args \
    -DFASTLIO_USE_CUDA=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-march=native -O3" \
    -DCUDA_ARCH_BIN="8.7"
```

**Critical Check Points:**
- [ ] ✅ Compilation completes without CUDA errors
- [ ] ✅ Check `install/fast_lio/lib/fast_lio/fast_lio_node` exists
- [ ] ✅ Run `nvidia-smi` - GPU visible inside container

### First Boot Tests

**Test 1: Ouster Driver**
```bash
ros2 launch ouster_ros driver.launch.xml \
  sensor_hostname:=169.254.56.220 \
  lidar_mode:=1024x10  # or 2048x10 for 10 Hz
```
- [ ] ✅ No crash (exit code 255)
- [ ] ✅ Topics appear: `/ouster/points`, `/ouster/imu`
- [ ] ✅ Data flowing: `ros2 topic hz /ouster/points` shows 10 Hz

**Troubleshooting:** If nodelets crash:
```bash
# Fallback: Use pre-built NVIDIA packages
sudo apt install ros-humble-ouster-ros --reinstall

# Or: Rebuild from source with ARM64 flags
cd ~/ros2_ws/src
git clone https://github.com/ouster-lidar/ouster-ros.git
cd ../
colcon build --packages-select ouster_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Test 2: FAST_LIO_GPU Launch**
```bash
ros2 launch fast_lio mapping.launch.py
```
- [ ] ✅ Node starts without errors
- [ ] ✅ Subscribes to `/ouster/points` and `/ouster/imu`
- [ ] ✅ Publishes `/fast_lio/odometry` at 10-20 Hz
- [ ] ✅ GPU utilization visible in `nvidia-smi` (should be >20%)

**Test 3: MAVROS Serial Connection**
```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
```
- [ ] ✅ Connection established (check `ros2 topic echo /mavros/state`)
- [ ] ✅ `connected: true` in state message
- [ ] ✅ Heartbeat received from Cube Orange

---

## Phase 5: Integration Testing

### End-to-End Data Flow
```bash
# Terminal 1: Launch full system
ros2 launch slam_integration slam_system.launch.py

# Terminal 2: Monitor frequencies
ros2 topic hz /ouster/points        # Should be 10 Hz
ros2 topic hz /fast_lio/odometry    # Should be 10-20 Hz
ros2 topic hz /mavros/vision_pose/pose  # Should match odometry

# Terminal 3: Check transforms
ros2 run tf2_ros tf2_echo base_link ouster_link
ros2 run tf2_ros tf2_echo map base_link
```

### Performance Benchmarks
- [ ] **Odometry Frequency:** >= 10 Hz sustained (5 min test)
  ```bash
  ros2 topic hz /fast_lio/odometry --window 300
  ```
- [ ] **GPU Memory:** < 6 GB (2 GB headroom)
  ```bash
  watch -n 1 nvidia-smi
  ```
- [ ] **CPU Usage:** < 80% average (check for thermal throttling)
  ```bash
  htop
  ```
- [ ] **Latency:** Sensor to FC < 200 ms
  ```bash
  ros2 topic delay /ouster/points /mavros/vision_pose/pose
  ```

### Functional Tests
- [ ] **Static Environment:** SLAM maintains stable pose (drift < 0.1 m over 1 min)
- [ ] **Dynamic Movement:** Smooth odometry during translation/rotation
- [ ] **Feature-Rich Area:** Map builds correctly with walls/objects
- [ ] **Feature-Poor Area:** SLAM degrades gracefully (warnings, no crash)
- [ ] **Loop Closure:** (If enabled) Detects and corrects drift

### ArduPilot Integration
- [ ] **Vision Pose Received:** Check Mission Planner/QGC for vision data
- [ ] **EKF3 Fusion:** ArduPilot uses vision for position estimate
- [ ] **Coordinate Frame Match:** Drone moves in expected direction
- [ ] **Failsafe:** SLAM failure triggers position hold (not crash)

---

## Phase 6: Troubleshooting Guide

### Issue: Ouster Nodelets Crash (Exit 255)
**Symptom:** Driver starts but immediately exits
**Solution:**
1. Reinstall: `sudo apt install ros-humble-ouster-ros --reinstall`
2. Rebuild from source with ARM64 optimizations
3. Fallback: Use different driver version from ultraviewdev_ws

### Issue: FAST_LIO_GPU No GPU Acceleration
**Symptom:** CPU usage high, GPU usage 0%
**Solution:**
1. Check CUDA flag: `colcon build ... -DFASTLIO_USE_CUDA=ON`
2. Verify CUDA visible: `nvidia-smi` inside container
3. Check CMake output for CUDA detection errors

### Issue: "Failed to find match for field 'time'"
**Symptom:** FAST_LIO_GPU warns about missing timestamps
**Solution:**
1. Verify Ouster driver publishes `PointCloud2` with `time` field
2. Check Ouster firmware version (update if old)
3. Enable per-point timestamps in Ouster config

### Issue: Coordinate Frame Mismatch
**Symptom:** Drone flies in wrong direction when following SLAM
**Solution:**
1. Verify URDF extrinsics (LiDAR rotation relative to drone)
2. Check MAVROS frame_id settings (should match SLAM output)
3. Test static transform with `tf2_echo`

### Issue: Low Odometry Frequency (< 10 Hz)
**Symptom:** Sluggish performance, high latency
**Solution:**
1. Check GPU memory: May be thrashing if >8GB used
2. Reduce point cloud density: Lower scan rate or downsample
3. Disable map visualization: Save GPU/CPU cycles

---

## Known Good Configuration (Reference)

**System Profile:** jetson_orin_nx-ouster_os1_64-fast_lio2_gpu-ardupilot-humble

**Hardware:**
- Jetson Orin NX 8GB (JetPack 6.2, CUDA 12.6)
- Ouster OS1-64 (10 Hz, 1024x10 mode)
- Cube Orange (ArduPilot Copter 4.5+)
- Serial: /dev/ttyUSB0 @ 921600 baud

**Software Stack:**
- ROS 2 Humble (Ubuntu 22.04 ARM64)
- FAST_LIO_GPU (CUDA enabled, compute 8.7)
- ouster-ros 0.13.9+ (ARM64 pre-built)
- MAVROS 2.6.0+

**Expected Performance:**
- Odometry: 15-20 Hz sustained
- GPU Memory: 4-5 GB
- CPU Usage: 60-70% average
- Latency: 80-120 ms (sensor to FC)

---

## Quick Commands Reference

### Build Commands
```bash
# Full rebuild with CUDA
cd ~/ros2_ws
colcon build --cmake-args -DFASTLIO_USE_CUDA=ON -DCMAKE_BUILD_TYPE=Release

# CPU-only fallback
colcon build --cmake-args -DFASTLIO_USE_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
```

### Launch Commands
```bash
# Full system
ros2 launch slam_integration slam_system.launch.py

# Individual components
ros2 launch ouster_ros driver.launch.xml sensor_hostname:=169.254.56.220
ros2 launch fast_lio mapping.launch.py
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:921600
```

### Diagnostic Commands
```bash
# Topic monitoring
ros2 topic list | grep -E '(ouster|fast_lio|mavros)'
ros2 topic hz /fast_lio/odometry
ros2 topic echo /mavros/state --no-arr

# Transform tree
ros2 run tf2_tools view_frames

# GPU monitoring
nvidia-smi -l 1

# ROS 2 node graph
rqt_graph
```

---

**Generated:** 2026-02-11
**Status:** Phase 2 Complete ✅ | Ready for Phase 3
**Confidence:** HIGH (95%)
