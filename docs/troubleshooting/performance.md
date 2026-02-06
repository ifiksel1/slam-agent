# Troubleshooting: Performance & Real-Time Issues

## Symptoms
- SLAM < 10 Hz, dropped frames
- High CPU (> 80%), system lag
- Point cloud delayed or stuttering

## Fixes

### 1. CPU Governor
```bash
# Set to performance mode
sudo cpufreq-set -g performance
# Jetson:
sudo nvpmodel -m 0  # MAX power
sudo jetson_clocks   # Lock max frequency
```

### 2. Point Cloud Downsampling
In SLAM config:
- FAST-LIO: `point_filter_num: 4` (keep every 4th point)
- LIO-SAM: `downsampleRate: 2`, `mappingSurfLeafSize: 0.4`
- Reduce horizontal resolution if possible (2048 to 1024)

### 3. ROS Queue Tuning
```xml
<!-- Increase queue sizes for slow subscribers -->
<param name="queue_size" value="10"/>
<!-- Use TCP_NODELAY for low latency -->
<param name="tcp_nodelay" value="true"/>
```

### 4. Thread Priority
```bash
# Run SLAM with real-time priority
sudo chrt -f 50 roslaunch PACKAGE slam.launch
```

### 5. Resource Monitoring
```bash
htop                          # CPU/RAM
nvidia-smi                    # GPU (Jetson: tegrastats)
rostopic hz /slam/odometry    # SLAM rate
rostopic delay /slam/odometry # Processing latency
```

### 6. Reduce Map Size
- Limit voxel grid size
- Reduce keyframe density
- Disable loop closure if not needed (saves CPU)
