# SLAM System Docker Setup

Complete Docker containerization for the SLAM system using Ouster OS2-64 LiDAR and FAST-LIO2 on ROS Noetic.

## Overview

This Docker setup provides:
- **Isolated environment**: No conflicts with host ROS installation
- **Reproducible builds**: Same environment across Jetson and x86_64
- **Easy deployment**: Single command to build and run
- **Network access**: Host networking for Ouster sensor communication
- **Volume mounts**: Access logs and configs without rebuilding

## Architecture

```
slam-agent/
├── Dockerfile                    # Main image definition
├── docker-compose.yml            # Multi-service orchestration
├── config/
│   └── ouster64.yaml            # SLAM configuration (corrected topics)
├── launch/
│   └── mapping_ouster64_docker.launch  # Combined Ouster + SLAM launch
├── scripts/
│   ├── docker_entrypoint.sh     # Container initialization
│   └── preflight_check_docker.sh # In-container verification
├── logs/                         # Mounted for ROS logs
└── workspace/                    # Mounted for development
```

## Quick Start

### 1. Build the Image

```bash
cd /home/dev/slam-agent

# Build for your architecture (auto-detects ARM64/x86_64)
docker compose build

# Or build with progress output
docker compose build --progress=plain
```

Build time: 10-20 minutes on first build (depending on CPU)

### 2. Run Preflight Check

```bash
# Start container in interactive mode
docker compose run --rm slam-system

# Inside container, run preflight check
/root/preflight_check.sh
```

### 3. Run SLAM System

#### Option A: Interactive Mode (Recommended for Testing)

```bash
# Start container with shell
docker compose run --rm slam-system

# Inside container:
source /root/slam_ws/devel/setup.bash

# Start ROS master
roscore &

# In another terminal, start Ouster + SLAM
roslaunch fast_lio mapping_ouster64_docker.launch
```

#### Option B: Automatic Startup

```bash
# Start with automatic SLAM launch
docker compose --profile autostart up slam-autostart
```

#### Option C: Ouster Driver Only

```bash
# Run just the Ouster driver
docker compose --profile driver-only up ouster-driver
```

## Network Configuration

### Connecting to Ouster Sensor

The Ouster sensor must be on the same network as the Docker host.

#### Default Configuration
- **Ouster IP**: 169.254.56.220
- **Network Mode**: host (container uses host network stack)
- **Ports**: 7502 (LiDAR), 7503 (IMU)

#### Network Setup on Jetson

```bash
# Configure eth0 for Ouster sensor
sudo ip addr add 169.254.56.1/16 dev eth0

# Verify connectivity
ping -c 3 169.254.56.220

# Check link status
ip link show eth0
```

#### Verify from Container

```bash
# Enter running container
docker exec -it slam-ouster-fastlio bash

# Test Ouster connectivity
ping -c 3 169.254.56.220

# Check network interfaces
ip addr show
```

### Custom Ouster IP

If your Ouster has a different IP:

```bash
# Set environment variable
export OUSTER_IP=192.168.1.100

# Run with custom IP
docker compose run --rm -e OUSTER_IP=192.168.1.100 slam-system
```

Or edit `docker-compose.yml`:

```yaml
environment:
  - OUSTER_IP=192.168.1.100
```

## Configuration

### SLAM Configuration

Edit `/home/dev/slam-agent/config/ouster64.yaml`:

```yaml
common:
    lid_topic:  "/ouster/points"  # Ouster point cloud topic
    imu_topic:  "/ouster/imu"     # Ouster IMU topic
    time_sync_en: false

preprocess:
    lidar_type: 3                  # 3 = Ouster
    scan_line: 64
    blind: 4

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    # ... other parameters
```

Changes take effect after container rebuild:
```bash
docker compose build
```

### Launch Configuration

Edit `/home/dev/slam-agent/launch/mapping_ouster64_docker.launch` to:
- Change Ouster sensor IP
- Enable/disable RViz
- Adjust lidar mode (512x10, 1024x10, 1024x20, etc.)

```xml
<arg name="sensor_hostname" default="169.254.56.220"/>
<arg name="rviz" default="false"/>
<arg name="lidar_mode" value="1024x10"/>
```

## Testing and Validation

### 1. Container Build Verification

```bash
# Check image was created
docker images | grep slam-system

# Should show:
# slam-system   latest   [IMAGE_ID]   [SIZE]   [TIME]
```

### 2. Package Build Verification

```bash
# Start container
docker compose run --rm slam-system

# Check packages are built
ls -la /root/slam_ws/devel/lib/ | grep -E "(fast_lio|ouster_ros)"

# Should see:
# drwxr-xr-x  fast_lio
# drwxr-xr-x  ouster_ros
```

### 3. Run Preflight Check

```bash
# Inside container
/root/preflight_check.sh

# Expected output:
# [1] NETWORK CONNECTIVITY
#   ✓ Ouster sensor reachable (169.254.56.220)
# [2] ROS INFRASTRUCTURE
#   ✓ ROS Master running
# [3] PACKAGE INSTALLATION
#   ✓ FAST-LIO package built
#   ✓ Ouster ROS driver built
# [4] CONFIGURATION VERIFICATION
#   ✓ SLAM config file exists
#   ✓ SLAM config points to /ouster/points
#   ✓ SLAM config points to /ouster/imu
# [5] LAUNCH FILES
#   ✓ Docker launch file available
#
# ✅ ALL CRITICAL CHECKS PASSED - CONTAINER READY
```

### 4. Test Ouster Connection

```bash
# Inside container with ROS master running
roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.56.220 viz:=false

# In another terminal/container
rostopic list | grep ouster

# Should show:
# /ouster/imu
# /ouster/points
# /ouster/lidar_packets
# ...

# Verify data flow
rostopic hz /ouster/points
# Should show ~10 Hz for 1024x10 mode
```

### 5. Test SLAM Initialization

```bash
# Inside container, launch full system
roslaunch fast_lio mapping_ouster64_docker.launch

# In another terminal, verify SLAM output
rostopic list | grep -E "(Odometry|path|map)"

# Should show:
# /Odometry
# /path
# /cloud_registered
# ...

# Wait 20-30 seconds for initialization
rostopic hz /Odometry
# Should show data once SLAM has initialized
```

### 6. Monitor Resources

```bash
# From host machine
docker stats slam-ouster-fastlio

# Shows CPU, Memory, Network I/O in real-time
```

## Troubleshooting

### Issue: Cannot reach Ouster sensor

**Symptoms:**
```
⚠ Warning: Cannot reach Ouster sensor at 169.254.56.220
```

**Solution:**
```bash
# Check host network configuration
ip addr show eth0

# Add route if needed
sudo ip addr add 169.254.56.1/16 dev eth0

# Test from host
ping 169.254.56.220

# Restart container
docker compose restart slam-system
```

### Issue: ROS Master not running

**Symptoms:**
```
✗ ROS Master running
ERROR: Unable to communicate with master!
```

**Solution:**
```bash
# Inside container
roscore &
sleep 3

# Verify
rostopic list
```

### Issue: Build fails with CMake errors

**Symptoms:**
```
CMake Error: Could not find googletest
```

**Solution:**
```bash
# Rebuild with --no-cache to force fresh install
docker compose build --no-cache
```

### Issue: Ouster topics not appearing

**Symptoms:**
```
rostopic list | grep ouster
# (no output)
```

**Solution:**
```bash
# Check Ouster driver logs
rosnode list | grep ouster

# Restart Ouster driver
rosnode kill /ouster/os_node
roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.56.220
```

### Issue: SLAM not producing odometry

**Symptoms:**
```
rostopic hz /Odometry
# average rate: 0.000
```

**Solution:**
```bash
# SLAM needs 20-30 seconds to initialize
# Wait and check again

# Verify input data is flowing
rostopic hz /ouster/points
rostopic hz /ouster/imu

# Check SLAM node logs
rosnode info /laserMapping
```

### Issue: Container uses too much memory

**Symptoms:**
```
docker stats shows > 6GB memory usage
```

**Solution:**
Edit `docker-compose.yml`:
```yaml
deploy:
  resources:
    limits:
      memory: 4G  # Reduce from 6G
```

Or disable dense point cloud publishing in config:
```yaml
publish:
    dense_publish_en: false
```

## Advanced Usage

### Using with X11 for RViz

```bash
# Allow container to access X server
xhost +local:docker

# Run with RViz enabled
docker compose run --rm -e RVIZ_ENABLE=true -e DISPLAY=$DISPLAY slam-system

# Inside container
roslaunch fast_lio mapping_ouster64_docker.launch rviz:=true
```

### Development Workflow

Mount your development workspace:

```bash
# Edit docker-compose.yml to add volume
volumes:
  - /path/to/your/code:/root/slam_ws/src/custom_package:rw

# Rebuild inside container
docker compose run --rm slam-system bash -c "cd /root/slam_ws && catkin build custom_package"
```

### Multi-Container Setup

Run ROS master separately:

```yaml
# Add to docker-compose.yml
roscore:
  extends: slam-system
  container_name: roscore
  command: roscore

slam-node:
  extends: slam-system
  container_name: slam-node
  environment:
    - ROS_MASTER_URI=http://roscore:11311
  depends_on:
    - roscore
```

### Saving Logs

Logs are automatically saved to `./logs/` directory:

```bash
# View container logs
docker compose logs slam-system

# View ROS logs
ls -la logs/

# Copy logs from container
docker cp slam-ouster-fastlio:/root/.ros/log/ ./logs/ros_logs/
```

## Performance Considerations

### Jetson Optimization

For Jetson AGX Orin/Xavier:

1. **Enable MAX-N power mode:**
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

2. **Reduce CPU cores in docker-compose.yml:**
   ```yaml
   deploy:
     resources:
       limits:
         cpus: '6'  # Use 6 of 8 cores
   ```

3. **Disable RViz in production:**
   ```bash
   docker compose run --rm -e RVIZ_ENABLE=false slam-system
   ```

### Memory Management

Monitor memory usage:
```bash
# Real-time monitoring
docker stats slam-ouster-fastlio

# Set memory limits
docker compose run --rm --memory=4g slam-system
```

## Production Deployment

### Systemd Service

Create `/etc/systemd/system/slam-docker.service`:

```ini
[Unit]
Description=SLAM Docker Container
After=docker.service
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=/home/dev/slam-agent
ExecStart=/usr/bin/docker compose --profile autostart up slam-autostart
ExecStop=/usr/bin/docker compose --profile autostart down
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable slam-docker
sudo systemctl start slam-docker
sudo systemctl status slam-docker
```

### Automatic Restart on Failure

The docker-compose.yml already includes:
```yaml
restart: unless-stopped
```

This ensures the container restarts automatically after:
- Container crashes
- Host reboot
- Docker daemon restart

## Cleanup

```bash
# Stop and remove containers
docker compose down

# Remove images
docker rmi slam-system:latest

# Clean all Docker data (WARNING: removes all containers/images)
docker system prune -a
```

## Support and Troubleshooting

For issues:
1. Check preflight results: `/root/preflight_check.sh`
2. Review container logs: `docker compose logs`
3. Verify network: `ping 169.254.56.220`
4. Check ROS topics: `rostopic list`
5. Monitor resources: `docker stats`

## Next Steps

- **Test with hardware**: Connect Ouster sensor and run full system test
- **Integrate with ArduPilot**: Add vision_to_mavros bridge
- **Add data recording**: Enable rosbag recording for flights
- **Deploy to production**: Set up systemd service for automatic startup

## License

This Docker setup is part of the SLAM Agent project. See main README for licensing information.
