# Docker SLAM System Execution Guide

Complete SLAM system with FAST-LIO2, STD loop-closure, Ouster OS1-64, and ArduPilot 4.6.2 integration.

## Components

- **FAST-LIO2**: Fast LiDAR-Inertial Odometry (10 Hz)
- **STD**: Stable Triangle Descriptor for loop-closure detection
- **Ouster OS1-64**: 3D LiDAR sensor driver
- **MAVRos**: MAVLink communication with ArduPilot 4.6.2
- **GTSAM**: Pose graph optimization
- **vision_to_mavros**: SLAM odometry → ArduPilot vision position bridge

## Quick Setup

```bash
# 1. Run setup script (creates all Docker files in ~/slam_ws)
bash /home/dev/slam-agent/DOCKER_SLAM_SETUP.sh

# 2. Navigate to slam_ws
cd /home/dev/slam_ws

# 3. Build the Docker image (takes 10-15 minutes)
docker-compose build slam-system

# 4. Start the container
docker-compose up -d slam-system

# 5. Enter the container
docker exec -it slam-ouster-fastlio-std bash
```

## Inside the Container

```bash
# Verify installation
rostopic list
# Should show:
# /Odometry (from FAST-LIO2)
# /ouster/points (LiDAR point cloud)
# /ouster/imu (IMU data)
# /vision_estimate (from vision_to_mavros)
# /mavros/* (MAVRos topics)

# Test SLAM system with loop closure
roslaunch fast_lio mapping_ouster64_docker.launch

# Or with loop closure enabled (STD detector running)
# roslaunch fast_lio slam_with_loop_closure.launch ouster_ip:=169.254.56.220 fc_ip:=192.168.1.100
```

## Configuration Files

Located in `/home/dev/slam_ws/config/`:

- `ouster64.yaml` - FAST-LIO2 configuration (topic names, sensor params)
- `std_config.yaml` - STD loop-closure parameters

Located in `/home/dev/slam_ws/launch/`:

- `mapping_ouster64_docker.launch` - Basic SLAM (odometry only)
- `slam_with_loop_closure.launch` - SLAM with loop-closure detection and ArduPilot bridge

## Data Flow

```
Ouster LiDAR (169.254.56.220)
         ↓
    /ouster/points
         ↓
    FAST-LIO2 (odometry @ 10Hz)
         ↓
    /Odometry
    ↙           ↘
  STD            vision_to_mavros
 (loop            ↓
  closure)      /vision_estimate
    ↓             ↓
  GTSAM ------→ MAVRos
  (pose graph)    ↓
              ArduPilot 4.6.2 (FC)
```

## Environment Variables

```bash
# In docker-compose.yml or when running:
OUSTER_IP=169.254.56.220          # Ouster sensor IP
FC_IP=192.168.1.100               # Flight controller IP
ROS_HOSTNAME=localhost
ROS_MASTER_URI=http://localhost:11311
```

## Network Configuration

The Docker container uses **host network mode** (`network_mode: host`) to:
- Access Ouster LiDAR on the local network
- Connect to ArduPilot flight controller
- Communicate with roscore on host

## Troubleshooting

### No LiDAR data
```bash
# Inside container
rostopic info /ouster/points
# Should show publisher: /ouster_driver_node

# Check Ouster connectivity
ping 169.254.56.220
```

### No SLAM odometry
```bash
# Check if FAST-LIO2 is running and receiving data
rostopic hz /Odometry
rostopic hz /ouster/points
```

### MAVRos connection issues
```bash
# Verify FC_IP and network connectivity
ping ${FC_IP}

# Check MAVRos connection status
rostopic echo /mavros/state
```

### Loop-closure detection issues
```bash
# Monitor STD detector
rostopic echo /std/loop_constraints

# Check pose graph optimization
rosparam get /gtsam
```

## Next Steps

1. **Configure Ouster IP**: Update OUSTER_IP in docker-compose.yml if different
2. **Configure FC IP**: Update FC_IP for ArduPilot 4.6.2 flight controller
3. **Mount real workspace** (optional): Add volume mount for `/root/slam_ws/maps` to save maps
4. **Run rosbag tests**: Record and replay to test without live hardware
5. **Integrate with Gazebo** (optional): Add Gazebo simulation

## Build Customization

Edit `/home/dev/slam_ws/Dockerfile` to:
- Change ROS distro (Noetic → Humble)
- Add different LiDAR drivers (Livox, Hesai, Velodyne)
- Add different SLAM algorithms (LIO-SAM, R3LIVE, MULLS)
- Custom loop-closure methods

Then rebuild:
```bash
docker-compose build --no-cache slam-system
```

## Maintenance

```bash
# View logs
docker logs slam-ouster-fastlio-std

# Monitor resources
docker stats slam-ouster-fastlio-std

# Stop container
docker-compose down

# Remove image (to rebuild)
docker image rm slam-system:latest
docker-compose build slam-system
```

## Hardware Requirements

- **CPU**: 4+ cores (recommended 8)
- **RAM**: 4GB minimum (8GB recommended)
- **Disk**: 20GB for image + workspace
- **Network**: Gigabit Ethernet for LiDAR + FC connection

## Source References

- [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO_SLAM)
- [STD Loop-Closure GitHub](https://github.com/hku-mars/STD)
- [Ouster ROS Driver](https://github.com/ouster-lidar/ouster-ros)
- [MAVRos Documentation](http://wiki.ros.org/mavros)
- [ArduPilot Docs](https://ardupilot.org/)
