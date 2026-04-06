#!/bin/bash
# Setup complete SLAM Docker system in ~/slam_ws
# Includes: FAST-LIO2, STD loop-closure, Ouster driver, MAVRos, vision_to_mavros

set -e

echo "================================================"
echo "Setting up Complete SLAM Docker System"
echo "================================================"

# Take ownership of slam_ws
echo "[1] Fixing permissions on ~/slam_ws..."
sudo chown -R $USER:$USER /home/dev/slam_ws

# Copy Docker configuration
echo "[2] Copying Docker configuration files..."
cp /home/dev/slam-agent/Dockerfile /home/dev/slam_ws/Dockerfile
cp /home/dev/slam-agent/docker-compose.yml /home/dev/slam_ws/docker-compose.yml

# Create directories if they don't exist
mkdir -p /home/dev/slam_ws/config
mkdir -p /home/dev/slam_ws/launch
mkdir -p /home/dev/slam_ws/scripts

# Copy config files
echo "[3] Copying configuration files..."
cp /home/dev/slam-agent/config/ouster64.yaml /home/dev/slam_ws/config/
cp /home/dev/slam-agent/launch/mapping_ouster64_docker.launch /home/dev/slam_ws/launch/

# Copy scripts
echo "[4] Copying scripts..."
cp /home/dev/slam-agent/scripts/docker_entrypoint.sh /home/dev/slam_ws/scripts/
cp /home/dev/slam-agent/scripts/preflight_check_docker.sh /home/dev/slam_ws/scripts/
chmod +x /home/dev/slam_ws/scripts/*.sh

# Create STD config if it doesn't exist
if [ ! -f /home/dev/slam_ws/config/std_config.yaml ]; then
    echo "[5] Creating STD configuration..."
    cat > /home/dev/slam_ws/config/std_config.yaml << 'EOF'
# STD (Stable Triangle Descriptor) Configuration
# Loop closure detection for SLAM

# STD parameters
descriptor_radius: 5.0  # meters
key_frame_distance: 2.0  # meters
loop_closure_threshold: 0.9
min_loop_candidates: 3
skip_frames: 1

# LiDAR parameters (for Ouster OS1-64)
lidar_topic: "/ouster/points"
imu_topic: "/ouster/imu"
num_threads: 4

# Pose graph optimization
enable_pose_graph: true
g2o_solver_type: "lm_var"
max_iterations: 10
EOF
fi

# Create loop-closure launch file if it doesn't exist
if [ ! -f /home/dev/slam_ws/launch/slam_with_loop_closure.launch ]; then
    echo "[6] Creating loop-closure launch file..."
    cat > /home/dev/slam_ws/launch/slam_with_loop_closure.launch << 'EOF'
<launch>
    <!-- SLAM System with Loop Closure Detection -->
    <!-- Combines FAST-LIO2 odometry with STD loop closure -->

    <!-- Ouster LiDAR Driver -->
    <include file="$(find ouster_ros)/launch/sensor.launch">
        <arg name="sensor_hostname" value="$(arg ouster_ip)" />
        <arg name="udp_dest" value="localhost" />
        <arg name="lidar_mode" value="1024x10" />
        <arg name="viz" value="false" />
    </include>

    <!-- FAST-LIO2 Odometry -->
    <node pkg="fast_lio" name="fastlio_node" type="fastlio_node">
        <param name="common/lid_topic" value="/ouster/points" />
        <param name="common/imu_topic" value="/ouster/imu" />
        <param name="common/time_sync_en" value="false" />
        <param name="common/time_offset_lidar_to_imu" value="0.0" />
        <param name="common/rotate_around_z" value="false" />
        <param name="common/extrinsic_T" value="[ 0, 0, 0]" />
        <param name="common/extrinsic_R" value="[1, 0, 0, 0, 1, 0, 0, 0, 1]" />
    </node>

    <!-- STD Loop Closure Detector -->
    <node pkg="std" name="std_node" type="std_node">
        <param name="descriptor_radius" value="5.0" />
        <param name="key_frame_distance" value="2.0" />
        <param name="loop_closure_threshold" value="0.9" />
        <remap from="lidar_cloud" to="/ouster/points" />
        <remap from="odometry" to="/Odometry" />
    </node>

    <!-- TF Broadcaster for map -> odom -> base_link -->
    <node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster"
        args="0 0 0 0 0 0 /map /odom 100" />
    <node pkg="tf" type="static_transform_publisher" name="odom_base_link_broadcaster"
        args="0 0 0 0 0 0 /odom /base_link 100" />

    <!-- MAVRos for ArduPilot 4.6.2 Connection -->
    <node pkg="mavros" type="mavros_node" name="mavros">
        <param name="fcu_url" value="udp://:14550@$(arg fc_ip):14550" />
        <param name="gcs_url" value="" />
        <param name="target_system_id" value="1" />
        <param name="target_component_id" value="1" />
        <param name="fcu_protocol" value="v2.0" />

        <!-- Remap odometry to vision position estimate -->
        <remap from="local_position/pose" to="local_position/pose" />
        <remap from="/Odometry" to="vision_estimate" />
    </node>
</launch>
EOF
fi

# Create docker-compose if it doesn't exist in slam_ws
if [ ! -f /home/dev/slam_ws/docker-compose.yml ]; then
    echo "[7] Creating docker-compose.yml..."
    cat > /home/dev/slam_ws/docker-compose.yml << 'EOF'
version: '3.8'

services:
  slam-system:
    build:
      context: .
      dockerfile: Dockerfile
    image: slam-system:latest
    container_name: slam-ouster-fastlio-std

    # Network configuration for Ouster sensor access
    network_mode: host

    # Privileged mode for hardware access
    privileged: true

    # Environment variables
    environment:
      - ROS_HOSTNAME=localhost
      - ROS_MASTER_URI=http://localhost:11311
      - DISPLAY=${DISPLAY:-:0}
      - OUSTER_IP=${OUSTER_IP:-169.254.56.220}
      - FC_IP=${FC_IP:-192.168.1.100}
      - RVIZ_ENABLE=false

    # Volume mounts
    volumes:
      - ./workspace:/root/slam_ws/src/workspace:rw
      - ./logs:/root/slam_ws/logs:rw
      - ./bags:/root/slam_ws/bags:rw
      - ./maps:/root/slam_ws/maps:rw
      - ./config:/root/slam_ws/config:rw
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

    # Restart policy
    restart: unless-stopped

    # Resource limits
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 8G
        reservations:
          cpus: '2'
          memory: 2G

    # Health check
    healthcheck:
      test: ["CMD", "rostopic", "list"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

    # Command - keep container running
    command: bash -c "source /root/slam_ws/devel/setup.bash && bash"

    # TTY for interactive use
    tty: true
    stdin_open: true

volumes:
  workspace:
  logs:
  bags:
  maps:

networks:
  default:
    name: slam-network
EOF
fi

echo ""
echo "================================================"
echo "Docker Setup Complete!"
echo "================================================"
echo ""
echo "Files created in /home/dev/slam_ws:"
echo "  ✓ Dockerfile"
echo "  ✓ docker-compose.yml"
echo "  ✓ config/ouster64.yaml"
echo "  ✓ config/std_config.yaml"
echo "  ✓ launch/mapping_ouster64_docker.launch"
echo "  ✓ launch/slam_with_loop_closure.launch"
echo "  ✓ scripts/docker_entrypoint.sh"
echo "  ✓ scripts/preflight_check_docker.sh"
echo ""
echo "Next steps:"
echo "  1. cd /home/dev/slam_ws"
echo "  2. docker-compose build slam-system  (takes 10-15 minutes)"
echo "  3. docker-compose up -d slam-system"
echo "  4. docker exec -it slam-ouster-fastlio-std bash"
echo ""
