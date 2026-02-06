# Phase 3: Configuration File Generation

## Input

Validated `slam_hardware_config.yaml` from Phase 2.

## Files to Generate

### File 1: SLAM Configuration (config/slam_params.yaml)

#### LIO-SAM Template

```yaml
lio_sam:
  # Topics (from user config)
  pointCloudTopic: "/LIDAR_TOPIC"
  imuTopic: "/IMU_TOPIC"
  odomTopic: "odometry/imu"
  gpsTopic: "odometry/gps"

  # Frames
  lidarFrame: "lidar_link"
  baselinkFrame: "base_link"
  odometryFrame: "odom"
  mapFrame: "map"

  # Sensor
  sensor: velodyne  # velodyne, ouster, livox, hesai, robosense
  N_SCAN: CHANNELS  # from config
  Horizon_SCAN: RESOLUTION  # from config
  downsampleRate: 1
  lidarMinRange: 1.0
  lidarMaxRange: 120.0

  # IMU (CRITICAL - requires Allan variance calibration)
  imuAccNoise: 3.9939570888238808e-03  # MUST calibrate
  imuGyrNoise: 1.5636343949698187e-03
  imuAccBiasN: 6.4356659353532566e-05
  imuGyrBiasN: 3.5640318696367613e-05
  imuGravity: 9.80511
  imuRPYWeight: 0.01

  # Extrinsics: LiDAR-to-IMU
  extrinsicTrans: [0.0, 0.0, 0.0]  # from physical measurements
  extrinsicRot: [-1, 0, 0, 0, 1, 0, 0, 0, -1]  # identity or calibrated
  extrinsicRPY: [0, 1, 0, -1, 0, 0, 0, 0, 1]

  # LOAM
  edgeThreshold: 1.0
  surfThreshold: 0.1
  edgeFeatureMinValidNum: 10
  surfFeatureMinValidNum: 100

  # Mapping
  mappingSurfLeafSize: 0.4  # outdoor: 0.4, indoor: 0.2
  surroundingkeyframeAddingDistThreshold: 1.0
  surroundingkeyframeAddingAngleThreshold: 0.2
  surroundingKeyframeDensity: 2.0
  surroundingKeyframeSearchRadius: 50.0

  # Loop Closure
  loopClosureEnableFlag: true  # true for missions > 5 min
  loopClosureFrequency: 1.0
  historyKeyframeSearchRadius: 15.0
  historyKeyframeSearchNum: 25
  historyKeyframeFitnessScore: 0.3
```

#### FAST-LIO Template

```yaml
common:
  lid_topic: "/LIDAR_TOPIC"
  imu_topic: "/IMU_TOPIC"
  time_sync_en: false  # true if using FC IMU

preprocess:
  lidar_type: 2  # 1=Avia, 2=Velodyne/Ouster, 3=Livox
  scan_line: CHANNELS
  blind: 0.5  # minimum range
  point_filter_num: 3  # keep every Nth point (downsample)
  feature_extract_enable: false  # false for ikd-tree method
  scan_rate: 10  # LiDAR scan rate Hz
  timestamp_unit: 0  # 0=second, 1=ms, 2=us, 3=ns

mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  fov_degree: 180
  det_range: 100.0
  extrinsic_est_en: false  # true to auto-calibrate
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]

publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: false

pcd_save:
  pcd_save_en: false
  interval: -1
```

#### OpenVINS (3 files required)

**estimator_config.yaml** -- Copy from OpenVINS examples, adjust:
- `num_pts`: feature count (default 250, reduce for weak compute)
- `use_stereo`: true if stereo camera
- `calib_cam_extrinsics`: true for first flights
- `calib_cam_intrinsics`: true for first flights
- `calib_cam_timeoffset`: true for first flights

**kalibr_imu_chain.yaml**:
```yaml
imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 3.9939570888238808e-03
  accelerometer_random_walk: 6.4356659353532566e-05
  gyroscope_noise_density: 1.5636343949698187e-03
  gyroscope_random_walk: 3.5640318696367613e-05
  model: calibrated
  rostopic: /IMU_TOPIC
  time_offset: 0.0
  update_rate: 200.0
```

**kalibr_imucam_chain.yaml**:
```yaml
cam0:
  T_imu_cam:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [FX, FY, CX, CY]
  resolution: [WIDTH, HEIGHT]
  rostopic: /CAMERA_TOPIC
  timeshift_cam_imu: 0.0
```

WARNING: OpenVINS requires Kalibr camera-IMU calibration. Do not skip.

---

### File 2: URDF (urdf/drone.urdf)

```xml
<?xml version="1.0"?>
<robot name="DRONE_NAME" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link at FC/IMU location -->
  <link name="base_link"/>

  <!-- LiDAR -->
  <link name="lidar_link"/>
  <joint name="base_to_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>
  </joint>

  <!-- Camera (if applicable) -->
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>
  </joint>
</robot>
```

Common mounting patterns:
- Top-mounted upside-down LiDAR: `rpy="3.14159 0 0"`
- Forward-facing camera: `xyz="0.15 0 0.02" rpy="0 0 0"`
- Downward-facing camera: `rpy="0 1.5708 0"`

Alternative -- static TF publishers in launch file:
```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_lidar"
      args="X Y Z YAW PITCH ROLL base_link lidar_link"/>
```
Note: `static_transform_publisher` uses RADIANS and YPR order (not RPY).

---

### File 3: Launch Files

#### SLAM Launch (launch/slam.launch -- ROS1)

```xml
<launch>
  <rosparam file="$(find PACKAGE)/config/slam_params.yaml" command="load"/>
  <node pkg="SLAM_PACKAGE" type="EXECUTABLE" name="SLAM_NODE" output="screen">
    <remap from="/points_raw" to="/LIDAR_TOPIC"/>
    <remap from="/imu_raw" to="/IMU_TOPIC"/>
  </node>
</launch>
```

#### Master Launch (launch/master.launch -- ROS1)

```xml
<launch>
  <!-- MAVROS -->
  <include file="$(find mavros)/launch/apm.launch">
    <arg name="fcu_url" value="DEVICE:BAUD"/>
  </include>

  <!-- Robot Description -->
  <param name="robot_description" textfile="$(find PACKAGE)/urdf/drone.urdf"/>
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher"/>

  <!-- SLAM -->
  <include file="$(find PACKAGE)/launch/slam.launch"/>

  <!-- Vision Bridge -->
  <node pkg="vision_to_mavros" type="vision_to_mavros_node" name="vision_to_mavros">
    <param name="target_frame_id" value="map"/>
    <param name="source_frame_id" value="base_link"/>
    <param name="output_rate" value="30"/>
  </node>
</launch>
```

#### ROS2 Equivalent (launch/master_launch.py)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('PACKAGE')

    return LaunchDescription([
        # MAVROS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch.py')
            ),
            launch_arguments={'fcu_url': 'DEVICE:BAUD'}.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(os.path.join(pkg_dir, 'urdf', 'drone.urdf')).read()}]
        ),

        # SLAM Node
        Node(
            package='SLAM_PACKAGE',
            executable='EXECUTABLE',
            name='SLAM_NODE',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'slam_params.yaml')],
            remappings=[
                ('/points_raw', '/LIDAR_TOPIC'),
                ('/imu_raw', '/IMU_TOPIC'),
            ]
        ),

        # Vision Bridge
        Node(
            package='vision_to_mavros',
            executable='vision_to_mavros_node',
            name='vision_to_mavros',
            parameters=[{
                'target_frame_id': 'map',
                'source_frame_id': 'base_link',
                'output_rate': 30.0,
            }]
        ),
    ])
```

---

### File 4: Vision Bridge

#### Option A: vision_to_mavros (ROS1/ROS2 + ArduPilot)

Node remaps SLAM odometry to `/mavros/vision_pose/pose`.

Configuration in launch file:
```xml
<node pkg="vision_to_mavros" type="vision_to_mavros_node" name="vision_to_mavros">
  <param name="target_frame_id" value="map"/>
  <param name="source_frame_id" value="base_link"/>
  <param name="output_rate" value="30"/>
  <remap from="/vision_pose" to="/mavros/vision_pose/pose"/>
</node>
```

#### Option B: DDS Publisher (ROS2 + PX4)

C++ node that subscribes to SLAM odometry, converts ENU to NED, publishes to `/fmu/in/vehicle_visual_odometry`.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VisionBridge : public rclcpp::Node {
public:
  VisionBridge() : Node("vision_bridge") {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/slam/odometry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto out = px4_msgs::msg::VehicleOdometry();
        out.timestamp = get_clock()->now().nanoseconds() / 1000;
        out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        // ENU -> NED conversion
        out.position = {
          static_cast<float>(msg->pose.pose.position.y),
          static_cast<float>(msg->pose.pose.position.x),
          static_cast<float>(-msg->pose.pose.position.z)
        };
        // ENU->NED quaternion: (w, x, y, z) -> (w, y, x, -z)
        out.q = {
          static_cast<float>(msg->pose.pose.orientation.w),
          static_cast<float>(msg->pose.pose.orientation.y),
          static_cast<float>(msg->pose.pose.orientation.x),
          static_cast<float>(-msg->pose.pose.orientation.z)
        };
        pub_->publish(out);
      });
    pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry", 10);
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionBridge>());
  rclcpp::shutdown();
}
```

---

### File 5: Autopilot Parameters

#### ArduPilot (config/ardupilot_params.parm)

```
AHRS_EKF_TYPE,3
EK3_ENABLE,1
EK3_SRC1_POSXY,6
EK3_SRC1_VELXY,6
EK3_SRC1_POSZ,1
EK3_SRC1_YAW,6
VISO_TYPE,2
ARMING_CHECK,388598
FENCE_ENABLE,1
FENCE_TYPE,7
FENCE_ACTION,1
FENCE_RADIUS,RADIUS
FENCE_ALT_MAX,ALT
FENCE_ALT_MIN,-1
WPNAV_SPEED,SPEED_CM_S
WPNAV_SPEED_UP,150
WPNAV_SPEED_DN,100
```

Parameter reference:
- `EK3_SRC1_POSXY=6`: ExternalNav source for horizontal position
- `EK3_SRC1_VELXY=6`: ExternalNav source for horizontal velocity
- `EK3_SRC1_POSZ=1`: Barometer for altitude (safer than vision Z)
- `EK3_SRC1_YAW=6`: ExternalNav source for yaw
- `VISO_TYPE=2`: MAVLink visual odometry input
- `ARMING_CHECK=388598`: All checks minus GPS
- `FENCE_TYPE=7`: Cylindrical + altitude fence
- `FENCE_ACTION=1`: RTL on breach
- `WPNAV_SPEED`: In cm/s (e.g., 200 = 2 m/s)

#### PX4 (config/px4_params.txt)

```
EKF2_EV_CTRL,15
EKF2_HGT_REF,3
EKF2_EV_DELAY,0
EKF2_EV_POS_X,0.0
EKF2_EV_POS_Y,0.0
EKF2_EV_POS_Z,0.0
GF_ACTION,1
GF_MAX_HOR_DIST,RADIUS
GF_MAX_VER_DIST,ALT
```

Parameter reference:
- `EKF2_EV_CTRL=15`: Fuse all external vision axes (bitmask: pos_x + pos_y + pos_z + yaw)
- `EKF2_HGT_REF=3`: Vision as height reference
- `EKF2_EV_DELAY=0`: Latency compensation (tune if needed, typically 0-50 ms)
- `EKF2_EV_POS_*`: Offset from vision sensor to FC center of gravity
- `GF_ACTION=1`: Warning on geofence breach (set 2 for loiter, 3 for RTL)

---

### File 6: Package Configuration

#### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>PACKAGE_NAME</name>
  <version>0.1.0</version>
  <description>SLAM integration for DRONE_NAME</description>
  <maintainer email="USER_EMAIL">USER_NAME</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>robot_state_publisher</depend>
  <depend>mavros</depend>
  <depend>SLAM_PACKAGE</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

For ROS1, use `<buildtool_depend>catkin</buildtool_depend>` and format="2".

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(PACKAGE_NAME)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# If vision bridge node is included
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(vision_bridge src/vision_bridge.cpp)
ament_target_dependencies(vision_bridge rclcpp nav_msgs px4_msgs)
install(TARGETS vision_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
```

For ROS1:
```cmake
cmake_minimum_required(VERSION 3.0)
project(PACKAGE_NAME)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs tf2_ros)
catkin_package()
install(DIRECTORY config launch urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

---

### File 7: Docker Configuration (if docker=true)

#### Dockerfile

```dockerfile
# Builder stage
FROM ros:DISTRO AS builder
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY . /ws/src/PACKAGE_NAME/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Runtime stage
FROM ros:DISTRO
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY --from=builder /ws/install /ws/install
COPY --from=builder /ws/src/PACKAGE_NAME/config /ws/config
COPY --from=builder /ws/src/PACKAGE_NAME/launch /ws/launch
COPY --from=builder /ws/src/PACKAGE_NAME/urdf /ws/urdf

ENV ROS_DOMAIN_ID=0
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "PACKAGE_NAME", "master_launch.py"]
```

#### docker-compose.yml

```yaml
version: "3.8"
services:
  slam:
    build: .
    network_mode: host
    privileged: true
    devices:
      - /dev/ttyACM0:/dev/ttyACM0  # FC serial
      - /dev/ttyUSB0:/dev/ttyUSB0  # LiDAR serial (if applicable)
    volumes:
      - ./config:/ws/config:ro
      - ./maps:/ws/maps
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - ROS_DOMAIN_ID=0
      - DISPLAY=${DISPLAY}
    restart: unless-stopped
```

#### .dockerignore

```
.git
*.bag
*.db3
maps/*.pcd
build/
install/
log/
__pycache__
*.pyc
```

---

## Output

All files generated with actual values from the validated hardware config (no placeholders remain). Store generated file paths for Phase 4 verification and testing.
