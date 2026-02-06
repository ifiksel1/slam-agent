# Phase 1: Hardware Assessment

## Pre-Check: Cached Profiles
Before asking questions, read `docs/learned/hardware_profiles.yaml`.
If a profile matches what the user describes, show it and ask:
"I have a cached profile for this hardware. Want to use it (skip to Phase 2) or start fresh?"
If accepted, output the cached `phase1_config` and proceed to Phase 2.

## Batching Strategy

Ask questions in 3 batches instead of 11 individual turns:

### Batch 1: Core Hardware (Ask all at once)
Ask the user for ALL of these in a single message:

**Q1: Computing Platform**
- What computer runs SLAM? (Jetson Orin NX 8/16GB, AGX Xavier, Orin Nano, Xavier NX, Intel NUC, x86 laptop, Raspberry Pi 4/5, TX2, custom)
- OS version? (Ubuntu 18.04/20.04/22.04, JetPack version)
- RAM? CPU cores? GPU?

**Q2: LiDAR Sensor** (if any)
- Brand/model? (Ouster OS1-64/128, OS2, Velodyne VLP-16/32C, Livox Avia/Mid-360, RoboSense RS-16/32, Hesai Pandar XT16/XT32, Slamtec RPLIDAR 2D, None)
- Channels/beams?
- Connection: Ethernet/USB/Serial?
- If Ethernet: Computer interface name? Computer IP? LiDAR IP?
- Does it have a built-in IMU? Model?

**Q3: Camera** (if any)
- Brand/model? (RealSense D435/D435i/D455, T265, ZED/ZED2, stereo CSI, monocular USB, fisheye, event camera, none)
- Type: Mono/Stereo/RGB-D/Event?
- Resolution, FPS, connection?

**Q4: Flight Controller**
- Model? (Pixhawk 4/5X/6X/6C, Cube Orange/Orange+, Kakute H7, Durandal, ARK V6X, other)
- Autopilot: ArduPilot or PX4?
- Firmware version?
- Connection to computer: USB/UART/UDP? Device path? Baud rate?

### Batch 2: Software & Algorithm (Ask all at once)

**Q5: SLAM Algorithm**
Recommend based on Batch 1 answers:

For LiDAR setups:
| Algorithm | Best For | Compute | Loop Closure |
|-----------|----------|---------|-------------|
| FAST-LIO2 | Most setups | 4GB+ RAM | No (add SC-A-LOAM/STD) |
| LIO-SAM | Long missions | 8GB+ RAM | Yes (built-in) |
| COIN-LIO | Degenerate environments | 6GB+ RAM | No |
| Cartographer | 2D/3D mapping | 6GB+ RAM | Yes |
| LeGO-LOAM | Outdoor ground | 4GB+ RAM | No |
| Point-LIO | Fast motion | 4GB+ RAM | No |

For camera setups:
| Algorithm | Best For | Compute | Loop Closure |
|-----------|----------|---------|-------------|
| ORB-SLAM3 | Visual SLAM | 4GB+ RAM | Yes |
| RTAB-Map | RGB-D | 6GB+ RAM | Yes |
| OpenVINS | VIO | 2GB+ RAM | No |
| VINS-Fusion | VIO with LC | 4GB+ RAM | Yes |
| Kimera-VIO | VIO + mesh | 4GB+ RAM | Yes |

Multi-sensor: LVI-SAM, R3LIVE, FAST-LIVO2
GNSS-integrated: GLIO, GVINS (for intermittent GPS)

Ask: Which algorithm? Or provide GitHub URL for custom.

**Q6: ROS Version**
- ROS1 Noetic, ROS2 Humble, ROS2 Foxy, ROS2 Iron?
- Communication method is auto-determined:
  - ROS1 + any FC → MAVROS
  - ROS2 + ArduPilot → MAVROS (default) or DDS (preferred)
  - ROS2 + PX4 → DDS (native)

**Q11: Docker Support**
- Run SLAM in Docker container? Yes/No

### Batch 3: Physical & Mission (Ask all at once)

**Q2b: IMU Source Selection** (AI decides, presents recommendation)
Based on Batch 1 answers, recommend which IMU to use for SLAM:

Decision logic:
1. If LiDAR has IMU with quality >= FC IMU → Use LiDAR IMU (hardware-synced, higher rate)
2. If FC IMU significantly better (2+ tiers) → Use FC IMU
3. If FAST-LIO + Ouster/Hesai with IMU → Use LiDAR IMU
4. If LiDAR has no IMU → Use FC IMU (only option)
5. If high-speed flight (>5 m/s) → Prefer higher data rate source

FC IMU quality reference:
| FC | IMU Model | Quality |
|---|---|---|
| Pixhawk 4/5/6, Cube Orange | ICM-20689/ICM-42688/ICM-20602 | Excellent |
| ARK V6X | ICM-42688-P | Excellent |
| Kakute H7 | BMI270 | Good |
| Generic racing FC | MPU6000/6050 | Fair |

LiDAR IMU quality reference:
| LiDAR | IMU Model | Quality |
|---|---|---|
| Ouster OS1/OS2 | BMI088 | Very Good (hardware-synced, 200Hz) |
| Hesai (some models) | BMI088 | Very Good |
| Livox Mid-360 | Built-in | Good (200Hz) |
| Livox Avia | 6-axis | Fair |
| Velodyne | None | N/A |

Present recommendation with reasons, let user confirm or override.

**Q7: URDF Status**
- Have existing URDF? / Need to create one? / Use static TF publishers?

**Q8: Physical Measurements**
- LiDAR position relative to flight controller (x, y, z in meters)?
- LiDAR rotation (roll, pitch, yaw in degrees)? Common: upside-down = 180 roll
- Camera position and rotation (if applicable)?

**Q9: Operating Environment**
- Indoor/Outdoor/Both?
- Area size?
- Feature richness: Lots of geometry (walls, objects) or sparse (open field)?

**Q10: Mission Requirements**
- Max flight speed (m/s)?
- Mission duration (minutes)?
- Need loop closure? (missions > 5 min or revisiting areas)
- Drift tolerance?

## Browser Tools Usage

For EVERY question, proactively use web search:
- Q1: "[platform] specifications" → verify RAM, CPU, GPU
- Q2: "[lidar model] specifications" → channels, range, default IP, ROS driver, built-in IMU
- Q2: "[lidar model] ROS driver github" → topic names, PointCloud2 format
- Q3: "[camera model] ROS driver" → topic names, supported modes
- Q4: "[FC model] IMU specifications" → IMU model, noise specs
- Q5: Navigate to SLAM algorithm GitHub repo → dependencies, supported sensors, config format
- Reference: https://github.com/engcang/SLAM-application for installation guides

## Ethernet LiDAR Network Setup

If LiDAR uses Ethernet, guide through:
1. Identify network interface: `ip a`
2. Configure static IP via netplan (`/etc/netplan/01-lidar-network.yaml`)
3. Test: `ping [LIDAR_IP]`
4. Verify data: `sudo tcpdump -i [INTERFACE] udp port [PORT] -c 10`

Default IPs: Ouster=link-local/192.0.2.x, Velodyne=192.168.1.201, Hesai=192.168.1.201, Livox=192.168.1.1xx

## Sensor Compatibility Note

Most SLAM algorithms work with ANY LiDAR publishing `sensor_msgs/PointCloud2`. The algorithm docs listing specific LiDARs are just tested examples. If user's LiDAR publishes PointCloud2 → compatible.

## Output

After all 3 batches, produce a structured config:

```yaml
# slam_hardware_config.yaml
platform:
  model: ""
  ram_gb: 0
  cpu_cores: 0
  gpu: ""
  os: ""

lidar:
  model: ""
  channels: 0
  connection: ""  # ethernet/usb/serial
  ip: ""
  computer_ip: ""
  interface: ""
  has_imu: false
  imu_model: ""

camera:
  model: ""
  type: ""  # mono/stereo/rgbd/event/none
  resolution: ""
  fps: 0
  connection: ""

flight_controller:
  model: ""
  autopilot: ""  # ardupilot/px4
  firmware_version: ""
  connection: ""
  device_path: ""
  baud_rate: 0
  imu_model: ""

slam:
  algorithm: ""
  repo_url: ""

ros:
  version: ""  # noetic/humble/foxy/iron
  comm_method: ""  # mavros/dds

imu_source: ""  # lidar/fc
imu_topic: ""
imu_reasoning: ""

physical:
  base_frame: "base_link"
  lidar_offset: [0, 0, 0]
  lidar_rotation: [0, 0, 0]  # degrees
  camera_offset: [0, 0, 0]
  camera_rotation: [0, 0, 0]
  urdf_status: ""  # exists/create/static_tf

environment:
  type: ""  # indoor/outdoor/both
  area_size: ""
  features: ""  # rich/moderate/sparse

mission:
  max_speed: 0
  duration_minutes: 0
  loop_closure: false
  drift_tolerance: ""

docker: false
```

## Save to Learning System
After generating the config, append a new entry to `docs/learned/hardware_profiles.yaml`:
- Generate fingerprint: `{platform}-{lidar|camera}-{slam}-{autopilot}-{ros_version}`
- Set `validated: false`, `integration_complete: false`
- Store the full `slam_hardware_config.yaml` as `phase1_config`

Proceed to Phase 2 with this config.
