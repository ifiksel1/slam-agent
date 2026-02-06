# AI-Guided SLAM Integration System Builder - INDEX

**Version**: 2.1  
**Date**: December 2024  
**Purpose**: Main routing file for phase-based AI guidance  
**Token Count**: ~3,000 (vs 26,000 in monolithic version)

---

## 🎯 HOW TO USE THIS GUIDE (FOR AI)

**YOU ARE AN AI ASSISTANT**. This guide uses selective phase loading for token efficiency.

### Loading Strategy

**ALWAYS LOAD**: This index file (~3k tokens)  
**LOAD AS NEEDED**: Phase-specific files based on current conversation state

### Phase-Based Architecture (Option A - Selective Reading)

```
Current Phase → Load Only These Line Ranges
─────────────────────────────────────────────
Phase 1: Assessment          → AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (~12k tokens)
Phase 2: Validation          → AI_SYSTEM_BUILDER_GUIDE.md lines 1506-1598 (~2k tokens)
Phase 3: File Generation     → AI_SYSTEM_BUILDER_GUIDE.md lines 1599-3454 (~15k tokens)
Phase 4: Workspace Setup     → AI_SYSTEM_BUILDER_GUIDE.md lines 3455-5057 (~14k tokens)
Phase 4.5: Docker (Optional) → AI_SYSTEM_BUILDER_GUIDE.md lines 5058-5718 (~6k tokens)
Phase 5: Testing             → AI_SYSTEM_BUILDER_GUIDE.md lines 5719-5977 (~3k tokens)
Phase 6-7: Troubleshooting  → AI_SYSTEM_BUILDER_GUIDE.md lines 5978-6505 (on-demand, ~3k per section)
Phase 8: Troubleshooting     → AI_SYSTEM_BUILDER_GUIDE.md lines 6506-7888 (on-demand, ~3k per section)
```

**Note**: We use Option A (selective reading from monolithic file), not split files.

**Token Savings**: 
- Old: 26,000 tokens × 35 turns = 910,000 tokens
- New: ~8,000 tokens × 35 turns = 280,000 tokens
- Savings: 630,000 tokens (69%)

---

## 🎯 IMPLEMENTATION: OPTION A (Selective Reading)

**ACTIVE METHOD**: Read specific line ranges from monolithic file as needed

**Source File**: `AI_SYSTEM_BUILDER_GUIDE.md` (147KB, ~26k tokens if loaded fully)

**Strategy**: Load ONLY the lines you need for current phase using the `read_file` tool with `offset` and `limit` parameters

## 📋 CONVERSATION STATE TRACKING (SAVE/RESUME FEATURE)

Users can save progress and resume later.

Track where you are in the process:

```yaml
# SLAM Integration Progress - Save this to resume later!
# Generated: 2025-11-24 16:30:00
# Version: 1.0

current_phase: 1  # Update as you progress (1-7)
session_id: "20251124-163000"  # Timestamp for tracking

user_hardware:
  platform: "Jetson Orin NX 16GB"
  platform_specs:
    ram_gb: 16
    cpu_cores: 8
    gpu: "NVIDIA Ampere"
    os: "Ubuntu 20.04"
  
  lidar: "Ouster OS1-64"
  lidar_specs:
    channels: 64
    connection: "Ethernet"
    ip: "192.168.1.201"
    computer_ip: "192.168.1.100"
    interface: "eth0"
    has_imu: true
    imu_model: "BMI088"
  
  imu_source: "lidar"  # "lidar" or "fc"
  imu_reasoning: "Hardware-synced with scans, BMI088 quality ≥ FC IMU"
  
  camera: "Intel RealSense D435i"
  camera_specs:
    type: "RGB-D"
    resolution: "640x480"
    fps: 30
  
  fc: "Pixhawk 6X"
  fc_specs:
    imu_model: "ICM-42688-P"
    autopilot: "ArduPilot"
    firmware: "4.5.7"
    connection: "/dev/ttyACM0"
    baud: 921600
  
  slam_algorithm: "FAST-LIO2"
  slam_repo: "https://github.com/hku-mars/FAST_LIO"
  
  ros_version: "ROS1 Noetic"
  use_dds: false  # true if ROS2+PX4 or ROS2+ArduPilot (optional)
  
  urdf_status: "will_create"  # "exists", "will_create", "use_static_tf"
  
  environment:
    type: "Indoor warehouse"
    size: "50m x 50m"
    features: "Moderate (shelves, walls)"
  
  mission:
    max_speed: 3.0  # m/s
    duration: 10  # minutes
    requires_loop_closure: true

completed_phases:
  - phase: 1
    completed_at: "2025-11-24 16:30:00"
    summary: "Hardware assessment complete"
  # Add more as phases complete

files_generated:
  - path: "~/catkin_ws/src/my_slam_integration/config/fast_lio_config.yaml"
    phase: 3
    type: "slam_config"
  - path: "~/catkin_ws/src/my_slam_integration/urdf/drone.urdf"
    phase: 3
    type: "urdf"
  # Add more as files are created

installation_status:
  workspace_created: true
  workspace_path: "~/catkin_ws"
  workspace_type: "catkin"  # or "colcon" for ROS2
  
  core_ros_packages: true
  ros_packages_checked: true
  
  mavros: true
  mavros_version: "1.16.0"
  mavros_geographiclib: true
  micro_ros_agent: false  # Only for ROS2+PX4
  
  sensor_drivers:
    ouster_ros: true
    ouster_ros_path: "~/catkin_ws/src/ouster-ros"
    realsense: false
    zed: false
    livox: false
  
  slam_algorithm: false
  slam_algorithm_name: "FAST-LIO2"
  slam_repo_cloned: false
  slam_repo_path: ""
  slam_dependencies_installed: false
  
  vision_to_mavros: false
  vision_to_mavros_path: ""
  
  lidar_network_configured: true
  lidar_ping_success: true
  lidar_udp_verified: true
  
  integration_package_created: false
  integration_package_path: ""
  config_files_copied: false
  
  workspace_built: false
  build_successful: false
  packages_sourced: false
  bashrc_updated: false

physical_measurements:
  base_frame: "base_link"
  lidar_offset: [0.0, 0.0, -0.06]  # x, y, z in meters
  lidar_rotation: [3.14159, 0.0, 0.0]  # roll, pitch, yaw in radians
  camera_offset: [0.05, 0.0, 0.0]
  camera_rotation: [0.0, 0.0, 0.0]

notes:
  - "User stopped after Phase 1 assessment"
  - "Need to resume at Phase 2 validation"
  - "Ethernet LiDAR tested and pinging successfully"

next_steps:
  - "Load Phase 2 validation (lines 1506-1598)"
  - "Show compatibility summary"
  - "Get user confirmation to proceed"
```

### 💾 How to Use Save/Resume

**For AI Assistants:**

1. **Generate progress YAML after each phase**:
   ```
   "Here's your progress file. Save this if you need to stop:
   
   [Generate YAML with current state]
   
   You can resume later by providing this file."
   ```

2. **When user provides saved YAML**:
   ```python
   # Parse the YAML
   state = parse_yaml(user_provided_yaml)
   
   # Determine where to resume
   last_phase = state['current_phase']
   next_phase = last_phase + 1
   
   # Load appropriate section
   if next_phase == 2:
       read_file("AI_SYSTEM_BUILDER_GUIDE.md", offset=1506, limit=93)
   elif next_phase == 3:
       read_file("AI_SYSTEM_BUILDER_GUIDE.md", offset=1599, limit=1856)
   # etc.
   
   # Acknowledge resume
   "Welcome back! I see you completed Phase {last_phase}. 
   Let's continue with Phase {next_phase}..."
   ```

3. **Update YAML as you progress**:
   - Increment `current_phase` when moving to next phase
   - Add to `completed_phases` list with timestamp
   - Update `files_generated` when creating files
   - Track `installation_status` during Phase 4
   - Add `notes` for important decisions/issues

**For Users:**

Save your progress at any time:
1. Ask AI: "Can I get my progress file?"
2. AI generates current state YAML
3. Save to file: `slam_progress_20251124.yaml`
4. Resume later: "I want to resume my SLAM integration" + provide file

**Benefits:**
- ✅ Stop and resume without starting over
- ✅ Share progress with team members
- ✅ Track what's been done vs what's left
- ✅ Recover from interruptions
- ✅ AI knows exactly where you left off
- ✅ All previous answers preserved in compact format

---

## 🖥️ PLATFORM-SPECIFIC INTERACTION GUIDELINES

### 🔷 VOXL Systems (ModalAI) - CRITICAL

**If user has VOXL/ModalAI hardware**, you MUST use `adb shell` for ALL interactions:

#### Connection Method
```bash
# ALL VOXL commands MUST be prefixed with "adb shell"
adb shell "command_here"

# Examples:
adb shell "systemctl status voxl-qvio-server"
adb shell "voxl-inspect-services"
adb shell "px4-listener vehicle_visual_odometry"
```

#### Key VOXL Characteristics
- **NO standard Linux shell access** - requires Android Debug Bridge (adb)
- **NO ROS topics** - uses MPA pipes (`/run/mpa/`)
- **PX4 runs onboard** - not on separate flight controller
- **Pre-installed VIO** - qVIO or OpenVINS already configured
- **Portal GUI** - http://voxl-ip:5000 for visualization

#### VOXL-Specific Tools (all require adb shell)
| Command | Purpose |
|---------|---------|
| `voxl-inspect-services` | Check all services status & CPU |
| `px4-listener <topic>` | Monitor PX4 uORB topics (NOT rostopic) |
| `voxl-inspect-pose <pipe>` | View MPA pipe data |
| `systemctl restart <service>` | Restart VOXL services |
| `journalctl -u <service>` | View service logs |
| `voxl-configure-vision-hub wizard` | Configure VIO forwarding |

#### Configuration Files (via adb shell)
- `/etc/modalai/voxl-vision-hub.conf` - VIO → PX4 bridge
- `/etc/modalai/voxl-qvio-server.conf` - qVIO settings
- `/etc/modalai/voxl-open-vins-server.conf` - OpenVINS settings
- `/etc/modalai/vio_cams.conf` - Camera selection for VIO
- `/etc/modalai/voxl-camera-server.conf` - Camera enables
- `/etc/modalai/extrinsics.conf` - System-wide extrinsics

#### VOXL Workflow Differences
1. **Service restarts**: Always use systemctl via adb shell
2. **No catkin/colcon**: Software pre-installed, config-only
3. **Initialization required**: VIO needs figure-8 motion to start
4. **Portal-first debugging**: Check http://voxl-ip:5000 before logs

#### When to Load Phase 9 (VOXL)
- User mentions: "VOXL", "ModalAI", "Starling", "Seeker", "Sentinel"
- User says PX4 runs on compute board
- User mentions qVIO or VOXL-OpenVINS
- **Load immediately**: AI_SYSTEM_BUILDER_GUIDE.md lines 8499-9050

#### Critical VOXL Gotchas
❌ **NEVER** run commands without `adb shell` prefix  
❌ **NEVER** use `rostopic echo` - use `px4-listener` instead  
❌ **NEVER** skip initialization motion (VIO won't start)  
❌ **NEVER** assume ROS workspace exists  
❌ **NEVER** blindly disable cameras - compare with working backup first  
❌ **NEVER** restore old backup configs without checking SDK compatibility  
✅ **ALWAYS** check portal first  
✅ **ALWAYS** verify service status before troubleshooting  
✅ **ALWAYS** restart services after config changes  
✅ **ALWAYS** compare with working backup before making changes  
✅ **ALWAYS** check for `cal_file_2` field when restoring old configs  

### 🔷 Standard ROS Systems (Jetson, NUC, etc.)

**If user has standard Linux computer**, use normal shell commands:

```bash
# Direct command execution (no adb shell)
systemctl status mavros
rostopic echo /mavros/vision_pose/pose
catkin_make
source ~/catkin_ws/devel/setup.bash
```

#### Key Differences from VOXL
- ✅ Direct shell access
- ✅ ROS topics for data flow
- ✅ MAVROS for PX4 communication
- ✅ Manual workspace creation (catkin/colcon)
- ✅ Build from source (git clone, build)

---

## 🗺️ QUICK REFERENCE: What To Read When (Option A)

**Method**: Use `read_file` tool with `offset` and `limit` on `AI_SYSTEM_BUILDER_GUIDE.md`

### Starting New Session
**Read**: 
- This index (AI_GUIDE_INDEX.md) - full file
- AI_SYSTEM_BUILDER_GUIDE.md lines 1-100 (header + instructions)
- AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (Phase 1 questions)

**Total**: ~11k tokens

**Action**: Begin asking Phase 1 questions

---

### Phase 1: Initial Assessment (Q1-Q11 + Q2b)
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (~12k tokens)

**Contains**:
- Opening statement with calibration warnings (lines 60-80)
- Q1: Hardware platform (lines 86-140)
- Q2: LiDAR sensor + network setup (lines 142-430)
  - OS-specific network setup examples
- **Q2b: IMU source selection** (lines 553-798) **CRITICAL**
- Q3: Camera (lines 799-828)
- Q4: Flight controller (lines 830-908)
- Q5: SLAM algorithm (lines 910-1078)
- Q6: ROS version (lines 1080-1126)
- Q7: URDF status (lines 1128-1150)
- Q8: Physical measurements (lines 1153-1198)
- Q9: Operating environment (lines 1200-1240)
- Q10: Mission requirements (lines 1242-1278)
- Q11: Docker Container Support (lines 1289-1337)

**When complete**: 
- Summarize user's config in 500 tokens
- DON'T keep Phase 1 lines in context
- Read Phase 2 lines

---

### Phase 2: Validation & Summary
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 1506-1598 (~2k tokens)
- Keep: User config summary (500 tokens)

**Contains**:
- Compatibility matrix (lines 1506-1565)
- Summary template (lines 1567-1598)
- Validation checklist

**When complete**:
- Store validated config summary
- DON'T keep Phase 2 lines
- Read Phase 3 lines

---

### Phase 3: File Generation
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 1599-3454 (~15k tokens)
- Keep: User config summary (500 tokens)

**Contains**:
- SLAM config templates (lines 1599-1870)
  - LIO-SAM template (detailed example) with calibration warnings
  - OpenVINS template (VIO-specific, includes camera-IMU config)
- URDF creation tutorial (lines 1872-2320)
- Launch file templates (lines 2322-2520)
- Parameter files (lines 2522-2670)
- Package configuration (lines 2672-2820)
- Docker configuration (lines 3032-3454) - Optional Dockerfiles, docker-compose, build scripts
- Quick Start vs Production path selection

**When complete**:
- Store generated file paths (200 tokens summary)
- DON'T keep Phase 3 lines
- Read Phase 4 lines

---

### Phase 4: Workspace Setup & Installation
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 3455-5057 (~14k tokens)
- Keep: Installation checklist (500 tokens)
- **CRITICAL**: Check what's already installed from progress YAML (if resuming)

**Contains**:
- **⭐ Automated Installation Scripts** (NEW! - see scripts/README_INSTALL.md)
  - Interactive installation: `./install_slam_integration.sh`
  - Config file mode: `./install_slam_integration.sh config.yaml`
  - Automates all steps below
- Step 1: Workspace creation (lines 3455-3525)
- Step 2: Core ROS packages (lines 3527-3595)
- Step 3: MAVROS/DDS install (lines 3597-3765)
- Step 4: Sensor drivers (lines 3767-3945)
- Step 5: SLAM algorithm (lines 3947-4125)
  - Docker options for each algorithm
- Step 6: vision_to_mavros (lines 4127-4205)
- Step 7: LiDAR network (lines 4207-4305)
  - OS-specific examples
- Step 8: Integration package (lines 4307-4365)
- Step 9: Build workspace (lines 4367-4425)
- Step 10: Verification (lines 4427-4485)

### Phase 4.5: Docker Container Build & Deployment (Optional)
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 5058-5718 (~6k tokens)
- Only if: `USE_DOCKER == true` from Phase 1 Q11

**Contains**:
- Base image options (including OpenVINS official Docker)
- Docker image build instructions
- Container testing and verification
- Hardware access configuration (USB, network, X11)
- Production deployment (registry, systemd service)
- Monitoring and logging
- Docker troubleshooting

**BEFORE Installing Each Component**:
1. **Check progress YAML** (if resuming):
   ```yaml
   installation_status:
     workspace_created: true  # ← Skip workspace creation
     mavros: true             # ← Skip MAVROS install
     sensor_drivers:
       ouster_ros: true       # ← Skip Ouster install
       realsense: false       # ← Need to install
     slam_algorithm: false    # ← Need to install
   ```

2. **Check existing installation**:
   ```bash
   # Workspace exists?
   ls ~/catkin_ws/devel/setup.bash 2>/dev/null && echo "exists" || echo "missing"
   
   # ROS package installed?
   rospack find mavros 2>/dev/null && echo "installed" || echo "missing"
   ros2 pkg list | grep mavros  # ROS2
   
   # System package installed?
   dpkg -l | grep ros-noetic-tf2  # Check if installed
   
   # Git repo cloned?
   ls ~/catkin_ws/src/ouster-ros 2>/dev/null && echo "exists"
   ```

3. **Only install what's missing**:
   - If package exists → Skip with message: "✓ Already installed: mavros"
   - If missing → Proceed with installation
   - Update progress YAML after each successful install

4. **Handle conflicts/overwrites**:
   - If config file exists: Ask user before overwriting
   - If git repo exists: Ask to pull/update or skip
   - If build artifacts exist: Offer to clean before rebuild

**Example Check Workflow**:
```
AI: "Checking your system..."
    [Runs checks]
    
    "Found:
    ✓ Workspace exists: ~/catkin_ws
    ✓ MAVROS installed
    ✓ Ouster driver installed
    ✗ FAST-LIO not found
    ✗ vision_to_mavros not found
    
    I'll install the missing components (FAST-LIO, vision_to_mavros).
    This will take about 10 minutes. Ready?"
```

**When complete**:
- Store installation status (300 tokens) with all checks
- DON'T keep Phase 4 lines
- Read Phase 5 lines

---

### Phase 5: Testing & Validation
**Read**: 
- AI_SYSTEM_BUILDER_GUIDE.md lines 5719-5977 (~3k tokens)

**Contains**:
- Mandatory Pre-Flight Checklist (lines 5719-5765) - Blocks progression if incomplete
- Bench testing (lines 5767-5820)
- Ground testing (lines 5822-5875)
- Flight testing (lines 5877-5930)
- Performance validation (lines 5932-5977)

**When complete**:
- System verified
- Provide final summary (500 tokens)

---

### Phase 6-7: Troubleshooting & Optimization (On-Demand)
**Read ONLY when user reports specific issue**:
- Troubleshooting guide: lines 5826-5905 (~2k)
- Optimization & tuning: lines 5906-5977 (~2k)

### Phase 8: Common Setup Issues Reference (On-Demand)
**Read ONLY when user reports specific issue**:
- Coordinate frames: lines 5978-6113 (~3k)
- ROS environment: lines 6114-6234 (~2k)
- Visualization: lines 6239-6423 (~2k)
- Performance: lines 6427-6567 (~3k)
- Sensor calibration: lines 6506-7326 (~8k) **EXPANDED**
  - Camera calibration (lines 6510-6627)
  - Camera-IMU calibration with detailed Kalibr tutorial (lines 6629-7420)
  - **LiDAR-IMU calibration (NEW!)** (lines 7421-7670) ⭐
    - LI-Calib offline calibration
    - Online calibration (FAST-LIO extrinsic_est_en)
    - Manual measurement + frame transformation
    - Verification procedure
    - Common LiDAR frame conventions table
  - IMU calibration (Allan variance) (lines 7671-7744)
- Hardware issues: lines 7328-7386 (~2k)
- Data quality: lines 7388-7452 (~2k)
- Dependencies: lines 7454-7532 (~2k)
  - Expanded build error recovery table
- VIO-specific troubleshooting: lines 7479-8137 (~6k)
  - Environment requirements for VIO (lines 7484-7560)
  - VIO initialization guidance (lines 7562-7654)
  - Visual tracking failures (lines 7656-7749)
  - Initialization failures (lines 7751-7849)
  - Scale drift (lines 7851-7949)
  - Rotation drift (lines 7951-8049)
  - Environment-specific issues (lines 8051-8137)
- **Self-improvement: Documenting new solutions** (lines 8702-8805) ⭐ NEW
  - When to document solutions
  - How to document with templates
  - Updating existing solutions
  - Quality standards

**Strategy**: 
- User says "drone moves backward" → Read lines 5978-6113 (coordinate frames)
- User says "package not found" → Read lines 6114-6234 (ROS environment)
- User says "SLAM is slow" → Read lines 6427-6567 (performance)
- User says "build fails" → Read lines 7454-7532 (dependencies, expanded error table)
- User says "VIO tracking lost" / "OpenVINS fails" → Read lines 7479-8137 (VIO troubleshooting)
- User says "VIO won't initialize" → Read lines 7562-7654 (VIO initialization guidance)
- User says "poor environment" → Read lines 7484-7560 (environment requirements)
- **User says "LiDAR-IMU calibration" / "extrinsics wrong"** → Read lines 7421-7670 (LiDAR-IMU calibration) ⭐
- **After solving new issue not in guide** → Read lines 8702-8805 (document the solution!) ⭐

**Don't preload all troubleshooting** - read specific line ranges as needed.

---

### Phase 9: VOXL-Specific Validation (ModalAI Systems) ⭐ NEW

**When to use**: User has VOXL computer (ModalAI) with PX4 and OpenVINS/qVIO

**Read**: AI_SYSTEM_BUILDER_GUIDE.md lines 8499-9485 (~20k tokens - expanded with "CAM_MISSING STALLED" troubleshooting)

**Contains**:
- **9.1**: VOXL architecture overview (MPA pipes, vision-hub, onboard PX4)
- **9.2**: 10-step pre-flight validation procedure
  - Service checks (voxl-inspect-services)
  - OpenVINS health monitoring (journalctl)
  - Vision-hub status and timestamp errors
  - OpenVINS initialization (requires motion!)
  - Verifying vision data reaches PX4 (px4-listener)
  - EKF fusion verification
  - PX4 EKF2 parameters (EKF2_EV_CTRL = 15)
  - Drift test (60 second stationary)
  - Feature tracking check (via portal)
  - Estimator status
- **9.3**: Complete VOXL validation checklist
- **9.4**: Common VOXL issues & solutions
  - "never published" troubleshooting
  - Timestamp errors in vision-hub
  - voxl-inspect-pose format errors
  - Low feature count fixes
  - **"CAM_MISSING STALLED" error** ⭐ NEW (lines 9134-9330)
    - Standby mode causing 0% CPU (most common!)
    - SDK version mismatch (cal_file_2 missing)
    - Wrong pipe name (_ion vs non-_ion)
    - Camera overload / frame drops
    - Vision-hub connection issues
    - Complete diagnostic flowchart
- **9.5**: VOXL command reference (px4-*, systemctl, config files)
- **9.6**: VOXL portal usage guide
- **9.7**: When to use VOXL procedures

**Key Differences from Standard ROS**:
- Uses MPA pipes (/run/mpa/) not ROS topics
- voxl-vision-hub bridges VIO to PX4 (not MAVROS)
- PX4 runs onboard VOXL (voxl-px4 service)
- Use px4-listener instead of rostopic echo
- OpenVINS/qVIO pre-installed, just needs initialization

**Critical VOXL-Specific Checks**:
1. `px4-listener vehicle_visual_odometry` must NOT show "never published"
2. Vision-hub must NOT show continuous timestamp errors
3. OpenVINS REQUIRES motion to initialize (figure-8 for 10-15s)
4. 3-second warmup period before data sent to PX4

**When to Reference**:
- User mentions "VOXL", "ModalAI", "Starling", "Seeker", "Sentinel"
- User has PX4 running on compute board (not separate Pixhawk)
- User mentions "MPA pipes" or "voxl-vision-hub"
- OpenVINS/qVIO pre-installed
- Standard ROS commands don't work (no rostopic)

---

## 🔑 KEY PRINCIPLES (Option A Implementation)

### 0. Key External Resources (Use Browser Tools)
**SLAM Installation & Configuration Repository**: https://github.com/engcang/SLAM-application ⭐
- **Installation guides** for 20+ SLAM systems (FAST-LIO2, LIO-SAM, LVI-SAM, Point-LIO, etc.)
- **Working config files** and troubleshooting fixes
- **Use browser tools** to navigate to this repo when:
  - User needs help installing a specific SLAM algorithm
  - Build errors or dependency conflicts arise
  - User asks about SLAM algorithm options
  - Need example configurations to adapt
- Navigate to specific algorithm folder (e.g., `/FAST_LIO/`, `/direct_lidar_odometry/`)

**ArduPilot DDS Docker Setup**: https://github.com/ezrpa/ardupilot_ros2_swarm/blob/main/src/dockerfiles/Dockerfile.dds-agent ⭐
- **Docker-based micro-ROS agent** for ArduPilot + ROS2
- Includes `ardupilot_msgs` pre-built
- Reference when user wants DDS for ArduPilot (ROS2)
- Provides complete Dockerfile with all dependencies

### 1. Selective Reading (Use read_file with offset/limit)
**CRITICAL**: Only read the line ranges you need RIGHT NOW. Don't read the entire 26k token file.

Example:
```python
# DON'T do this:
read_file("AI_SYSTEM_BUILDER_GUIDE.md")  # Reads all 26k tokens ❌

# DO this instead:
read_file("AI_SYSTEM_BUILDER_GUIDE.md", offset=60, limit=1445)  # Phase 1 only, ~12k tokens ✅
```

### 2. Summarize, Don't Repeat
After Phase 1, create a 500-token summary of user's hardware. Reference this instead of keeping full Q&A in context.

Example summary:
```yaml
config:
  platform: Jetson Orin NX 16GB
  lidar: Ouster OS1-64 (BMI088 IMU)
  imu_source: lidar  # Using LiDAR IMU (hardware-synced)
  fc: Pixhawk 6X (ICM-42688-P IMU)
  slam: FAST-LIO2
  ros: ROS1 Noetic
  environment: Indoor warehouse, 50m×50m
  max_speed: 3 m/s
```

### 3. Generate Files Once
After generating a config file, don't repeat its contents. Just reference:
```
"Your SLAM config is in ~/catkin_ws/src/my_slam/config/slam_params.yaml"
```

### 4. Conditional Troubleshooting (Read specific line ranges)
Only read troubleshooting lines when user reports specific issues:
- "Drone moves wrong direction" → Read lines 5978-6113 (coordinate frames)
- "Can't find package" → Read lines 6114-6234 (ROS environment)
- "SLAM too slow" → Read lines 6427-6567 (performance)

### 5. Don't Keep Old Phases in Context
After moving to Phase 2, DON'T keep Phase 1 lines in memory. Only keep:
- This index file
- User config summary (500 tokens)
- Current phase lines

### 6. Language Preference: C++ for Performance-Critical Code
**CRITICAL**: When generating any code files, **prefer C++ implementations** for:
- SLAM algorithm nodes (if creating custom implementations)
- Sensor processing nodes
- Bridge nodes (vision_to_mavros, DDS publishers)
- Real-time data processing
- Any performance-critical components

**Python is acceptable for**:
- Launch files (ROS2 uses Python launch files by default)
- Utility scripts (calibration helpers, data conversion)
- Non-real-time tools

**Rationale**: C++ provides lower latency, better real-time performance, more predictable execution times, and better resource efficiency on embedded platforms. This is the industry standard for production SLAM systems.

---

## 📊 TOKEN USAGE TRACKING

Keep mental note of current token usage:

```
Typical session (35 turns):
─────────────────────────────────────────
Turn 1-10 (Phase 1):   12k × 10 = 120k
Turn 11-12 (Phase 2):  2k × 2  = 4k
Turn 13-18 (Phase 3):  15k × 6 = 90k
Turn 19-30 (Phase 4):  14k × 12 = 168k
Turn 31-35 (Phase 5):  3k × 5  = 15k
History accumulation:   ~100k
─────────────────────────────────────────
Total:                  ~497k tokens ✅

Compare to reading entire guide each turn:
26k × 35 = 910k tokens
Savings: 413k tokens (45%)
```

---

## 🚀 WORKFLOW EXAMPLE (Option A)

**Turn 1**: User says "Help me integrate SLAM"
- Read: AI_GUIDE_INDEX.md (full file, 3k tokens)
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (Phase 1, 12k tokens)
- Total context: 15k tokens
- Ask: Q1 (Hardware platform)

**Turn 2-11**: Phase 1 questions
- Keep: Phase 1 lines in context
- Use browser tools to look up specs
- Track answers in conversation
- Still at 11k tokens per turn

**Turn 12**: Phase 1 complete
- Create: 500-token config summary
- DON'T keep Phase 1 lines anymore
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 1506-1598 (Phase 2, 2k tokens)
- Total context: Index (3k) + summary (500) + Phase 2 (2k) = 5.5k tokens
- Show: Compatibility check + summary

**Turn 13**: User confirms
- DON'T keep Phase 2 lines anymore
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 1599-3454 (Phase 3, 15k tokens)
- Total context: Index (3k) + summary (500) + Phase 3 (15k) = 18.5k tokens
- Begin: File generation

**Turns 14-19**: Generate files
- Keep: Phase 3 lines + config summary
- Generate: SLAM config, URDF, launch files, etc.
- Store: File paths (add 200 tokens to summary)
- Context: ~13.7k tokens

**Turn 20**: Files complete
- DON'T keep Phase 3 lines anymore
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 3455-5057 (Phase 4, 14k tokens)
- Total context: Index (3k) + summary (700) + Phase 4 (14k) = 17.7k tokens
- Begin: Installation

**Turns 21-32**: Workspace setup
- Keep: Phase 4 lines + installation checklist
- Guide: Through each installation step
- Track: Installation status (add 300 tokens to summary)
- Context: ~16k tokens

**Turn 33**: Build complete
- DON'T keep Phase 4 lines anymore
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 5719-5977 (Phase 5, 3k tokens)
- Total context: Index (3k) + summary (1k) + Phase 5 (3k) = 7k tokens
- Begin: Testing procedures (with mandatory pre-flight checklist)

**Turns 34-35**: Testing
- Keep: Phase 5 lines
- Verify: System working
- Context: ~10k tokens

**Turn 36** (if issue): User reports "Drone moves backward"
- Read: AI_SYSTEM_BUILDER_GUIDE.md lines 5978-6113 (troubleshooting, 3k tokens)
- Total context: Index (3k) + summary (1k) + troubleshooting (3k) = 7k tokens
- Diagnose: Frame orientation issue
- Fix: Update URDF rotation
- DON'T keep troubleshooting lines after resolved

**Result**: Average 10-12k tokens per turn vs 26k if reading entire guide.

---

## 🔄 RESUME SESSION EXAMPLE

**Scenario**: User completed Phase 1 yesterday, wants to continue today

**Turn 1**: User returns
- User: "I want to resume my SLAM integration from yesterday"
- AI: "Welcome back! Do you have your progress file from last time?"

**Turn 2**: User provides progress YAML
- User: [Pastes YAML file]
- AI: [Parses YAML]
  ```yaml
  current_phase: 1
  completed_phases:
    - phase: 1
      completed_at: "2025-11-23 14:30:00"
  user_hardware:
    platform: "Jetson Orin NX 16GB"
    lidar: "Ouster OS1-64"
    slam: "FAST-LIO2"
    # ... etc
  ```
- AI determines: Phase 1 complete, need Phase 2 next
- AI reads: Lines 1280-1370 (Phase 2 validation)
- Context: Index (3k) + Phase 2 (2k) = 5k tokens (no need to reload Phase 1!)

**Turn 3**: AI continues seamlessly
- AI: "Welcome back! I see you completed Phase 1 (Hardware Assessment) 
  yesterday. You're using:
  • Jetson Orin NX 16GB
  • Ouster OS1-64 LiDAR (using BMI088 IMU)
  • Pixhawk 6X flight controller
  • FAST-LIO2 SLAM algorithm
  • ROS1 Noetic
  
  Let's continue with Phase 2: Validation & Compatibility Check.
  
  [Shows compatibility matrix]"

**Turns 4-8**: Phase 2 complete
- AI validates compatibility
- AI generates updated progress YAML:
  ```yaml
  current_phase: 2
  completed_phases:
    - phase: 1
      completed_at: "2025-11-23 14:30:00"
    - phase: 2
      completed_at: "2025-11-24 10:15:00"
  # ... rest of state
  ```
- AI: "Phase 2 complete! Here's your updated progress file [YAML].
  Would you like to continue with Phase 3 (File Generation) now?"

**Turn 9**: User must stop again
- User: "I need to stop here, can I get my progress?"
- AI: "Absolutely! Here's your progress file:
  
  [Generates updated YAML with Phase 2 complete]
  
  Save this file and provide it when you return. We'll pick up
  at Phase 3 (File Generation) next time."

**Turn 10** (next day): User resumes again
- User: "Resume from this: [pastes YAML with Phase 2 complete]"
- AI: Loads Phase 3 (lines 1599-3454, 15k tokens)
- AI: "Welcome back! You've completed Phases 1-2. Let's generate
  your configuration files now (Phase 3)..."

**Key Benefit**: User answered Phase 1 questions ONCE, never repeated!

---

## 📁 FILE STRUCTURE

```
slam_integration/docs/
├── AI_GUIDE_INDEX.md                    ← YOU ARE HERE (always load, ~3k tokens)
├── AI_SYSTEM_BUILDER_GUIDE.md           ← Monolithic guide (read selectively by line ranges)
│                                           Phase 1: lines 60-1505
│                                           Phase 2: lines 1506-1598
│                                           Phase 3: lines 1599-3454
│                                           Phase 4: lines 3455-5057
│                                           Phase 4.5: lines 5058-5718 (Docker, optional)
│                                           Phase 5: lines 5719-5977
│                                           Phase 6-7: lines 5826-5977
│                                           Phase 8: lines 5978-7888 (on-demand)
│
├── OPTIMIZATION_README.md               ← Implementation details (for developers)
├── OPTIMIZATION_SUMMARY.md              ← Benefits overview (for users)
├── SAVE_RESUME_FEATURE.md               ← Progress YAML guide
├── SLAM_ARDUPILOT_INTEGRATION_GUIDE.md  ← Technical reference
├── SLAM_INTEGRATION_TEMPLATE.md         ← Config templates
└── SLAM_INTEGRATION_DIAGNOSTICS.md      ← Troubleshooting guide
```

**Note**: We use Option A (selective reading) - read specific line ranges from `AI_SYSTEM_BUILDER_GUIDE.md` instead of split files.

---

## 🎓 REMEMBER

1. **You're optimizing for tokens**, not reading speed
2. **Load ONLY what you need** for the current phase
3. **Summarize previous phases** in 500 tokens max
4. **Don't repeat generated files** - reference by path
5. **Track phase number** - determines what to load next
6. **Use browser tools** to reduce back-and-forth
7. **Troubleshoot on-demand** - don't preload all issues

---

## ✅ START HERE (Option A Instructions)

### Starting New Session

When user requests SLAM integration help:

1. **Read**: AI_GUIDE_INDEX.md (this file, full)
2. **Read**: AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (Phase 1 questions)
   ```
   read_file("AI_SYSTEM_BUILDER_GUIDE.md", offset=60, limit=1445)  # Phase 1: 60-1505
   ```
3. **Present**: Welcome message from Phase 1 (around line 66)
4. **Ask**: Q1 (Hardware platform, starts around line 86)
5. **Begin**: Systematic questioning process

**Files to read initially**:
- AI_GUIDE_INDEX.md (~3k tokens) ← Full file
- AI_SYSTEM_BUILDER_GUIDE.md lines 60-1505 (~12k tokens) ← Phase 1 only

**Total starting context**: ~15k tokens (vs 26k if you read entire guide)

### Resuming Existing Session (NEW!)

When user says "I want to resume" or provides progress YAML:

1. **Ask for progress file** if not provided:
   ```
   "Do you have your progress file from last time? 
   It's a YAML file I generated for you."
   ```

2. **Parse the YAML** to extract:
   - `current_phase` - Where they stopped
   - `user_hardware` - All their hardware specs
   - `completed_phases` - What's already done
   - `files_generated` - Files already created
   - `installation_status` - What's installed
   - `next_steps` - What to do next

3. **Determine resume point**:
   ```python
   last_completed = state['current_phase']
   next_phase = last_completed + 1
   
   # Load appropriate lines
   phase_line_ranges = {
       2: (1506, 93),     # Validation: 1506-1598
       3: (1599, 1856),   # Generation: 1599-3454
       4: (3455, 1603),   # Installation: 3455-5057
       4.5: (5058, 661),  # Docker: 5058-5718
       5: (5719, 259),    # Testing: 5719-5977
   }
   
   if next_phase in phase_line_ranges:
       offset, limit = phase_line_ranges[next_phase]
       read_file("AI_SYSTEM_BUILDER_GUIDE.md", offset=offset, limit=limit)
   ```

4. **Acknowledge and continue**:
   ```
   "Welcome back! I see you completed Phase {last_completed} 
   ({phase_name}) on {date}.
   
   You've already:
   - [List completed items from YAML]
   
   Let's continue with Phase {next_phase}: {next_phase_name}
   
   [Proceed with next phase steps]"
   ```

5. **Update progress YAML** as you continue:
   - After each phase completion
   - When generating new files
   - During installation milestones
   - Provide updated YAML to user periodically

**Benefits of Resume Feature**:
- User can stop mid-session without losing work
- Team members can collaborate (share progress file)
- AI picks up exactly where user left off
- No need to re-answer Phase 1 questions
- Track installation progress across sessions

### As You Progress

- DON'T keep previous phase lines in memory
- Read next phase lines as needed (see line ranges above)
- **Generate progress YAML after each phase completion**
- Summarize user config in 500 tokens (or load from YAML)
- Only read troubleshooting lines when user reports issues

---

**Document Version**: 2.1  
**Optimization**: Phase-based selective loading  
**Token Reduction**: ~54% savings per session  
**Cost Reduction**: ~42-50% depending on model

