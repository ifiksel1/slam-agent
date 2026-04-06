# Multi-ROS Docker Architecture (ROS1 + ROS2)

**Status**: Design Document
**Support**: ROS1 Noetic + ROS2 Humble
**Decision Point**: Choose architecture before implementation

---

## Architecture Comparison

### Option A: Multi-Image (Recommended)

```
slam_integration:noetic
├── Base: ros:noetic-ros-base-focal (Ubuntu 20.04)
├── SLAM: FAST-LIO2 (ROS1)
├── Packages: 6 ROS1 packages
└── Image Size: 5.11 GB

slam_integration:humble
├── Base: ros:humble-ros-base-jammy (Ubuntu 22.04)
├── SLAM: FAST-LIO2 (ROS2)
├── Packages: 6 ROS2 packages
└── Image Size: ~4.5 GB (estimated)
```

**Pros**:
- ✅ Clean separation
- ✅ Optimized per ROS version
- ✅ Easier troubleshooting
- ✅ Independent versions

**Cons**:
- ❌ Maintain two Dockerfiles
- ❌ Double disk space (5GB × 2)
- ❌ Slower rebuild cycle

**Use `docker-compose.yml`**:
```yaml
services:
  slam_noetic:
    image: slam_integration:noetic
    # ...

  slam_humble:
    image: slam_integration:humble
    # ...
```

---

### Option B: Single Flexible Image

```dockerfile
FROM ros:noetic-ros-base-focal AS ros1_builder
# Build ROS1 packages

FROM ros:humble-ros-base-jammy AS ros2_builder
# Build ROS2 packages

FROM ros:humble-ros-base-jammy AS runtime
# Copy both ROS1 (via ros1_bridge) and ROS2
# Runtime selector script
```

**Pros**:
- ✅ Single image to maintain
- ✅ Easy version switching
- ✅ ROS1/ROS2 interop via bridge

**Cons**:
- ❌ Image size larger (~9 GB)
- ❌ Complex build logic
- ❌ More testing needed

**Use environment variable**:
```bash
docker compose run -e ROS_VERSION=humble slam_system
```

---

### Option C: Compatibility Bridge

```dockerfile
FROM ros:noetic-ros-base-focal AS ros1
# ROS1 Noetic + SLAM

FROM ros:humble-ros-base-jammy AS ros2_bridge
# Install ros1_bridge
# Bridge ROS1 topics ↔ ROS2 topics
```

**Pros**:
- ✅ Seamless ROS1 ↔ ROS2 communication
- ✅ Single deployment
- ✅ Hybrid node support

**Cons**:
- ❌ Bridge adds latency
- ❌ Complex debugging
- ❌ Type conversion overhead

---

## Recommendation: Option A (Multi-Image)

### Why?

1. **Simplicity**: Each image independent
2. **Performance**: No bridge overhead
3. **Clarity**: No confusion about ROS version
4. **Maintainability**: Separate CI/CD pipelines
5. **Production**: Industry standard approach

### Implementation Plan

```
Phase 0a: ROS1 Noetic (Complete) ✅
├── Dockerfile.noetic
├── docker-compose.noetic.yml
└── docs/DOCKER_OPERATIONS_RUNBOOK.md

Phase 0b: ROS2 Humble (New)
├── Dockerfile.humble
├── docker-compose.humble.yml
├── docs/DOCKER_OPERATIONS_RUNBOOK_ROS2.md
└── scripts/docker_diagnostics_ros2.py

Phase 0c: Version Selector (New)
├── docker-compose.yml (selector)
├── .env.example (version setting)
└── scripts/select_ros_version.sh
```

---

## ROS1 vs ROS2 Differences for Runbook

### Tool Changes

| Operation | ROS1 Noetic | ROS2 Humble |
|-----------|------------|------------|
| List nodes | `rosnode list` | `ros2 node list` |
| List topics | `rostopic list` | `ros2 topic list` |
| Echo topic | `rostopic echo /topic` | `ros2 topic echo /topic` |
| Topic rate | `rostopic hz /topic` | `ros2 topic hz /topic` |
| Launch system | `roslaunch pkg launch.launch` | `ros2 launch pkg launch.py` |
| List packages | `rospack list` | `ros2 pkg list` |
| View frames | `rosrun tf view_frames` | `ros2 run tf2_tools view_frames.py` |
| Record bag | `rosbag record -a` | `ros2 bag record -a` |
| Play bag | `rosbag play file.bag` | `ros2 bag play file.bag` |

### Environment Variables

**ROS1**:
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_PACKAGE_PATH=/catkin_ws/src
```

**ROS2**:
```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# No ROS_MASTER_URI in ROS2
```

### Launch Files

**ROS1** (XML):
```xml
<launch>
  <node pkg="fast_lio" type="mapping" name="fastlio_mapping" output="screen">
    <param name="lid_topic" value="/ouster/points" />
  </node>
</launch>
```

**ROS2** (Python):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='mapping',
            name='fastlio_mapping',
            parameters=[{'lid_topic': '/ouster/points'}],
        ),
    ])
```

### Build Systems

**ROS1**: `catkin build`
```bash
catkin build
source devel/setup.bash
```

**ROS2**: `colcon build`
```bash
colcon build
source install/setup.bash
```

---

## Implementation: ROS1 to ROS2 Migration Path

### Step 1: Build ROS2 Image (Dockerfile.humble)

```dockerfile
FROM ros:humble-ros-base-jammy AS builder

# Install build deps
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    # ... (same SLAM deps as ROS1, adjusted for ROS2)

# Create workspace
WORKDIR /colcon_ws/src

# Clone packages (ROS2 forks of SLAM packages)
RUN git clone https://github.com/...

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Build
RUN cd /colcon_ws && colcon build

# Runtime stage
FROM ros:humble-ros-base-jammy AS runtime
COPY --from=builder /colcon_ws/install /colcon_ws/install
```

### Step 2: Create ROS2 Operations Runbook

Copy DOCKER_OPERATIONS_RUNBOOK.md → DOCKER_OPERATIONS_RUNBOOK_ROS2.md

Replace all commands:
```bash
# Old (ROS1)
rostopic list

# New (ROS2)
ros2 topic list
```

### Step 3: Create Version Selector

**docker-compose.yml** (selector):
```yaml
version: '3.8'

services:
  slam_system:
    # Read from .env file
    image: slam_integration:${ROS_VERSION:-noetic}
    # ... rest of config
```

**.env** (version selector):
```
ROS_VERSION=noetic    # Change to 'humble' for ROS2
```

**Usage**:
```bash
# ROS1
ROS_VERSION=noetic docker compose up

# ROS2
ROS_VERSION=humble docker compose up

# Or edit .env and just run:
docker compose up
```

### Step 4: Version-Aware Diagnostics

**scripts/docker_diagnostics.py** (enhanced):
```python
def detect_ros_version():
    """Detect ROS version inside container."""
    exit_code, output = run_command([
        "bash -c 'echo $ROS_DISTRO'"
    ])
    return "humble" if "humble" in output else "noetic"

def run_version_specific_checks():
    version = detect_ros_version()

    if version == "noetic":
        # Run ROS1 checks
        check_ros1_nodes()
    else:
        # Run ROS2 checks
        check_ros2_nodes()
```

---

## Migration Strategy

### Immediate (Phase 0a - Current)
✅ Complete: ROS1 Noetic Docker system

### Short Term (Phase 0b)
Build ROS2 Humble parallel image:
- [ ] Create Dockerfile.humble
- [ ] Build and test ROS2 image
- [ ] Create docker-compose.humble.yml
- [ ] Create DOCKER_OPERATIONS_RUNBOOK_ROS2.md

### Medium Term (Phase 0c)
Unified version selector:
- [ ] Create docker-compose.yml (selector)
- [ ] Create .env configuration
- [ ] Update diagnostics for version detection
- [ ] Version-agnostic scripts

### Long Term (Phase 1+)
Deprecation plan:
- [ ] Monitor ROS1 Noetic EOL (May 2025)
- [ ] Plan ROS2 cutover
- [ ] Test migration path for users

---

## Decision Matrix

| Criteria | Option A | Option B | Option C |
|----------|----------|----------|----------|
| **Complexity** | Low | Medium | High |
| **Performance** | Optimal | Good | Fair |
| **Maintainability** | Easy | Hard | Medium |
| **Disk Space** | 10 GB | 9 GB | 8 GB |
| **ROS1/ROS2 Interop** | Separate | Same | Bridged |
| **User Confusion** | None | Some | High |
| **CI/CD Pipeline** | Simple | Complex | Medium |
| **Recommended** | ✅ YES | ❌ | ⚠️ |

---

## Recommended Next Steps

1. **Decide**: Which architecture? (Recommend: Option A)
2. **If Option A**: Create ROS2 Humble Dockerfile
3. **Build**: Test ROS2 image (parallel to ROS1)
4. **Document**: Create ROS2 operations runbook
5. **Test**: Verify both images work independently
6. **Integrate**: Create version selector (docker-compose.yml)
7. **Validate**: Test switching between versions

---

## Questions to Consider

1. **Target Users**: Will users need both ROS1 and ROS2?
2. **Timeline**: Do you need ROS2 immediately or can it wait?
3. **Hardware**: Same system running both, or separate deployments?
4. **Legacy Support**: How long to support ROS1 after ROS2 ready?
5. **CI/CD**: Can you maintain separate build pipelines?

---

**Recommendation**: Implement **Option A** (Multi-Image)

- Start with current ROS1 setup ✅
- Build ROS2 image in parallel
- Create version selector when both ready
- Document clearly for users

This is the **safest, clearest, most maintainable** approach for production systems.

