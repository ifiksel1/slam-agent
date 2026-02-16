# Phase 0: Docker Deployment & Containerization

**Status**: Production Ready
**Prerequisites**: Docker, docker-compose, Git
**Hardware**: Any system with Docker support
**Time Estimate**: 30-60 minutes (depends on network speed)
**Success Criteria**: All 6 SLAM packages compile, container starts, ROS environment initializes

---

## Overview

This phase containerizes the complete SLAM integration system using Docker multi-stage builds, optimizing for:
- **Fast builds** on systems with slow native compilation
- **Reproducible environments** (tested on Ubuntu 20.04 focal)
- **Portable deployment** to Jetson Orin NX or any x86/ARM64 system
- **Clean separation** of build and runtime stages
- **Production-ready** with all dependencies pre-compiled

---

## Architecture

### Multi-Stage Docker Build

```
┌─────────────────────────────────────┐
│   Stage 1: Builder (Compiler)       │
├─────────────────────────────────────┤
│ Base: ros:noetic-ros-base-focal     │
│ Compiles:                           │
│  • Ceres 2.1.0 (from source)        │
│  • GTSAM 4.0.3 (from source)        │
│  • 6 ROS packages (catkin build)    │
│ Output: /catkin_ws/{devel,src}      │
│ Size: ~8-10 GB (temporary)          │
└─────────────────────────────────────┘
              ↓ (library extraction)
┌─────────────────────────────────────┐
│  Stage 2: Runtime (Minimal)         │
├─────────────────────────────────────┤
│ Base: ros:noetic-ros-base-focal     │
│ Copies:                             │
│  • Compiled binaries                │
│  • Runtime libraries only           │
│  • ROS environment                  │
│ Output: slam_integration:latest     │
│ Size: 5.11 GB (optimized)           │
└─────────────────────────────────────┘
```

---

## Critical Build Issues & Solutions

### Issue 1: Ceres Version Incompatibility

**Problem**: STD package requires Ceres 2.1+ for Manifold API, but Ubuntu focal only has Ceres 1.14

**Solution**: Pre-compile Ceres 2.1.0 from source BEFORE GTSAM

```dockerfile
# Install Ceres Solver 2.1.0 (required by STD package for Manifold API)
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    make -j$(nproc) && make install && ldconfig
```

**Why**: GTSAM bundles older Ceres but doesn't export CMake config properly

---

### Issue 2: Message Generation Dependencies

**Problem**: fast_lio executable compiles before Pose6D.msg is generated

**Solution**: Add explicit dependency in CMakeLists.txt

```cmake
add_executable(fastlio_mapping src/laserMapping.cpp ...)
add_dependencies(fastlio_mapping ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ...)
```

**Why**: ROS message generation must complete before targets that use them

---

### Issue 3: Missing Build Dependencies

**Problem**: ouster-ros requires spdlog, libjsoncpp-dev, nodelet; orin_slam_integration requires mavros, dynamic-reconfigure

**Solution**: Exhaustively list all build_depend packages in Dockerfile apt install

```dockerfile
RUN apt-get install -y \
    libspdlog-dev \              # ouster-ros
    libjsoncpp-dev \             # ouster-ros
    ros-noetic-nodelet \         # ouster-ros
    ros-noetic-mavros \          # orin_slam_integration
    ros-noetic-dynamic-reconfigure \ # orin_slam_integration
    ...
```

**Why**: `rosdep install -r` silently skips missing packages with `-r` flag

---

## Build Procedure

### Step 1: Verify Source Tree

```bash
cd ~/slam_ws
# Check all 6 packages present
for pkg in fast_lio aloam_velodyne std_detector ouster_ros vision_to_mavros orin_slam_integration; do
  echo -n "$pkg: "
  [ -d "src/${pkg%/*}" ] && echo "✓" || echo "✗ MISSING"
done
```

### Step 2: Build Docker Image

```bash
# Full build (no cache)
docker build -t slam_integration:latest -f Dockerfile .

# Or with progress output
docker buildx build --progress=plain -t slam_integration:latest .
```

**Build Time**: ~1 hour on typical hardware (Ceres+GTSAM compilation intensive)

**Memory Required**: 6+ GB RAM (Jetson Orin NX: 8 GB available)

### Step 3: Verify Image

```bash
# Check size and creation time
docker images | grep slam_integration

# Verify all packages discoverable
docker run --rm slam_integration:latest bash -c \
  "source /catkin_ws/devel/setup.bash && rospack list | wc -l"
# Should return: 111 (or similar)
```

---

## Deployment

### Using docker-compose

```bash
cd ~/slam_ws

# Start complete SLAM system (MAVROS + LiDAR + SLAM + Vision Bridge)
docker compose up -d slam_launch

# Monitor logs
docker compose logs -f slam_launch

# Access container
docker compose exec slam_launch bash

# Stop system
docker compose down
```

### Using raw Docker

```bash
# Interactive shell
docker run -it --rm --network host \
  --device=/dev/ttyACM0 \
  slam_integration:latest bash

# Run command
docker run --rm --network host \
  --device=/dev/ttyACM0 \
  slam_integration:latest \
  bash -c "source /catkin_ws/devel/setup.bash && roslaunch orin_slam_integration master.launch"
```

---

## Validation Tests

### T-1: Package Discovery

```bash
docker run --rm slam_integration:latest bash -c \
  "source /catkin_ws/devel/setup.bash && \
   for pkg in fast_lio ouster_ros std_detector orin_slam_integration aloam_velodyne vision_to_mavros; do \
     echo -n \"$pkg: \"; \
     rospack find $pkg 2>/dev/null && echo OK || echo FAIL; \
   done"
```

**Expected**: All 6 packages return OK

### T-2: Launch Files

```bash
docker run --rm slam_integration:latest bash -c \
  "find /catkin_ws -name '*.launch' | wc -l"
```

**Expected**: 26+ launch files found

### T-3: ROS Topics (Container Running)

```bash
docker compose up -d slam_launch
sleep 10
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rostopic list | wc -l"
docker compose down
```

**Expected**: 11+ topics publishing

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Build Time | 60-90 min | Peak during Ceres/GTSAM compilation |
| Image Size | 5.11 GB | Optimized: no build artifacts |
| Memory During Build | 6+ GB | GTSAM compilation intensive |
| Memory Runtime | 1-2 GB | Depends on LiDAR processing |
| CPU Usage | 70-100% | During compilation; ~30-40% runtime |
| Base OS | Ubuntu 20.04 focal | ROS Noetic compatible |

---

## Files Modified

### Dockerfile
- Added Ceres 2.1.0 pre-install
- Fixed GTSAM library dependencies
- Optimized multi-stage build (removed build/ copy)
- Added complete library list for runtime

### docker-compose.yml
- Added orin_slam_integration config mount
- Fixed healthcheck (rospack instead of rosnode)
- Fixed slam_launch to use master.launch
- Cleaned up dead named volumes

### Source Code Fixes
- `src/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt`: Added message generation dependency
- `preflight_check.sh`: Added container-aware path detection (/.dockerenv)

### Documentation
- `.dockerignore`: Optimized exclusions
- Generated: DOCKER_BUILD_REPORT.md, DOCKER_VALIDATION_REPORT.md, SYSTEM_TEST_REPORT.md

---

## Troubleshooting

### Build Fails: "Could not find a package configuration file provided by X"

**Cause**: Missing ROS package in builder stage apt install

**Fix**: Add to Dockerfile builder apt install:
```dockerfile
RUN apt-get install -y ros-noetic-<package>
```

### Build Fails: "EigenQuaternionManifold not found"

**Cause**: Ceres 1.14 installed instead of 2.1.0

**Fix**: Ensure Ceres 2.1.0 build comes BEFORE GTSAM in Dockerfile

### Build Fails: "fast_lio/Pose6D.h: No such file"

**Cause**: Message generation dependency missing

**Fix**: Add to CMakeLists.txt:
```cmake
add_dependencies(target_name ${PROJECT_NAME}_generate_messages_cpp)
```

### Container Won't Start: "ROS_MASTER_URI not set"

**Cause**: Entrypoint script not executed or sourcing failed

**Fix**: Manually source environment:
```bash
docker run ... bash -c "source /catkin_ws/devel/setup.bash && ..."
```

---

## Next Phase Integration

After successful Docker deployment:
- **Phase 5** (Testing): Use container for repeatable system tests
- **Phase 6** (Troubleshooting): Container isolates variables for debugging
- **Phase 7** (Optimization): Profile code in container environment

---

## Learning & Profiles

**Hardware Profile**: Docker-based (generic x86/ARM64)
- Base: Ubuntu 20.04 + ROS Noetic
- GPU: Optional (NVIDIA container runtime)
- Memory: 6+ GB recommended

**Known Good Configs**:
- Ceres 2.1.0 + GTSAM 4.0.3 + Ubuntu focal ✓
- All 6 packages compile without errors ✓
- Container size: 5.11 GB (optimized) ✓

---

## References

- Dockerfile: `/home/dev/slam_ws/Dockerfile`
- docker-compose.yml: `/home/dev/slam_ws/docker-compose.yml`
- Build Report: `/home/dev/slam_ws/DOCKER_BUILD_REPORT.md`
- Validation Report: `/home/dev/slam_ws/DOCKER_VALIDATION_REPORT.md`
- Test Report: `/home/dev/slam_ws/SYSTEM_TEST_REPORT.md`
- Memory/Lessons: `/home/dev/.claude/projects/-home-dev-slam-ws/memory/MEMORY.md`

---

**Phase 0 Status**: ✅ Complete and Production Ready
