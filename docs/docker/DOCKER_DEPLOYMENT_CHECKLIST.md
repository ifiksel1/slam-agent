# Docker Deployment Checklist - Critical Issues Learned

**Quick Reference for Preventing Known Build Failures**
**Last Updated**: 2026-02-08
**Experience Level**: Production (9 critical issues resolved)

---

## Pre-Build Phase

### 1. Analyze All Package Dependencies
```bash
# BEFORE writing Dockerfile, scan all packages:
grep -r "<build_depend>" /catkin_ws/src/*/package.xml | sed 's/.*<build_depend>//;s/<\/build_depend>.*//' | sort -u

# Output should list all required apt packages
# Example: libspdlog-dev, libjsoncpp-dev, libcurl4-openssl-dev, ...
```

### 2. Check C++ API Requirements
```bash
# For each major C++ library (GTSAM, Ceres, PCL, OpenCV):
# 1. Go to GitHub releases
# 2. Check if specific version needed for APIs used in code
# 3. Document any version conflicts
```

**Known Issues**:
- ✅ STD detector REQUIRES Ceres 2.1+ (for Manifold API)
- ✅ GTSAM 4.0.3 requires Ceres to be pre-installed
- ✅ Ouster ROS requires specific spdlog, jsoncpp, curl versions

### 3. Check ROS Package Components
```bash
# Find all find_package(catkin REQUIRED COMPONENTS) entries:
grep -h "find_package(catkin" /catkin_ws/src/*/CMakeLists.txt | \
    sed 's/.*COMPONENTS//' | tr ' ' '\n' | grep -v '^)' | sort -u

# These must be explicitly installed as ros-<distro>-<pkg>
```

---

## Dockerfile Design Phase

### Critical Order of Operations

**1. Install System Dependencies FIRST**
```dockerfile
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-catkin-tools python3-rosdep ... \
    # SLAM dependencies
    libeigen3-dev libpcl-dev libopencv-dev ... \
    # Ouster-specific (Lesson: Must be explicit!)
    libspdlog-dev libjsoncpp-dev libcurl4-openssl-dev \
    # ROS packages (Lesson: All from find_package!)
    ros-noetic-mavros ros-noetic-dynamic-reconfigure \
    ros-noetic-visualization-msgs ...
```

**2. Pre-install Special Version Dependencies**
```dockerfile
# BEFORE any package that uses it
# Ceres must come BEFORE GTSAM
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && ldconfig
```

**3. Handle Message Generation Dependencies**
```cmake
# In CMakeLists.txt, AFTER find_package(catkin)
add_executable(target_name src/main.cpp)
add_dependencies(target_name ${PROJECT_NAME}_generate_messages_cpp)
```

**4. Use Multi-Stage Builds**
```dockerfile
# Stage 1: Builder (with all build tools)
FROM ros:noetic-ros-base-focal AS builder
# ... install everything, build workspace

# Stage 2: Runtime (minimal)
FROM ros:noetic-ros-base-focal AS runtime
# Copy ONLY:
# - /install directory
# - /src directory (for reference)
# - Compiled libraries from external installs (Ceres, GTSAM)
# DO NOT copy /build directory (saves ~500MB)
```

---

## Build Validation Phase

### Pre-Build Checks

```bash
✓ All package.xml build_depend entries in Dockerfile apt install
✓ All CMakeLists.txt find_package components in apt install
✓ Special version dependencies pre-installed (e.g., Ceres 2.1.0)
✓ Message generation dependencies added (add_dependencies)
✓ Multi-stage Dockerfile structure verified
✓ Workspace ready to build locally first
```

### During Build

```bash
docker compose build --progress=plain | tee docker_build.log

# Watch for:
⚠ SKIP: ... (rosdep silently skipping packages - BAD!)
✗ not found (Ceres, GTSAM, or ROS packages missing)
✗ Could not find ... (Message generation not complete)
```

### Post-Build Validation

```bash
# 1. Image exists and reasonable size
docker images | grep slam_integration
# Should be 4-5 GB (not >6 GB)

# 2. Health check passes
docker compose ps
# STATUS should show (healthy) or Up

# 3. All packages discoverable
docker compose exec -T slam_launch bash -c \
  "source /catkin_ws/devel/setup.bash && rospack list 2>/dev/null | wc -l"
# Should be 50+

# 4. Message headers generated
docker compose exec -T slam_launch bash -c \
  "ls -la /catkin_ws/devel/include/fast_lio/Pose6D.h"
# Must exist

# 5. Ceres version correct
docker compose exec -T slam_launch bash -c \
  "pkg-config --modversion ceres-solver"
# Should output: 2.1.0

# 6. Critical nodes can start
docker compose logs slam_launch | grep -E "ERROR|fatal"
# Should have NO fatal errors
```

---

## Runtime Issues Checklist

### If Build Fails with "X not found"

**Probable Causes** (in order):
1. ❌ Ceres version issue
   - **Fix**: Check `pkg-config --modversion ceres-solver` (need 2.1+)
   - **Solution**: Pre-install Ceres 2.1.0 BEFORE GTSAM

2. ❌ Missing build dependency
   - **Fix**: Check `dpkg -l | grep <pkg>`
   - **Solution**: Add to `apt-get install` in Dockerfile

3. ❌ Missing ROS package
   - **Fix**: Check `dpkg -l | grep ros-noetic-<pkg>`
   - **Solution**: Add `ros-noetic-<pkg>` to apt install

4. ❌ Message generation incomplete
   - **Fix**: Check `ls -la /catkin_ws/devel/include/<pkg>/`
   - **Solution**: Add `add_dependencies(target ${PROJECT_NAME}_generate_messages_cpp)`

5. ❌ CMakeLists.txt missing find_package
   - **Fix**: Check `grep find_package /catkin_ws/src/<pkg>/CMakeLists.txt`
   - **Solution**: Add missing components to both CMakeLists.txt and Dockerfile

---

## Performance Metrics to Monitor

| Component | Expected | Acceptable | Action if > |
|-----------|----------|------------|-------------|
| Image size | 5.1 GB | 5.5 GB | Check for `/build` copy; remove non-essential tools |
| Build time | 90 min | 120 min | May indicate recompilation; check cache |
| Ceres build | 25 min | 30 min | Normal, depends on system; be patient |
| GTSAM build | 20 min | 25 min | Normal, depends on system; be patient |
| catkin build | 25 min | 30 min | Parallel (-j4 recommended) |
| Container RAM | <4 GB idle | <5 GB | May indicate memory leak; check ros logs |

---

## Essential Diagnostics Commands

```bash
# Complete diagnostic suite (in order)
docker compose exec -T slam_launch bash << 'EOF'
source /catkin_ws/devel/setup.bash

echo "=== Ceres ==="
pkg-config --modversion ceres-solver

echo "=== Build Dependencies ==="
dpkg -l | grep -E "(spdlog|jsoncpp|curl)" | awk '{print $2, $3}'

echo "=== Message Headers ==="
ls -la /catkin_ws/devel/include/fast_lio/Pose6D.h

echo "=== ROS Packages ==="
rospack find fast_lio ouster_ros std_detector 2>&1 | head -3

echo "=== Nodes ==="
rosnode list 2>/dev/null | head -5 || echo "(ROS Master not running)"

echo "=== Topics ==="
rostopic list 2>/dev/null | head -5 || echo "(ROS Master not running)"

echo "=== Launch Files ==="
find /catkin_ws -name "*.launch" | wc -l
EOF
```

---

## Common Issues & Quick Fixes

### Issue: Ceres Manifold API not found
```
error: 'EigenQuaternionManifold' was not declared
```
**Root Cause**: Ubuntu focal provides Ceres 1.14, need 2.1+
**Fix**: Add to builder Dockerfile BEFORE GTSAM
```dockerfile
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && ldconfig
```

### Issue: "X header not found"
```
fatal error: spdlog/spdlog.h: No such file
fatal error: json/json.h: No such file
```
**Root Cause**: rosdep install silently skipped missing packages
**Fix**: Explicitly add to Dockerfile apt install
```dockerfile
libspdlog-dev \
libjsoncpp-dev \
libcurl4-openssl-dev \
```

### Issue: "Pose6D.h: No such file or directory"
```
fatal error: fast_lio/Pose6D.h: No such file
```
**Root Cause**: Message generation timing - executable compiled before Pose6D.msg processed
**Fix**: Add to src/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt
```cmake
add_executable(fastlio_mapping ...)
add_dependencies(fastlio_mapping ${PROJECT_NAME}_generate_messages_cpp)
```

### Issue: "Could not find a package configuration file provided by mavros"
```
Could not find a package configuration file provided by "mavros"
```
**Root Cause**: ROS package not installed in builder stage
**Fix**: Add to Dockerfile apt install
```dockerfile
ros-noetic-mavros \
ros-noetic-dynamic-reconfigure \
ros-noetic-visualization-msgs \
```

### Issue: Container won't start / scripts fail
```
/home/dev/slam_ws/devel/setup.bash: No such file
```
**Root Cause**: Hardcoded host paths don't exist in container
**Fix**: Use container detection in scripts
```bash
if [ -f /.dockerenv ]; then
    WS_ROOT="/catkin_ws"
else
    WS_ROOT="/home/dev/slam_ws"
fi
```

### Issue: Configuration files immutable
**Root Cause**: Config directories not mounted in docker-compose.yml
**Fix**: Add to docker-compose.yml volumes
```yaml
volumes:
  - ./src/orin_slam_integration/config:/catkin_ws/src/orin_slam_integration/config:rw
  - ./src/orin_slam_integration/launch:/catkin_ws/src/orin_slam_integration/launch:rw
```

---

## Lessons Encoded in Enhanced Diagnostics

The `docker_diagnostics.py` script now automatically checks for:

✅ `check_ceres_version()` - Detects Ceres < 2.1 issues
✅ `check_build_dependencies()` - Detects missing spdlog, jsoncpp, curl
✅ `check_message_generation()` - Detects incomplete Pose6D.h generation
✅ `check_container_environment()` - Detects path issues
✅ `check_config_mounts()` - Detects immutable configs
✅ `check_health_status()` - Detects container health issues

**Run after each build**:
```bash
cd /home/dev/slam-agent
python3 scripts/docker_diagnostics.py
```

---

## Decision Tree for Build Failures

```
Build fails?
├─ Ceres/Manifold API error?
│  └─ Pre-install Ceres 2.1.0 BEFORE GTSAM
├─ "Header not found" (spdlog, jsoncpp, curl)?
│  └─ Add package to apt-get install explicitly
├─ "Could not find package" (mavros, dynamic_reconfigure)?
│  └─ Add ros-noetic-<pkg> to apt-get install
├─ "Pose6D.h not found"?
│  └─ Add add_dependencies() to fast_lio CMakeLists.txt
├─ Path error (/home/dev/slam_ws not found)?
│  └─ Use /.dockerenv detection in scripts
├─ Configuration immutable?
│  └─ Add volume mount to docker-compose.yml
└─ Other error?
   └─ Check DOCKER_BUILD_LESSONS_LEARNED.md (Section X)
```

---

## Success Criteria

A successful Docker build should pass ALL of:

- [ ] `docker compose build` completes with no FATAL errors
- [ ] `docker compose ps` shows `(healthy)` status
- [ ] `docker compose exec -T slam_launch bash -c "source /catkin_ws/devel/setup.bash && rospack find fast_lio"` succeeds
- [ ] `pkg-config --modversion ceres-solver` outputs 2.1.0+
- [ ] `dpkg -l | grep -E "(spdlog|jsoncpp|curl)"` shows all 3 packages
- [ ] `ls /catkin_ws/devel/include/fast_lio/Pose6D.h` exists
- [ ] `python3 /home/dev/slam-agent/scripts/docker_diagnostics.py` passes 12+ checks
- [ ] Final image size 4.5-5.5 GB
- [ ] Build time 90-120 minutes

---

**Use this checklist BEFORE writing any Dockerfile.**
**Use diagnostics AFTER each build attempt.**
**Refer to DOCKER_BUILD_LESSONS_LEARNED.md for detailed explanations.**
