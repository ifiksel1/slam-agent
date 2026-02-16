# Docker Build Lessons Learned - SLAM Integration System

**Version**: 1.0
**Date**: 2026-02-08
**Status**: Critical - Apply to all future Docker builds
**Experience**: Production build completed with 3 iterations, 9 critical issues resolved

---

## Executive Summary

This document captures critical lessons learned from successfully building and containerizing the complete SLAM integration system. The build process encountered **9 critical/high-priority issues** that required investigation, solution design, and implementation. Understanding these issues is essential for future deployments and troubleshooting.

### Key Takeaway
**Always check C++ API requirements, build dependencies, and message generation timing before building ROS packages in Docker.**

---

## 1. CRITICAL ISSUE: Ceres Solver Version Incompatibility

### Problem
**Build Error**: `std_detector` package failed with:
```
fatal error: ceres/internal/eigen.h:...
error: 'EigenQuaternionManifold' was not declared in this scope
```

### Root Cause
- STD (Stable Triangle Descriptor) requires Ceres 2.1+ for `Manifold` API
- Ubuntu 20.04 (Focal) in `ros:noetic-ros-base-focal` only provides Ceres 1.14
- Ceres 1.14 lacks the `EigenQuaternionManifold` class introduced in 2.1

### Solution Applied
**Pre-install Ceres 2.1.0 from source BEFORE GTSAM in builder stage:**

```dockerfile
# Install Ceres Solver 2.1.0 FIRST (required by STD package for Manifold API)
WORKDIR /tmp
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/ceres-solver
```

### Prevention Checklist
- [ ] Check all C++ packages for specific version requirements in their documentation
- [ ] For ROS packages, review `package.xml` `build_depend` entries
- [ ] For algorithm packages (GTSAM, Ceres, OpenCV), check GitHub releases and API docs
- [ ] Test the build locally on native system first if possible
- [ ] If custom version needed, pre-install BEFORE dependent packages

### Diagnosis Commands
```bash
# Check Ceres version in container
pkg-config --modversion ceres-solver

# Check which packages use Ceres
grep -r "find_package(Ceres" /catkin_ws/src/

# Test Ceres Manifold API availability
cat > /tmp/test_ceres.cpp << 'EOF'
#include <ceres/manifold.h>
using namespace ceres;
int main() { return 0; }
EOF
g++ -c /tmp/test_ceres.cpp $(pkg-config --cflags ceres)
```

---

## 2. CRITICAL ISSUE: Missing ROS Build Dependencies

### Problem
**Build Error**: `ouster-ros` failed with:
```
fatal error: spdlog/spdlog.h: No such file or directory
fatal error: json/json.h: No such file or directory
fatal error: curl/curl.h: No such file or directory
```

### Root Cause
- Package.xml declares `build_depend` for `libspdlog-dev`, `libjsoncpp-dev`, `libcurl4-openssl-dev`
- Running `rosdep install -r` with `-r` flag silently skips missing packages
- Dockerfile didn't explicitly install these packages
- Build continued anyway, failing at cmake/compile step

### Solution Applied
**Explicitly list ALL build_depend packages in Dockerfile apt install:**

```dockerfile
RUN apt-get update && apt-get install -y \
    # From package.xml build_depend entries
    libspdlog-dev \
    libjsoncpp-dev \
    libcurl4-openssl-dev \
    # From CMakeLists.txt find_package(catkin REQUIRED COMPONENTS ...)
    ros-noetic-mavros \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-visualization-msgs \
    # ... other deps
```

### Prevention Checklist
- [ ] Read ALL `package.xml` files in workspace for `<build_depend>` entries
- [ ] Parse all `CMakeLists.txt` files for `find_package(catkin REQUIRED COMPONENTS ...)`
- [ ] Create comprehensive apt dependency list BEFORE running rosdep
- [ ] Don't rely on `rosdep install -r` to catch all dependencies
- [ ] Build locally first to catch missing system dependencies

### Diagnosis Commands
```bash
# List all build_depend entries
grep "<build_depend>" /catkin_ws/src/*/package.xml

# Check for missing apt packages
for pkg in $(grep "<build_depend>" /catkin_ws/src/*/package.xml | sed 's/.*<build_depend>//;s/<\/build_depend>.*//' | sort -u); do
  apt-cache search "^${pkg}$" >/dev/null 2>&1 || echo "MISSING: $pkg"
done

# Verify installed packages
dpkg -l | grep -E "(spdlog|jsoncpp|curl)"
```

---

## 3. CRITICAL ISSUE: ROS Message Generation Timing

### Problem
**Build Error**: `fast_lio` failed with:
```
fatal error: fast_lio/Pose6D.h: No such file or directory
```

The executable `fastlio_mapping` was compiled BEFORE the custom message `Pose6D.msg` was generated from `Pose6D.msg`.

### Root Cause
- `fast_lio/CMakeLists.txt` declares `add_message_files()` and `generate_messages()`
- Build system may compile executable before running message generation
- Missing explicit dependency declaration between target and message generation

### Solution Applied
**Add explicit dependency in CMakeLists.txt:**

```cmake
# In src/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt
# AFTER find_package(catkin ...) but BEFORE add_executable()

# Ensure messages are generated BEFORE executables use them
add_executable(fastlio_mapping src/laserMapping.cpp ...)
add_dependencies(fastlio_mapping ${PROJECT_NAME}_generate_messages_cpp)

# Also ensure message headers are generated before this target
target_include_directories(fastlio_mapping PRIVATE
    ${PROJECT_BINARY_DIR}/devel/include
    ${catkin_INCLUDE_DIRS})
```

### Prevention Checklist
- [ ] For any ROS package with custom messages, check for `add_message_files()`
- [ ] Verify `generate_messages()` is called BEFORE `add_executable()`
- [ ] Add explicit `add_dependencies(target_name ${PROJECT_NAME}_generate_messages_cpp)`
- [ ] Test build locally with `catkin build --verbose` to verify message generation order
- [ ] Check `devel/include/` exists with generated headers before compilation

### Diagnosis Commands
```bash
# Check if package generates messages
grep -l "add_message_files" /catkin_ws/src/*/CMakeLists.txt

# Check if add_dependencies is set
grep "add_dependencies.*generate_messages" /catkin_ws/src/*/CMakeLists.txt

# Verify message generation completed
ls -la /catkin_ws/devel/include/fast_lio/ | grep Pose6D

# Check message header timestamp vs executable timestamp
ls -la /catkin_ws/devel/lib/fast_lio/fastlio_mapping
ls -la /catkin_ws/devel/include/fast_lio/Pose6D.h
# Header should be older (generated first)
```

---

## 4. HIGH ISSUE: Missing ROS Packages in Builder Stage

### Problem
**Build Error**: `orin_slam_integration` CMakeLists.txt failed:
```
Could not find a package configuration file provided by "mavros"
Could not find a package configuration file provided by "dynamic_reconfigure"
```

### Root Cause
- CMakeLists.txt uses `find_package(catkin REQUIRED COMPONENTS mavros dynamic_reconfigure ...)`
- Packages are in `package.xml` as `build_depend` but NOT in builder stage apt
- rosdep install found ROS packages but they depend on system libraries

### Solution Applied
**Scan all CMakeLists.txt files for missing ROS packages:**

```bash
# Find all find_package(catkin ...) entries
grep -h "find_package(catkin" /catkin_ws/src/*/CMakeLists.txt | \
    sed 's/.*COMPONENTS//' | tr ' ' '\n' | grep -v '^)' | sort -u

# Add these to Dockerfile apt install:
# ros-noetic-mavros
# ros-noetic-dynamic-reconfigure
# ros-noetic-visualization-msgs
```

### Prevention Checklist
- [ ] Scan ALL `CMakeLists.txt` files for `find_package(catkin REQUIRED COMPONENTS ...)`
- [ ] Extract component names and prefix with `ros-<distro>-`
- [ ] Install in Dockerfile builder apt before rosdep
- [ ] Verify with: `dpkg -l | grep ros-noetic-`
- [ ] Test CMake configuration: `catkin config && catkin build`

### Diagnosis Commands
```bash
# List all catkin components used
find /catkin_ws/src -name "CMakeLists.txt" -exec grep -h "find_package(catkin" {} \; | \
    sed 's/.*COMPONENTS//' | tr ' ' '\n' | sort -u | head -20

# Check which are installed
for pkg in mavros dynamic_reconfigure visualization_msgs; do
  dpkg -l | grep ros-noetic-$pkg && echo "✓ $pkg" || echo "✗ $pkg"
done
```

---

## 5. HIGH ISSUE: Build vs Runtime Dependency Separation

### Problem
Multi-stage Dockerfile was copying entire `build/` directory to runtime stage:
```dockerfile
COPY --from=builder /catkin_ws/build /catkin_ws/build     # WRONG!
```

### Impact
- Build artifacts (CMake objects, test executables) wasted ~500 MB per image
- Runtime image bloated with unnecessary build tools
- Slowed down image transfers and deployments

### Solution Applied
**Only copy install and src directories to runtime:**

```dockerfile
# Stage 2: Runtime
FROM ros:noetic-ros-base-focal AS runtime

# Copy ONLY install and src (NOT build)
COPY --from=builder /catkin_ws/install /catkin_ws/install
COPY --from=builder /catkin_ws/src /catkin_ws/src
# DO NOT copy: /catkin_ws/build

# Copy compiled libraries separately for external dependencies
COPY --from=builder /usr/local/lib/libceres* /usr/local/lib/
COPY --from=builder /usr/local/lib/libgtsam* /usr/local/lib/
COPY --from=builder /usr/local/lib/libmetis* /usr/local/lib/
RUN ldconfig
```

### Prevention Checklist
- [ ] Use multi-stage Docker builds (builder + runtime)
- [ ] Only copy `/install` and `/src` from builder
- [ ] Selectively copy external library installations
- [ ] Measure final image size: `docker images`
- [ ] Compare: `docker history image_name`

### Diagnosis Commands
```bash
# Check image layers and sizes
docker history slam_integration:latest

# Check directory sizes in builder vs runtime
docker run --rm slam_integration:latest du -sh /catkin_ws/*
# Should show: install (~200MB), src (~100MB), NO build/

# Compare image sizes
docker images | grep slam_integration
# Should be <5GB (not >6GB)
```

---

## 6. MEDIUM ISSUE: Container-Aware Path Detection

### Problem
Preflight check script had hardcoded paths:
```bash
WS_ROOT="/home/dev/slam_ws"  # Works on host, NOT in container!
```

Inside container, this path doesn't exist:
```
ERROR: /home/dev/slam_ws/devel/setup.bash: No such file or directory
```

### Solution Applied
**Add container detection logic:**

```bash
#!/bin/bash

# Detect if running in container
if [ -f /.dockerenv ]; then
    WS_ROOT="/catkin_ws"  # Container workspace
else
    WS_ROOT="/home/dev/slam_ws"  # Host workspace
fi

# Now safe to use:
source ${WS_ROOT}/devel/setup.bash
```

### Prevention Checklist
- [ ] All scripts should detect container environment
- [ ] Use `[ -f /.dockerenv ]` to detect Docker
- [ ] Provide fallback paths for both host and container
- [ ] Test scripts in BOTH environments
- [ ] Document expected paths in comments

### Diagnosis Commands
```bash
# Check if running in container
[ -f /.dockerenv ] && echo "IN CONTAINER" || echo "ON HOST"

# Find all hardcoded paths in scripts
grep -r "/home/dev" /home/dev/slam-agent/scripts/ | grep -v ".md"

# Test script in container
docker compose run --rm slam-system bash -c "source preflight_check.sh"
```

---

## 7. MEDIUM ISSUE: Launch File Selection

### Problem
Container `slam_launch` service was using wrong launch file:
```dockerfile
roslaunch fast_lio mapping_ouster64.launch  # Incomplete - only FAST-LIO
```

Should launch complete integration:
```dockerfile
roslaunch orin_slam_integration master.launch  # Complete system
```

### Impact
- Only FAST-LIO runs, not full integration with MAVROS, STD, vision bridge
- MAVROS connection not established
- Vision odometry not sent to flight controller

### Solution Applied
**Verify launch file targets entire integration:**

```dockerfile
CMD ["bash", "-c", "source /root/slam_ws/devel/setup.bash && \
     roslaunch orin_slam_integration master.launch enable_rviz:=false"]
```

### Prevention Checklist
- [ ] Identify the "orchestrator" launch file (usually `master.launch`)
- [ ] Verify it includes all necessary sub-launches
- [ ] Test locally: `roslaunch orin_slam_integration master.launch`
- [ ] Check all nodes start: `rosnode list`
- [ ] Verify all topics appear: `rostopic list`

### Diagnosis Commands
```bash
# Find orchestrator launch file
find /catkin_ws/src -name "master.launch" -o -name "main.launch"

# Check what it includes
grep "include" /catkin_ws/src/orin_slam_integration/launch/master.launch

# Test launch file syntax
roslaunch orin_slam_integration master.launch --screen

# Verify all nodes
rosnode list | wc -l
```

---

## 8. MEDIUM ISSUE: Health Check Implementation

### Problem
Docker health check was attempting to list ROS nodes:
```dockerfile
healthcheck:
  test: ["CMD-SHELL", "source /opt/ros/noetic/setup.bash && rosnode list"]
```

This fails because `rosnode list` requires a running ROS Master, which the healthcheck shouldn't assume exists.

### Solution Applied
**Use package discovery instead:**

```dockerfile
healthcheck:
  test: ["CMD-SHELL", "source /opt/ros/noetic/setup.bash && rospack find fast_lio >/dev/null"]
  interval: 30s
  timeout: 10s
  retries: 3
  start_period: 10s
```

`rospack find` only checks that ROS environment is set up correctly, doesn't require ROS Master.

### Prevention Checklist
- [ ] Health checks should NOT depend on running ROS Master
- [ ] Use `rospack find <package>` to verify ROS environment
- [ ] Test health check: `docker compose exec slam_launch bash -c "rospack find fast_lio"`
- [ ] Monitor health status: `docker compose ps` shows `(healthy)` or `(unhealthy)`

### Diagnosis Commands
```bash
# Check container health status
docker compose ps
# Column: STATUS should show (healthy) or (unhealthy)

# Run health check manually
docker compose exec -T slam_launch bash -c \
  "source /opt/ros/noetic/setup.bash && rospack find fast_lio >/dev/null && echo OK"

# Check health check history
docker inspect slam_launch | grep -A 20 "Health"
```

---

## 9. MEDIUM ISSUE: Configuration Mount Missing

### Problem
`orin_slam_integration` configuration files weren't mounted in docker-compose.yml, making them immutable without rebuilding the entire image.

Launch files and config YAMLs couldn't be modified at runtime.

### Solution Applied
**Add volume mounts for configuration:**

```yaml
volumes:
  # Make configs editable without rebuild
  - ./src/orin_slam_integration/launch:/catkin_ws/src/orin_slam_integration/launch:rw
  - ./src/orin_slam_integration/config:/catkin_ws/src/orin_slam_integration/config:rw

  # Also mount other package configs
  - ./src/FAST_LIO_SLAM/FAST-LIO/config:/catkin_ws/src/FAST_LIO_SLAM/FAST-LIO/config:rw
  - ./src/FAST_LIO_SLAM/FAST-LIO/launch:/catkin_ws/src/FAST_LIO_SLAM/FAST-LIO/launch:rw
```

### Prevention Checklist
- [ ] Identify all editable configuration files (YAML, launch, params)
- [ ] Mount with `:rw` permission to allow modification
- [ ] Document which configs are runtime-modifiable
- [ ] Test: `docker compose run --rm slam-system`
  - Modify config
  - Reload in ROS without rebuild
- [ ] Use `rosparam load` for parameter updates

### Diagnosis Commands
```bash
# Check mounted volumes
docker compose config | grep -A 20 "volumes:"

# Verify mount inside container
docker compose exec slam_launch ls -la /catkin_ws/src/orin_slam_integration/

# Test modification and reload
docker compose exec slam_launch bash -c \
  "nano /catkin_ws/src/orin_slam_integration/config/fast_lio_config.yaml"
```

---

## Build Iteration Timeline

### Iteration 1: Initial Build Failure
- **Status**: ❌ Failed
- **Errors**:
  - Ceres 1.14 API mismatch (std_detector)
  - Missing spdlog, jsoncpp-dev (ouster-ros)
  - Missing build dependencies
- **Duration**: ~30 minutes (identify issues)

### Iteration 2: Added Ceres Pre-installation
- **Status**: ⚠️ Partial Success
- **Fixed**: std_detector now builds
- **New Error**: fast_lio message generation timing
- **Duration**: ~40 minutes (Ceres build)

### Iteration 3: Fixed Message Generation + Added ROS Packages
- **Status**: ✅ Complete Success
- **Fixed**: Message generation ordering
- **Added**: All missing ROS packages
- **Result**: All 6 packages compiled successfully
- **Duration**: ~45 minutes (full build)

**Total Build Time**: ~115 minutes (first time, from scratch)

---

## Best Practices Summary

### For Future Docker Builds

1. **Dependency Analysis Phase (Before Dockerfile Creation)**
   ```bash
   # Scan all packages for requirements
   grep -r "build_depend\|find_package" /catkin_ws/src/*/package.xml /catkin_ws/src/*/CMakeLists.txt
   ```

2. **Version Verification Phase**
   - Check all C++ libraries against package documentation
   - Verify ROS package versions match distro
   - List any special version requirements (e.g., Ceres 2.1+)

3. **Dockerfile Design Phase**
   - Use multi-stage builds (builder + runtime)
   - Pre-install special version dependencies first
   - Install all build_depend packages explicitly
   - Copy only necessary artifacts to runtime

4. **Build & Test Phase**
   - Build locally first with `catkin build --verbose`
   - Save log output to identify issues early
   - Test in Docker with progress output: `docker compose build --progress=plain`

5. **Container Validation Phase**
   - Run preflight checks
   - Verify all nodes start: `rosnode list`
   - Check all topics appear: `rostopic list`
   - Monitor resource usage: `docker stats`

### For Future Troubleshooting

If a new build fails, check in this order:

1. **C++ API/Version Issues**
   - Check pkg-config: `pkg-config --modversion <package>`
   - Review package documentation for API requirements
   - Check if pre-installation needed

2. **Missing Build Dependencies**
   - Run `grep "<build_depend>" package.xml`
   - Verify each is in `apt-get install`
   - Check with `dpkg -l | grep <package>`

3. **Message Generation Timing**
   - Check if `add_message_files()` exists
   - Verify `add_dependencies(target ${PROJECT_NAME}_generate_messages_cpp)`
   - Inspect `devel/include/` for generated headers

4. **ROS Package Dependencies**
   - Scan CMakeLists.txt for `find_package(catkin REQUIRED COMPONENTS ...)`
   - Verify all components installed: `apt-cache search ros-<distro>-<pkg>`
   - Check with `rospack find <package>`

5. **Runtime Issues**
   - Check container detection logic if using scripts
   - Verify configuration mounts are in place
   - Test health checks: `docker compose ps`

---

## Tools and Scripts Created

### `/home/dev/slam-agent/scripts/docker_diagnostics.py`
Automated diagnostic checking for all issues:
- Image presence
- Container status
- ROS environment
- Node status
- Topic publishing
- Package discovery
- Launch file verification
- Odometry production

### `/home/dev/slam-agent/scripts/deploy_docker_slam.sh`
Deployment and testing automation:
- Build image with progress monitoring
- Start/stop containers
- Run full diagnostic suite
- Generate reports

### Updated Documentation
- **DOCKER_OPERATIONS_RUNBOOK.md** - Complete ROS1 operations guide
- **DOCKER_OPERATIONS_RUNBOOK_ROS2.md** - ROS2 Humble equivalent
- **DOCKER_MULTI_ROS_ARCHITECTURE.md** - Multi-version support design

---

## Metrics and Performance

| Metric | Value |
|--------|-------|
| Final image size | 5.11 GB |
| Build time (first) | ~90-120 min |
| Build time (cached) | ~5 min |
| Ceres build time | 20-30 min |
| GTSAM build time | 15-25 min |
| catkin build time | 20-30 min |
| CPU cores used | 6 (Jetson Orin NX) |
| RAM required | 6+ GB |

---

## Validation Checklist for Future Builds

Before considering a Docker build complete:

- [ ] All 6 packages built successfully (or expected packages for config)
- [ ] No CMake warnings about missing dependencies
- [ ] Health check passes: `docker compose ps` shows `(healthy)`
- [ ] Container starts: `docker compose up -d slam_launch`
- [ ] All nodes appear: `docker compose exec -T slam_launch bash -c "source /opt/ros/noetic/setup.bash && rosnode list" | wc -l` > 5
- [ ] SLAM odometry publishing: `docker compose exec -T slam_launch bash -c "source /opt/ros/noetic/setup.bash && timeout 5 ros2 topic hz /Odometry 2>/dev/null || timeout 5 rostopic hz /Odometry"`
- [ ] Vision pose to MAVROS: `docker compose exec -T slam_launch bash -c "source /opt/ros/noetic/setup.bash && rostopic list | grep mavros"`
- [ ] Configuration files mounted and editable
- [ ] Preflight check passes: `docker compose exec slam_launch /usr/local/bin/preflight_check.sh`
- [ ] Resource limits applied: `docker compose config | grep -A 5 "deploy"`

---

## References

- **Ceres Documentation**: https://ceres-solver.org/
- **GTSAM Documentation**: https://gtsam.org/
- **ROS Build System**: http://docs.ros.org/en/api/catkin/html/
- **Docker Best Practices**: https://docs.docker.com/develop/dev-best-practices/
- **Multi-stage Builds**: https://docs.docker.com/build/building/multi-stage/

---

**Status**: ✅ All critical build issues resolved and documented
**Date Completed**: 2026-02-08
**Applies To**: All future Docker deployments for SLAM integration systems
**Review Cycle**: Bi-annually or when major dependencies updated
