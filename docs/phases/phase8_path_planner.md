# Phase 8: Autonomous Path Planning

Adds a path planner on top of the working SLAM → vision_to_mavros → MAVROS → ArduPilot pipeline. The drone already holds position; this phase gives it autonomous navigation.

---

## Planner Selection

Choose based on mission type. Simpler is better on Jetson Orin NX 8GB.

| Mission Type | Planner | Sub-file | Compute | Notes |
|---|---|---|---|---|
| **Waypoint nav** (inspection, known routes) | Waypoint nav node | [phase8a](phase8a_waypoint_nav.md) | Low | No map needed. Simplest path to flight. |
| **Fast obstacle avoidance** (safety-critical) | SUPER + ROG-Map | [phase8b](phase8b_super.md) | Medium | Dual-trajectory safety, up to 20 m/s. Best for FAST-LIO. |
| **Exploration** (unknown environments) | EGO-Planner-v2 | [phase8c](phase8c_ego_planner.md) | Medium | 3D trajectory + obstacle avoidance. |
| **Corridor / tunnel** | Nav2 | [phase8d](phase8d_nav2.md) | Medium | 2.5D costmap. ROS 2 only. |
| **Full 3D exploration** | FUEL + OctoMap | [phase8c](phase8c_ego_planner.md) | High | Frontier-based. Most compute-intensive. |

**Recommendation for FAST-LIO users:** Start with waypoint nav (8a) for initial validation, then upgrade to SUPER (8b) for autonomous missions.

**Load only the sub-file matching the user's chosen planner.** Return here for shared content (OctoMap, ArduPilot params, testing, control script).

---

## OctoMap Setup (Required for EGO-Planner, FUEL, Nav2)

OctoMap converts the SLAM point cloud into a 3D occupancy grid. Not needed for waypoint nav or SUPER (which bundles ROG-Map).

### Install

```bash
# Via MCP:
run_install_script("install_path_planner", "octomap ~/slam_ws ROS2")

# ROS 1 Noetic
sudo apt install ros-noetic-octomap ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins

# ROS 2 (replace ${ROS_DISTRO} with humble, jazzy, etc.)
sudo apt install ros-${ROS_DISTRO}-octomap ros-${ROS_DISTRO}-octomap-mapping
```

### Config (`config/octomap.yaml`)

```yaml
resolution: 0.1           # meters per voxel (0.1 indoor, 0.25 outdoor)
frame_id: map
base_frame_id: base_link
filter_ground: true

sensor_model:
  max_range: 15.0         # meters, clip beyond this (reduce CPU)
  hit:  0.7
  miss: 0.4
  min:  0.12
  max:  0.97
```

Verify: `rostopic hz /octomap_binary` should publish at ~1 Hz.

---

## ArduPilot Parameters for Path Planning

Set these in addition to the base SLAM integration params from Phase 3.

```
WPNAV_SPEED,200         # cm/s horizontal speed (200=2m/s, increase when confident)
WPNAV_SPEED_UP,200      # cm/s ascent speed
WPNAV_SPEED_DN,100      # cm/s descent speed
WPNAV_ACCEL,200         # cm/s² horizontal acceleration
WPNAV_ACCEL_Z,100       # cm/s² vertical acceleration
LOIT_SPEED,300          # cm/s loiter max lateral speed
LOIT_ACC_MAX,150        # cm/s² loiter acceleration
WP_YAW_BEHAVIOR,1       # 0=never yaw, 1=face next waypoint, 2=face next except RTL
PILOT_SPEED_UP,250      # cm/s pilot climb rate (safety override)
PSC_JERK_XY,5.0         # m/s³ position controller jerk limit
PSC_JERK_Z,5.0          # m/s³ vertical jerk limit
```

---

## Control Script (`scripts/planner.sh`)

```bash
#!/bin/bash
# scripts/planner.sh — start/stop/restart/status/logs for planner bridge

CONTAINER="planner"
IMAGE="slam-gpu:latest"
LOG_DIR="$HOME/slam-agent/logs"
LAUNCH_CMD="roslaunch slam_planner planner_bridge.launch"
# For ROS 2: LAUNCH_CMD="ros2 launch slam_planner planner_bridge.py"

mkdir -p "$LOG_DIR"

case "$1" in
  start)
    echo "Starting planner bridge..."
    docker run -d --rm \
      --name "$CONTAINER" \
      --network host \
      --env ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
      --env ROS_IP="${ROS_IP:-127.0.0.1}" \
      -v "$(pwd)/config:/ws/config:ro" \
      "$IMAGE" \
      bash -c "source /opt/ros/noetic/setup.bash && $LAUNCH_CMD" \
      > "$LOG_DIR/planner.log" 2>&1
    echo "Planner started (container: $CONTAINER)"
    ;;
  stop)
    docker stop "$CONTAINER" 2>/dev/null && echo "Planner stopped." || echo "Not running."
    ;;
  restart)
    $0 stop; sleep 1; $0 start
    ;;
  status)
    docker inspect --format='{{.State.Status}}' "$CONTAINER" 2>/dev/null || echo "not running"
    ;;
  logs)
    docker logs -f "$CONTAINER"
    ;;
  *)
    echo "Usage: $0 {start|stop|restart|status|logs}"
    exit 1
    ;;
esac
```

Via MCP:
```python
control_node("/home/dev/slam-agent", "planner", "start")
control_node("/home/dev/slam-agent", "planner", "status")
control_node("/home/dev/slam-agent", "planner", "logs")
```

---

## Testing Protocol

Run these steps in order. Do not skip to a later step if an earlier one fails.

### Step 1: Simulation (SITL) — if available

```bash
sim_vehicle.py -v ArduCopter --console --map \
  -A "--param ARMING_CHECK=388598" \
  -A "--param EK3_SRC1_POSXY=6"
```

### Step 2: Bench test (props off)

- [ ] SLAM running, odometry publishing at 10-20 Hz
- [ ] Planner bridge node running, `/mavros/setpoint_position/local` publishing at 10 Hz
- [ ] Verify setpoints match expected waypoints
- [ ] Confirm SLAM health check: kill SLAM node, verify setpoints stop within 1 second

### Step 3: Single waypoint hover (props on, tethered)

- [ ] Arm in STABILIZE, take off manually to ~1m
- [ ] Switch to GUIDED
- [ ] Drone holds position within 30 cm for 30 seconds

### Step 4: Two-point path

- [ ] Command waypoint 2m forward, verify smooth transit
- [ ] Check SLAM drift: position should return near origin after round trip

### Step 5: Multi-waypoint mission

- [ ] 4-6 waypoints forming a square (3m x 3m at 1.5m altitude)
- [ ] Verify hold time at each waypoint
- [ ] Verify return-to-start behavior

### Step 6: Obstacle test (EGO-Planner / SUPER / FUEL only)

- [ ] Place known obstacle along planned path
- [ ] Verify planner routes around obstacle with >= 0.5m clearance

### Step 7: SLAM dropout recovery

- [ ] Mid-mission, simulate SLAM loss
- [ ] Verify bridge stops publishing setpoints immediately
- [ ] Drone should enter LOITER automatically

---

## Diagnostic

```python
run_diagnostic("check_path_planner", "--planner waypoint_nav --json")
run_diagnostic("check_path_planner", "--planner super --json")
```

---

## Output

After testing passes, save the planner config to the learning system:

```python
save_known_good_config(fingerprint, configs_json)
commit_learning("validated path planner: <planner_type>")
```
