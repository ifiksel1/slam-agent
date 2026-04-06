# Phase 8B: SUPER + ROG-Map (Safety-Assured High-Speed Navigation)

SUPER (HKU-Mars, Science Robotics 2025) is a complete local planner with dual-trajectory safety. It generates two trajectories simultaneously — one optimizing speed through unknown space, one staying in known-free space as backup. If the aggressive trajectory becomes unsafe, it instantly switches to the backup.

**Key specs:** Up to 20 m/s demonstrated, 35x lower failure rate than baselines, designed for FAST-LIO output.

---

## ROG-Map (Bundled Map Backend)

ROG-Map (IROS 2024) is a robocentric sliding-window occupancy grid that **replaces OctoMap** for real-time drone navigation. It's bundled inside SUPER — no separate install needed.

**Advantages over OctoMap:**
- Zero-copy map sliding (constant memory as drone moves)
- Built-in ESDF for gradient-based planners
- Built-in obstacle inflation (configurable multi-resolution)
- Built-in frontier extraction (for exploration)
- Designed for FAST-LIO output format

**Trade-off:** Local map only (no persistent global map). Use OctoMap if you need post-mission global reconstruction.

### ROG-Map Standalone Install (without SUPER)

```bash
cd ~/catkin_ws/src   # ROS 1
git clone https://github.com/hku-mars/ROG-Map.git

# ARM64 (Jetson): delete MARSIM simulator examples (OpenGL x86 paths)
rm -rf ROG-Map/examples/MARSIM

# Dependencies
sudo apt install libeigen3-dev libdw-dev ros-noetic-rosfmt

cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

**Important:** Deactivate conda before building (`conda deactivate`).

### ROG-Map Config (`config/rog_map.yaml`)

```yaml
rog_map:
  resolution: 0.1              # meters (0.1 indoor, 0.2-0.3 outdoor/high-speed)
  inflation_resolution: 0.2    # coarser for inflation layer
  inflation_step: 2            # number of inflation layers
  map_size: [40.0, 40.0, 6.0]  # local map extent in meters

  map_sliding:
    enable: true               # robocentric sliding window

  virtual_ceil_height: 4.0     # max altitude in local map
  virtual_ground_height: -0.3  # min altitude (slightly below ground)

  raycasting:
    enable: false              # counter-based by default (faster)

  cloud_topic: /cloud_registered      # world-frame point cloud from FAST-LIO
  odom_topic: /Odometry               # odometry from FAST-LIO

  point_filt_num: 1            # temporal downsample (1 = use every point)
  intensity_thresh: -10        # dust/noise filtering (-10 = disabled)
```

### ROG-Map Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `/cloud_registered` | `sensor_msgs/PointCloud2` (must be in world frame) |
| Input | `/Odometry` | `nav_msgs/Odometry` |
| Output | `~rog_map/occ` | Occupied voxels (PointCloud2) |
| Output | `~rog_map/inf_occ` | Inflated occupied voxels (PointCloud2) |
| Output | `~rog_map/frontier` | Frontier points (PointCloud2) |
| Output | `~rog_map/esdf` | ESDF field (PointCloud2) |

**Critical:** Point cloud MUST be in world frame. FAST-LIO publishes `/cloud_registered` in world frame by default.

### ROG-Map C++ API (for custom planners)

```cpp
bool isOccupied(Eigen::Vector3d pos);
bool isKnownFree(Eigen::Vector3d pos);
bool isLineFree(Eigen::Vector3d start, Eigen::Vector3d end);
bool isOccupiedInflate(Eigen::Vector3d pos);
bool isUnknownInflate(Eigen::Vector3d pos);
double getDistance(Eigen::Vector3d pos);  // ESDF query
```

---

## SUPER Install

```bash
# ROS 1 Noetic (stable)
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/hku-mars/SUPER.git
cd SUPER && bash scripts/select_ros_version.sh ROS1

# Dependencies
sudo apt install libeigen3-dev libdw-dev libncurses5-dev libncursesw5-dev \
                 ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-pcl-ros \
                 ros-noetic-rosfmt
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen  # required by SUPER

# ARM64 (Jetson): remove MARSIM simulator (OpenGL, not needed on real hardware)
rm -rf mars_uav_sim/marsim_render

cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

```bash
# ROS 2 Foxy (experimental, unstable)
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/hku-mars/SUPER.git
cd SUPER && bash scripts/select_ros_version.sh ROS2
cd ~/ros2_ws && colcon build --symlink-install
```

**Note:** ROS 2 support is WIP — ROS 1 Noetic is the stable path. For trajectory tracking, use the simple position bridge below (conservative speeds) or the OMMPC controller (aggressive tracking, see below).

Via MCP:
```python
run_install_script("install_path_planner", "super ~/catkin_ws ROS1")
```

---

## SUPER Config (`config/super_planner.yaml`)

```yaml
super_planner:
  planning_horizon: 7.0       # meters lookahead (7 for indoor, up to 30 for outdoor)
  receding_dis: 4.0            # receding horizon distance
  robot_r: 0.3                 # collision radius (meters, match your drone)
  replan_rate: 15.0            # Hz, replanning frequency
  backup_traj_en: true         # dual-trajectory safety (KEEP THIS ON)

  max_vel: 2.0                 # m/s (start low, increase after validation)
  max_acc: 3.0                 # m/s²
  max_jerk: 20.0               # m/s³

  yaw_mode: 1                  # 1=heading to velocity, 2=heading to goal

  mpc_horizon: 15
  replan_forward_dt: 0.1

  # ROG-Map (bundled)
  rog_map:
    resolution: 0.15
    inflation_resolution: 0.2
    inflation_step: 2
    map_size: [40.0, 40.0, 6.0]
    map_sliding:
      enable: true
    cloud_topic: /cloud_registered
    odom_topic: /Odometry
```

---

## SUPER Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Input | `/cloud_registered` | `sensor_msgs/PointCloud2` | World-frame cloud from FAST-LIO |
| Input | `/Odometry` (or `/lidar_slam/odom`) | `nav_msgs/Odometry` | From FAST-LIO |
| Input | `/planning/click_goal` or RViz "G" key | `geometry_msgs/PoseStamped` | Goal position |
| Output | `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | Position + velocity + acceleration + yaw |
| Output | `/planning_cmd/poly_traj` | `quadrotor_msgs/PolynomialTrajectory` | Full polynomial trajectory |

---

## SUPER → MAVROS Bridge (Simple, for <3 m/s)

Converts `PositionCommand` to MAVROS position setpoints. ArduPilot's internal position controller handles tracking.

```python
#!/usr/bin/env python3
"""Bridge SUPER PositionCommand → MAVROS setpoint_position/local."""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import math

# quadrotor_msgs is bundled with SUPER
from quadrotor_msgs.msg import PositionCommand

class SuperBridge:
    def __init__(self):
        rospy.init_node('super_mavros_bridge')

        cfg = rospy.get_param('~waypoint_nav', {})
        self.geo = cfg.get('geofence', {})
        self.slam_timeout = cfg.get('slam_timeout', 1.0)
        self.rate_hz = cfg.get('setpoint_rate', 20.0)

        self.target = None
        self.last_slam_time = None
        self.guided = False

        self.pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self._cmd_cb)
        rospy.Subscriber('/Odometry', Odometry, self._slam_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)

    def _cmd_cb(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position = msg.position
        yaw = msg.yaw
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)

        p = msg.position
        if not self._in_geofence(p.x, p.y, p.z):
            rospy.logwarn_throttle(2.0, 'SUPER cmd outside geofence, ignored.')
            return
        self.target = ps

    def _slam_cb(self, _):
        self.last_slam_time = rospy.Time.now()

    def _state_cb(self, msg):
        self.guided = (msg.mode == 'GUIDED')

    def _slam_healthy(self):
        if self.last_slam_time is None:
            return False
        return (rospy.Time.now() - self.last_slam_time).to_sec() < self.slam_timeout

    def _in_geofence(self, x, y, z):
        return (self.geo.get('x_min', -999) <= x <= self.geo.get('x_max', 999) and
                self.geo.get('y_min', -999) <= y <= self.geo.get('y_max', 999) and
                self.geo.get('z_min', 0.0)  <= z <= self.geo.get('z_max', 999))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo('SUPER→MAVROS bridge ready.')
        while not rospy.is_shutdown():
            if self.target and self.guided and self._slam_healthy():
                self.target.header.stamp = rospy.Time.now()
                self.pub.publish(self.target)
            elif not self.guided:
                rospy.logwarn_throttle(10.0, 'Not in GUIDED — setpoints suppressed.')
            elif not self._slam_healthy():
                rospy.logwarn_throttle(2.0, 'SLAM unhealthy — setpoints suppressed.')
            rate.sleep()

if __name__ == '__main__':
    SuperBridge().run()
```

---

## OMMPC Controller (Advanced, for >3 m/s)

For aggressive maneuvers, the simple bridge is insufficient — ArduPilot's internal position controller can't track fast trajectories accurately. OMMPC (On-Manifold Model Predictive Controller) solves a QP at 100 Hz on the SO(3) manifold, outputting body-rate + thrust commands directly to MAVROS.

**Repo:** `https://github.com/OliverWu515/ommpc_controller` (standalone re-implementation based on SUPER's control derivation)

**Pipeline comparison:**
```
Simple:  SLAM → SUPER → bridge → /mavros/setpoint_position/local → ArduPilot position controller
OMMPC:   SLAM → SUPER → OMMPC  → /mavros/setpoint_raw/attitude   → ArduPilot rate controller
```

### Install

```bash
cd ~/catkin_ws/src

# OSQP v0.6.3 (required by OMMPC)
git clone --branch v0.6.3 --recurse-submodules https://github.com/osqp/osqp.git
mkdir -p osqp/build && cd osqp/build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install
cd ~/catkin_ws/src

# OMMPC controller
git clone https://github.com/OliverWu515/ommpc_controller.git

# Dependencies
sudo apt install ros-noetic-ddynamic-reconfigure

cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

Via MCP:
```python
run_install_script("install_path_planner", "ommpc ~/catkin_ws ROS1")
```

### Config (`config/ommpc_params.yaml`)

```yaml
ommpc:
  hover_percentage: 0.70        # hover_thrust / max_thrust — TUNE FOR YOUR DRONE
  takeoff_height: 1.0           # meters
  takeoff_land_speed: 0.3       # m/s
  use_fix_yaw: false            # false to track SUPER's yaw commands

  MPC_params:
    step_T: 0.01                # 100 Hz control rate
    Q_pos_xy: 1000.0            # horizontal position weight
    Q_pos_z: 800.0              # vertical position weight
    Q_attitude_rp: 40.0         # roll/pitch weight
    Q_attitude_yaw: 40.0        # yaw weight
    Q_velocity: 20.0            # velocity tracking weight
    R_thrust: 0.5               # thrust penalty
    R_pitchroll: 1.2            # roll/pitch rate penalty
    R_yaw: 0.6                  # yaw rate penalty
    max_bodyrate_xy: 6.0        # rad/s max roll/pitch rate
    max_bodyrate_z: 4.0         # rad/s max yaw rate
    min_thrust: 1.0             # m/s² min thrust (as accel)
    max_thrust: 30.0            # m/s² max thrust (as accel)
```

### OMMPC Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Input | `/mavros/local_position/odom` | `nav_msgs/Odometry` | From MAVROS |
| Input | `/mavros/imu/data` | `sensor_msgs/Imu` | For thrust estimation |
| Input | `/mavros/state` | `mavros_msgs/State` | Armed/mode check |
| Input | `/drone_0_planning/trajectory` | `traj_utils/PolyTraj` | Trajectory from planner |
| Output | `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Body rates + thrust |

### Required Adaptations for ArduPilot

OMMPC is written for PX4 OFFBOARD mode. For ArduPilot you must modify:

1. **Mode check:** Replace `MODE_PX4_OFFBOARD` with ArduPilot `GUIDED` mode in the FSM (`ommpc_controller.cpp`)
2. **Message adapter:** SUPER publishes `quadrotor_msgs/PolynomialTrajectory`, OMMPC subscribes to `traj_utils/PolyTraj`. Write a thin adapter node or modify OMMPC to consume `quadrotor_msgs/PositionCommand` from `/planning/pos_cmd` (simpler — SUPER publishes this at 100 Hz with full pos/vel/acc/jerk + yaw).
3. **Thrust tuning:** Measure `hover_percentage` for your drone (hover throttle / max throttle). Start conservative (0.5-0.7) and tune with hover tests.

### When to use OMMPC vs simple bridge

| | Simple Bridge | OMMPC |
|---|---|---|
| **Max speed** | ~2 m/s (ArduPilot position controller limits) | 5-20 m/s (direct body-rate control) |
| **Tracking accuracy** | Adequate for inspection/survey | Required for aggressive maneuvers |
| **Complexity** | Minimal (Python script) | Requires ArduPilot mods + tuning |
| **Safety** | ArduPilot handles stabilization | OMMPC handles stabilization (more risk) |
| **Recommendation** | Start here | Upgrade after validating SUPER at low speeds |

**Start with the simple bridge.** Only switch to OMMPC when you've validated SUPER at conservative speeds and need higher performance.

---

## ARM64 / Jetson Notes

- SUPER core is pure C++/Eigen/PCL — no CUDA, builds on ARM64
- Delete `mars_uav_sim/marsim_render` (OpenGL simulator, not needed on real hardware)
- Tested on Intel NUC by the authors; Jetson Orin NX is untested but feasible
- Monitor RAM: SUPER + FAST-LIO + ROG-Map together on 8GB shared RAM is tight — reduce `map_size` and increase `resolution` if needed
- OMMPC is also pure C++/Eigen, no architecture-specific code
