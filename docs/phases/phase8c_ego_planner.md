# Phase 8C: EGO-Planner-v2 / FUEL (3D Trajectory Planning)

## EGO-Planner-v2 (Obstacle Avoidance)

EGO-Planner-v2 (ZJU-FAST-Lab) is a gradient-based local trajectory planner with built-in ESDF. Good for obstacle avoidance without needing a separate map server.

### Install

```bash
# Dependencies
sudo apt install libeigen3-dev libpcl-dev ros-noetic-pcl-ros  # ROS 1
sudo apt install libeigen3-dev libpcl-dev ros-humble-pcl-ros  # ROS 2

# OSQP (required by EGO-Planner)
git clone --recursive https://github.com/osqp/osqp && cd osqp
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install

# EGO-Planner-v2
cd ~/catkin_ws/src   # or ~/ros2_ws/src for ROS 2
git clone https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

Via MCP:
```python
run_install_script("install_path_planner", "ego_planner ~/catkin_ws ROS1")
```

### Config (`config/ego_planner.yaml`)

```yaml
ego_planner:
  # Map
  map_size_x: 20.0       # meters, match geofence
  map_size_y: 20.0
  map_size_z:  4.0
  resolution:  0.15      # voxel size (increase to save RAM)

  # Planning
  planning_horizon: 7.5  # meters lookahead
  max_vel: 1.5           # m/s (match planner_config.yaml)
  max_acc: 2.0           # m/s²
  max_jerk: 4.0          # m/s³

  # Safety
  min_dist_from_obs: 0.5  # meters clearance
  min_dist_from_goal: 0.3

  # Replan
  replan_thresh: 0.5     # meters off trajectory before replanning
  replan_time: 1.0       # seconds between periodic replans
```

### Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `/slam/cloud_registered` | `sensor_msgs/PointCloud2` |
| Input | `/slam/odometry` | `nav_msgs/Odometry` |
| Input | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` |
| Output | `/planning/trajectory` | EGO trajectory message |
| Output | `/planning/pos_cmd` | position setpoints |

Wire `/planning/pos_cmd` to the planner bridge below.

---

## FUEL (Frontier-Based Exploration)

FUEL (HKUST, Science Robotics 2021) adds frontier-based exploration on top of trajectory planning. Use when the drone should autonomously explore unknown environments.

**ROS 1 Noetic only.** Requires OctoMap (install via Phase 8 main doc or MCP).

```bash
cd ~/catkin_ws/src
git clone --depth 1 --recurse-submodules https://github.com/HKUST-Aerial-Robotics/FUEL.git
sudo apt install libnlopt-dev libnlopt-cxx-dev
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

Via MCP:
```python
run_install_script("install_path_planner", "fuel ~/catkin_ws ROS1")
```

---

## Planner → MAVROS Bridge

EGO-Planner and FUEL need a bridge node to translate planner output into MAVROS setpoints. (SUPER has its own bridge — see Phase 8B.)

### Safety responsibilities

- Rate-limit to 10-20 Hz
- Pause setpoints if SLAM odometry is stale
- Only publish setpoints when ArduPilot is in GUIDED mode
- Clamp to geofence

### Bridge node template (`scripts/planner_bridge.py`)

```python
#!/usr/bin/env python3
"""Bridge: converts planner position commands → /mavros/setpoint_position/local.
Handles SLAM health, mode check, rate limiting, geofence clamping."""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import math

class PlannerBridge:
    def __init__(self):
        rospy.init_node('planner_bridge')

        cfg = rospy.get_param('~waypoint_nav', {})
        self.geo = cfg.get('geofence', {})
        self.slam_timeout = cfg.get('slam_timeout', 1.0)
        self.rate_hz = cfg.get('setpoint_rate', 10.0)

        self.target = None
        self.last_slam_time = None
        self.guided = False

        self.pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        rospy.Subscriber('/planning/pos_cmd', PoseStamped, self._cmd_cb)
        rospy.Subscriber('/slam/odometry', Odometry, self._slam_cb)
        rospy.Subscriber('/mavros/state', State, self._state_cb)

    def _cmd_cb(self, msg):
        p = msg.pose.position
        if not self._in_geofence(p.x, p.y, p.z):
            rospy.logwarn_throttle(2.0, 'Planner command outside geofence, ignored.')
            return
        self.target = msg

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
        while not rospy.is_shutdown():
            if self.target and self.guided and self._slam_healthy():
                self.target.header.stamp = rospy.Time.now()
                self.pub.publish(self.target)
            elif not self.guided:
                rospy.logwarn_throttle(10.0, 'Not in GUIDED mode, setpoints suppressed.')
            elif not self._slam_healthy():
                rospy.logwarn_throttle(2.0, 'SLAM unhealthy, setpoints suppressed.')
            rate.sleep()

if __name__ == '__main__':
    PlannerBridge().run()
```

### Coordinate frame reference

| Frame | Convention | Notes |
|---|---|---|
| SLAM output | ENU (East-North-Up) | map frame |
| MAVROS local | ENU | `/mavros/setpoint_position/local` is ENU |
| ArduPilot internal | NED | MAVROS handles ENU→NED conversion automatically |

No manual ENU↔NED conversion needed when publishing to MAVROS setpoint topics.
