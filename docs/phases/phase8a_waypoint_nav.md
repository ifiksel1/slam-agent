# Phase 8A: Waypoint Navigation

Lightweight Python node for GUIDED mode waypoint following. No map or planner framework required — just SLAM + MAVROS.

---

## Install

No external dependencies beyond MAVROS (already installed in Phase 4). Place files in your ROS package:

```
slam_planner/
  scripts/
    waypoint_nav.py
  config/
    waypoints.yaml
    planner_config.yaml
```

Via MCP:
```python
run_install_script("install_path_planner", "waypoint_nav ~/slam_ws ROS2")
```

---

## `config/planner_config.yaml`

```yaml
waypoint_nav:
  # Safety limits
  geofence:
    x_min: -20.0    # meters from EKF origin
    x_max:  20.0
    y_min: -20.0
    y_max:  20.0
    z_min:   0.3    # minimum altitude
    z_max:   5.0    # maximum altitude

  # Motion limits
  max_velocity: 1.5         # m/s, horizontal
  max_ascent_rate: 1.0      # m/s
  max_descent_rate: 0.5     # m/s
  position_tolerance: 0.3   # meters, waypoint reached threshold
  yaw_tolerance: 0.1        # radians

  # SLAM health
  slam_timeout: 1.0         # seconds, pause if odometry stops
  setpoint_rate: 10.0       # Hz, rate to publish to MAVROS

  # Behavior
  hold_time_at_waypoint: 2.0   # seconds to hold before next WP
  return_to_start: true        # RTL via planner after mission
```

## `config/waypoints.yaml`

```yaml
# Positions in ENU meters relative to EKF origin
# Set origin with set_origin2.py before flight
waypoints:
  - { x:  0.0, y:  0.0, z: 1.5, yaw: 0.0 }   # takeoff
  - { x:  3.0, y:  0.0, z: 1.5, yaw: 0.0 }
  - { x:  3.0, y:  3.0, z: 1.5, yaw: 1.5708 }
  - { x:  0.0, y:  3.0, z: 1.5, yaw: 3.1416 }
  - { x:  0.0, y:  0.0, z: 1.5, yaw: 0.0 }   # return
```

---

## `scripts/waypoint_nav.py`

```python
#!/usr/bin/env python3
"""
Lightweight waypoint navigator for GUIDED mode.
Requires: SLAM + vision_to_mavros + MAVROS running.
"""

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time

class WaypointNav:
    def __init__(self):
        rospy.init_node('waypoint_nav')

        cfg = rospy.get_param('~waypoint_nav', {})
        self.geo = cfg.get('geofence', {})
        self.max_vel = cfg.get('max_velocity', 1.5)
        self.pos_tol = cfg.get('position_tolerance', 0.3)
        self.hold_time = cfg.get('hold_time_at_waypoint', 2.0)
        self.slam_timeout = cfg.get('slam_timeout', 1.0)
        self.setpoint_rate = cfg.get('setpoint_rate', 10.0)

        waypoints_file = rospy.get_param('~waypoints_file', '')
        with open(waypoints_file) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.current_pose = None
        self.last_slam_time = None
        self.wp_index = 0

        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/planner/status', String, queue_size=1)

        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, self._pose_cb)
        rospy.Subscriber('/slam/odometry',
                         Odometry, self._slam_cb)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _slam_cb(self, msg):
        self.last_slam_time = rospy.Time.now()

    def _slam_healthy(self):
        if self.last_slam_time is None:
            return False
        age = (rospy.Time.now() - self.last_slam_time).to_sec()
        return age < self.slam_timeout

    def _in_geofence(self, x, y, z):
        return (self.geo.get('x_min', -999) <= x <= self.geo.get('x_max', 999) and
                self.geo.get('y_min', -999) <= y <= self.geo.get('y_max', 999) and
                self.geo.get('z_min', 0.0)  <= z <= self.geo.get('z_max', 999))

    def _dist(self, wp):
        if self.current_pose is None:
            return float('inf')
        p = self.current_pose.pose.position
        return math.sqrt((p.x - wp['x'])**2 +
                         (p.y - wp['y'])**2 +
                         (p.z - wp['z'])**2)

    def _publish_setpoint(self, wp):
        if not self._in_geofence(wp['x'], wp['y'], wp['z']):
            rospy.logwarn_throttle(2.0, 'Waypoint outside geofence, holding.')
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = wp['x']
        msg.pose.position.y = wp['y']
        msg.pose.position.z = wp['z']
        yaw = wp.get('yaw', 0.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(self.setpoint_rate)
        rospy.loginfo('Waypoint nav ready. %d waypoints loaded.', len(self.waypoints))

        while not rospy.is_shutdown():
            if self.wp_index >= len(self.waypoints):
                self.status_pub.publish('MISSION_COMPLETE')
                rospy.loginfo_once('Mission complete.')
                rate.sleep()
                continue

            if not self._slam_healthy():
                rospy.logwarn_throttle(2.0, 'SLAM unhealthy — holding setpoint.')
                rate.sleep()
                continue

            wp = self.waypoints[self.wp_index]
            self._publish_setpoint(wp)

            if self._dist(wp) < self.pos_tol:
                rospy.loginfo('Reached waypoint %d, holding %.1fs.',
                              self.wp_index, self.hold_time)
                deadline = rospy.Time.now() + rospy.Duration(self.hold_time)
                while rospy.Time.now() < deadline and not rospy.is_shutdown():
                    self._publish_setpoint(wp)
                    rate.sleep()
                self.wp_index += 1
                self.status_pub.publish(f'WP_{self.wp_index}_REACHED')

            rate.sleep()

if __name__ == '__main__':
    WaypointNav().run()
```

---

## Launch (ROS 1)

```xml
<!-- launch/planner_bridge.launch -->
<launch>
  <node pkg="slam_planner" type="waypoint_nav.py" name="waypoint_nav" output="screen">
    <rosparam file="$(find slam_planner)/config/planner_config.yaml" command="load"/>
    <param name="waypoints_file" value="$(find slam_planner)/config/waypoints.yaml"/>
  </node>
</launch>
```

## Launch (ROS 2)

```python
# launch/planner_bridge.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('slam_planner')
    return LaunchDescription([
        Node(
            package='slam_planner',
            executable='waypoint_nav.py',
            name='waypoint_nav',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'planner_config.yaml'),
                {'waypoints_file': os.path.join(pkg_dir, 'config', 'waypoints.yaml')},
            ],
        ),
    ])
```
