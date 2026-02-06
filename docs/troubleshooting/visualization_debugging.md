# Visualization & Debugging Tools

Load when user says "I can't see what's happening" or needs to debug connectivity/state.

---

## RViz Setup for SLAM

```bash
# ROS1:
rviz
# ROS2:
rviz2
```

### Essential Displays

1. **TF** - Shows coordinate frames (check for missing/jittery frames)
2. **RobotModel** - Shows URDF (requires `robot_description` parameter)
3. **PointCloud2** - LiDAR data (topic: `/ouster/points` or similar). Points should be stationary when robot is stationary
4. **Odometry** - SLAM path (topic: `/slam/odometry`, Keep Length: 1000)
5. **Map** - 2D Map or PointCloud2 for 3D map (topic: `/slam/cloud_registered`)

Save config: `File > Save Config As... > my_slam.rviz`

Launch file integration:
```xml
<node pkg="rviz" type="rviz" name="rviz"
      args="-d $(find my_pkg)/rviz/my_slam.rviz"/>
```

---

## Topic Debugging

```bash
# ROS1:
rostopic list
rostopic hz /ouster/points
rostopic hz /slam/odometry
rostopic hz /mavros/vision_pose/pose
rostopic echo /slam/odometry -n 1
rostopic info /slam/odometry    # Shows Type, Publishers, Subscribers

# ROS2:
ros2 topic list
ros2 topic hz /ouster/points
ros2 topic echo /slam/odometry --once
ros2 topic info /slam/odometry
```

---

## Node Debugging

```bash
rosnode list
rosnode info /slam_node     # Publications, Subscriptions, Services
rosnode ping /slam_node     # Check if responsive
rosnode kill /slam_node     # Kill stuck node
```

---

## Computation Graph

```bash
rqt_graph    # Visualize all node connections
```

**What to verify in the graph**:
- SLAM node subscribing to `/ouster/points` and `/mavros/imu/data`
- SLAM node publishing to `/slam/odometry`
- `vision_to_mavros` subscribing to `/slam/odometry`
- `vision_to_mavros` publishing to `/mavros/vision_pose/pose`

---

## TF Debugging

```bash
# Generate PDF of TF tree:
rosrun tf view_frames && evince frames.pdf

# Real-time viewer:
rosrun rqt_tf_tree rqt_tf_tree

# Echo specific transform:
rosrun tf tf_echo map base_link

# Check TF rates:
rostopic hz /tf
rostopic hz /tf_static
```

### Common TF Errors

| Error | Meaning | Fix |
|-------|---------|-----|
| `Lookup would require extrapolation into the past` | Timestamps out of sync | Check time sync, use `use_sim_time` if needed |
| `Transform from X to Y does not exist` | Missing link in TF tree | Check URDF or static publishers |
| `TF_REPEATED_DATA` | Duplicate publishers | Remove duplicate TF broadcasters |

---

## Bag Files (Offline Debugging)

```bash
# Record specific topics:
rosbag record /ouster/points /mavros/imu/data /slam/odometry -O test1.bag

# Play back:
rosbag play test1.bag

# Play at half speed:
rosbag play test1.bag -r 0.5

# Check contents:
rosbag info test1.bag
```

### Offline SLAM Testing
```bash
# Terminal 1:
rosbag play test1.bag --clock

# Terminal 2:
rosparam set use_sim_time true
roslaunch my_slam slam.launch

# Terminal 3:
rviz
```
