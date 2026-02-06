# Troubleshooting: Coordinate Frame Issues

## Symptoms
- Drone moves wrong direction (backward, sideways)
- Drone flips or spins uncontrollably
- Position drifts in one axis

## Frame Systems
- **ROS**: ENU (East-North-Up) - X=forward, Y=left, Z=up
- **ArduPilot/PX4**: NED (North-East-Down) - X=forward, Y=right, Z=down
- **vision_to_mavros**: Handles ENU to NED conversion

## Common Mistakes
| Symptom | Cause | Fix |
|---------|-------|-----|
| Moves backward | LiDAR X-axis reversed | Flip LiDAR rotation in URDF (add 180 yaw) |
| Moves sideways | X/Y axes swapped | Check sensor frame orientation |
| Altitude wrong | Z inverted | Check base_link Z convention |
| Spins | Yaw reference wrong | Check `gamma_world` in vision_to_mavros |
| Slow drift left/right | Slight rotation offset | Fine-tune URDF rpy values |

## Verification
```bash
# Check TF tree
rosrun tf view_frames && evince frames.pdf
# Verify transform
rosrun tf tf_echo base_link lidar_link
# Check vision pose frame
rostopic echo /mavros/vision_pose/pose -n 1
```

## Fix: gamma_world parameter
In vision_to_mavros:
- `gamma_world: 0` - ENU (default for ROS SLAM)
- `gamma_world: 1.5708` - Rotate 90 degrees
- If using custom bridge: ensure ENU to NED conversion in code
