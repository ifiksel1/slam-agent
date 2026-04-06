# Phase 8D: Nav2 (ROS 2, Corridor / Tunnel Environments)

Nav2 works for drones in linear environments by managing altitude separately. **ROS 2 only.**

---

## Install

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-nav2-costmap-2d
```

Via MCP:
```python
run_install_script("install_path_planner", "nav2 ~/ros2_ws ROS2")
```

---

## Key Adaptations for Drones

1. Use `voxel_layer` in the costmap, fed from the SLAM point cloud projected to 2.5D.
2. Replace the default DWB controller with a simple position controller (or carrot-follower).
3. Manage altitude via a separate MAVROS altitude setpoint node — Nav2 only handles X/Y.
4. Disable the footprint rotation cost function (drones can translate without rotating).

```yaml
# config/nav2_params.yaml (excerpt)
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        observation_sources: pointcloud
        pointcloud:
          topic: /slam/cloud_registered
          data_type: PointCloud2
          min_obstacle_height: 0.2
          max_obstacle_height: 3.0
          raytrace_max_range: 8.0
          obstacle_max_range: 6.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8
        cost_scaling_factor: 3.0
```

---

## Bridge to MAVROS

Nav2 outputs `/cmd_vel` (velocity commands) and `/plan` (path). Bridge these to MAVROS position setpoints using the generic planner bridge from Phase 8C, or write a velocity-to-position adapter.

For altitude control, run a separate node that publishes Z setpoints to `/mavros/setpoint_position/local` while Nav2 handles X/Y via `/cmd_vel`.
