# Foxglove Visualization Setup

## What It Is

Foxglove Studio is a free, browser-based robotics visualization tool (like RViz but runs on any device with a browser -- laptop, tablet, phone). It connects to your SLAM system over WebSocket and lets you visualize:
- Live 3D point clouds from your LiDAR
- SLAM odometry/trajectory in real-time
- TF tree, IMU data, diagnostics
- All without installing anything on the viewing device

It's especially useful for field testing -- you can monitor SLAM from your laptop while the drone is running, without needing X11 forwarding or a display attached to the companion computer.

## Setup by ROS Version

### ROS 2 (Humble/Jazzy)

Build the C++ `foxglove_bridge` from `foxglove/foxglove-sdk` repo (not the deprecated `ros-foxglove-bridge`):

- Dependencies: `rosx_introspection` (build from source), `rapidjson-dev`, `nlohmann-json3-dev`, `libasio-dev`
- Fix required: Add `find_package(sensor_msgs REQUIRED)` to foxglove_bridge CMakeLists.txt
- Service introspection errors are non-fatal -- topic bridging works fine
- Add to launch file with `enable_foxglove:=true` argument (default: true)
- Note: The Python `foxglove-websocket` 0.1.4 package is protocol-incompatible with modern Foxglove Studio -- always use the C++ bridge

### ROS 1 (Noetic)

Use `apt install ros-noetic-rosbridge-server` and Foxglove Studio's rosbridge connection mode.

### Docker

Expose port 8765 (already done if using host networking). Add to launch file so it starts automatically with SLAM.

## Control Script

Generate `scripts/foxglove.sh` with start/stop/restart/status/logs commands following the node control script pattern.

## User Instructions

Open Foxglove Studio at https://app.foxglove.dev, click "Open connection", enter `ws://<jetson-ip>:8765`.
