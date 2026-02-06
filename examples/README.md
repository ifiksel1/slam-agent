# Example Configurations

This directory contains working example configurations for various SLAM algorithms and hardware combinations.

## Directory Structure

```
examples/
├── lio_sam_ouster/          # LIO-SAM with Ouster OS1-64 (Ultra-onboard reference)
├── fast_lio_velodyne/       # FAST-LIO with Velodyne VLP-16
├── fast_lio_livox/          # FAST-LIO with Livox Mid-360
├── cartographer_2d/         # Cartographer 2D SLAM
├── orb_slam3_stereo/        # ORB-SLAM3 with stereo camera
├── rtabmap_rgbd/            # RTAB-Map with RGB-D camera
└── templates/               # Generic templates for new algorithms
```

## Usage

1. Choose the example closest to your hardware
2. Copy the configuration files to your workspace
3. Modify sensor topics, frame IDs, and parameters for your specific setup
4. Test following the included testing checklist

## Contributing Examples

When adding a new example configuration:

1. Create a directory: `algorithm_sensor/`
2. Include these files:
   - `config/params.yaml` - SLAM parameters
   - `config/autopilot.parm` - ArduPilot/PX4 parameters
   - `launch/main.launch` - Master launch file
   - `urdf/robot.urdf` (if using URDF) or `launch/static_tfs.launch`
   - `README.md` - Hardware specs, installation, testing notes
3. Test thoroughly before committing

## Quick Reference

| Example | SLAM | Sensor | Platform | Loop Closure | Notes |
|---------|------|--------|----------|--------------|-------|
| lio_sam_ouster | LIO-SAM | Ouster OS1-64 | Jetson Orin NX | Yes | Ultra-onboard baseline |
| fast_lio_velodyne | FAST-LIO2 | Velodyne VLP-16 | Intel NUC | No | Low-cost option |
| fast_lio_livox | FAST-LIO | Livox Mid-360 | Jetson Xavier NX | No | Solid-state LiDAR |
| cartographer_2d | Cartographer | 2D LiDAR | Raspberry Pi 4 | Yes | Lightweight 2D |
| orb_slam3_stereo | ORB-SLAM3 | ZED 2 | x86 Laptop | Yes | Visual-only |
| rtabmap_rgbd | RTAB-Map | RealSense D435i | Jetson Nano | Yes | RGB-D SLAM |

---

For detailed integration instructions, see `../docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md`

