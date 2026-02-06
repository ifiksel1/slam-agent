# Troubleshooting: Sensor Calibration

## Camera Calibration
Tools: `camera_calibration` ROS package or Kalibr
Target: Checkerboard or AprilGrid
Procedure:
1. Print calibration target at known size
2. Record images from multiple angles (50+ images)
3. Run calibrator, verify reprojection error < 0.5 pixels
4. Save to camera_info YAML

## Camera-IMU Calibration (Kalibr) - CRITICAL for VIO
Without this, VIO will have scale drift and rotation errors.

Steps:
1. Install Kalibr (build from source or Docker)
2. Create AprilGrid target (6x6, 0.088m tag size, 0.3 ratio)
3. Record bag: slow motion, all axes, 30-60 seconds
4. Run: `rosrun kalibr kalibr_calibrate_imu_camera --target target.yaml --bag calib.bag --cam cam.yaml --imu imu.yaml`
5. Results: T_cam_imu transform, time offset

Common mistakes:
| Mistake | Effect | Fix |
|---------|--------|-----|
| Too fast motion | Blurry frames | Move slowly, 3-5 seconds per motion |
| Single axis only | Poor observability | Rotate all 3 axes |
| Bad lighting | Few features detected | Even, bright lighting |
| Wrong IMU noise values | Bad calibration | Use Allan variance results |
| Wrong camera model | Distortion errors | Use correct model (pinhole vs fisheye) |

## LiDAR-IMU Calibration
Methods:
1. **LI-Calib** (offline): Best accuracy, requires recorded bag
2. **FAST-LIO online**: Set `extrinsic_est_en: true`, let it converge
3. **Manual measurement**: Measure physical offsets, apply frame transforms

Common LiDAR frame conventions:
| LiDAR | X | Y | Z | Notes |
|-------|---|---|---|-------|
| Ouster | Forward | Left | Up | Standard ROS |
| Velodyne | Forward | Left | Up | Standard ROS |
| Livox | Forward | Left | Up | Standard ROS |
| Hesai | Forward | Left | Up | Check specific model |

Verification: Move LiDAR slowly, check point cloud alignment doesn't shift.

## IMU Calibration (Allan Variance)
Required for accurate SLAM noise parameters.
```bash
# Record 2+ hours of static IMU data
rosbag record /imu/data -O imu_static.bag --duration 7200
# Run Allan variance
rosrun allan_variance_ros allan_variance imu_static.bag /imu/data
rosrun allan_variance_ros analysis.py --data allan_variance.csv
```
Extract: accelerometer_noise_density, gyroscope_noise_density, accelerometer_random_walk, gyroscope_random_walk

Use these values in SLAM config for imuAccNoise, imuGyrNoise, imuAccBiasN, imuGyrBiasN.
