# COIN-LIO Workspace Details

## Location
`/home/dev/coinlio_ws/`

## Packages
- **coin-lio**: LiDAR-Inertial Odometry with intensity/photometric fusion (ETH Zurich, ICRA 2024)
- **LiDAR_IMU_Init**: Online LiDAR-IMU extrinsic calibration (HKU MARS Lab)
- **ouster-ros**: Ouster ROS 1 driver (v0.13.9, latest master)
- **geographiclib**: Geographic coordinate conversions

## GitHub Remotes (user: ifiksel1)
All repos have `origin` → ifiksel1 fork, `upstream` → original repo:
- coin-lio: `origin=ifiksel1/coin-lio`, `upstream=ethz-asl/coin-lio`
- LiDAR_IMU_Init: `origin=ifiksel1/LiDAR_IMU_Init`, `upstream=hku-mars/LiDAR_IMU_Init`
- ouster-ros: `origin=ifiksel1/ouster-ros`, `upstream=ouster-lidar/ouster-ros`

## Key Upgrades Applied (2026-02-22)
1. **ARM64 multi-threading**: Removed x86-only guard, added `armv8.2-a+fp16` + `-mtune=native`
2. **C++17**: Updated from C++14
3. **OpenMP fix**: Guarded against `NOTFOUND` being injected as compiler flag
4. **OpenCV ABI fix**: LD_PRELOAD + link order (see `opencv_abi_conflict.md`)
5. **Ouster driver**: Updated to v0.13.9 from v0.13.0+14
6. **Vision bridge**: New `coin_lio_vision_bridge` node for MAVROS integration
7. **Loop closure**: Header-only `loop_closure.h` extracted from FAST_LIO_SAM (disabled by default, user chose not to enable)
8. **LiDAR_IMU_Init Ceres 2.2 fix**: Migrated from `LocalParameterization` to `Manifold` API
9. **FAST_LIO_SAM removed**: Not relevant to COIN-LIO

## Ouster Driver Note
- The ouster-ros in coinlio_ws crashes on ARM64 (exit 255)
- **Workaround**: Use pre-built driver from `ultraviewdev_ws` instead
- ultraviewdev_ws driver works on ARM64 and publishes to same topics

## Full Pipeline (tested working 2026-02-22)
```
ultraviewdev_ws Ouster driver → COIN-LIO (20Hz) → vision_bridge → MAVROS → CubeOrange FC
```
- Ouster at 169.254.56.220 (link-local), 1024x20 mode
- MAVROS on /dev/ttyACM0 (CubeOrange)
- Local position confirmed flowing to FC
