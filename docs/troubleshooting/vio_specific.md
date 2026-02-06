# Troubleshooting: VIO-Specific Issues

## Environment Requirements
- **Lighting**: 200+ lux, consistent, no flicker (use LED, not fluorescent)
- **Texture**: Walls/floor with visual features (posters, tape markings if bare)
- **Speed**: < 2 m/s for monocular, < 5 m/s for stereo
- **Features**: > 50 tracked features in camera view

## Initialization Failures
| Cause | Diagnosis | Fix |
|-------|-----------|-----|
| Insufficient motion | "Not enough excitation" | Move in all 6 DOF for 5-10s |
| Bad camera-IMU extrinsics | Large initial error | Re-run Kalibr calibration |
| Wrong IMU noise params | "IMU readings inconsistent" | Run Allan variance |
| Time sync offset | "Timestamp mismatch" | Check `time_offset` in config |
| Few visual features | "Not enough features" | Add texture, adjust lighting |

## Tracking Lost
1. **Texture-poor area**: Add visual markers, use fisheye lens
2. **Motion blur**: Reduce speed, lower exposure time
3. **Camera calibration wrong**: Re-calibrate, check reprojection error
4. **Feature parameters**: Increase `num_pts` (200-300 indoor, 100-200 outdoor)

## Scale Drift
Primary cause: Poor camera-IMU extrinsics - re-run Kalibr
Secondary: Wrong IMU noise parameters - run Allan variance
Tertiary: Time sync offset > 5ms - adjust `time_offset`

## Rotation Drift
Primary cause: IMU gyroscope bias instability
Fix: Better IMU noise parameters, check vibration isolation

## Environment-Specific
- **Low light**: Lower feature threshold, increase exposure, add lighting
- **Textureless**: Add visual markers, consider switching to LiDAR SLAM
- **Outdoor**: Enable auto-exposure, use polarizing filter for glare
- **Fast motion**: Increase camera FPS, enable multi-threading in VIO
