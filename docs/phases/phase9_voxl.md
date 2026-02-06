# Phase 9: VOXL/ModalAI Systems

## When to Use
User has VOXL hardware (ModalAI Starling, Seeker, Sentinel, VOXL2).
Detection: mentions "VOXL", "ModalAI", "qVIO", "adb shell", portal at port 5000.

## CRITICAL: ALL commands require `adb shell` prefix

```bash
adb shell "systemctl status voxl-qvio-server"
adb shell "voxl-inspect-services"
adb shell "px4-listener vehicle_visual_odometry"
```

## Architecture Differences
| Aspect | VOXL | Standard Linux |
|--------|------|----------------|
| Shell | `adb shell` | Direct SSH |
| Data flow | MPA pipes (/run/mpa/) | ROS topics |
| PX4 | Onboard (voxl-px4) | External Pixhawk |
| VIO | Pre-installed (qVIO/OpenVINS) | Build from source |
| Bridge | voxl-vision-hub | MAVROS |
| Monitor | px4-listener | rostopic echo |
| Visual | Portal (port 5000) | RViz |

## Config Files
| File | Purpose |
|------|---------|
| /etc/modalai/voxl-vision-hub.conf | VIO to PX4 forwarding |
| /etc/modalai/voxl-qvio-server.conf | qVIO parameters |
| /etc/modalai/voxl-open-vins-server.conf | OpenVINS parameters |
| /etc/modalai/vio_cams.conf | Camera selection |
| /etc/modalai/voxl-camera-server.conf | Camera enable/disable |
| /etc/modalai/extrinsics.conf | Camera-IMU transforms |

## 10-Step Pre-Flight Validation

1. **Services**: `adb shell "voxl-inspect-services"` - all running, CPU reasonable
2. **VIO health**: `adb shell "journalctl -u voxl-qvio-server --since '1 min ago'"` - no errors
3. **Vision-hub**: `adb shell "journalctl -u voxl-vision-hub --since '1 min ago'"` - no timestamp errors
4. **Initialize VIO**: Pick up drone, figure-8 motion 10-15s (props OFF!)
5. **Vision to PX4**: `adb shell "px4-listener vehicle_visual_odometry"` - NOT "never published"
6. **EKF fusion**: Move drone, check `adb shell "px4-listener vehicle_local_position"` updates
7. **PX4 params**: EKF2_EV_CTRL=15, EKF2_HGT_REF=3, battery failsafes set
8. **Drift test**: Hold still 60s, measure position drift
9. **Features**: Check portal (http://voxl-ip:5000), feature count > 20
10. **Estimator**: `adb shell "px4-listener estimator_status"` - healthy

## Common Issues

### "never published" on vehicle_visual_odometry
1. OpenVINS not initialized - Do figure-8 motion
2. Vision-hub not processing - `adb shell "systemctl restart voxl-vision-hub"`
3. Warmup period - Wait 3 seconds after initialization
4. Config error - Check voxl-vision-hub.conf

### "CAM_MISSING STALLED"
Most common causes:
1. **qVIO standby mode** (0% CPU): Set `"en_standby": false` in qVIO config
2. **SDK mismatch**: Check for missing `cal_file_2` field in camera config
3. **Wrong pipe name**: Verify pipe names match between services (_ion suffix)
4. **Camera overload**: Reduce enabled cameras
5. **Vision-hub connection**: Restart services in order

### Timestamp errors in vision-hub
- Check clock sync between services
- Verify camera server timestamps
- Restart all VIO services in order: camera-server, qvio-server, vision-hub

### Low feature count (<20)
- Improve lighting/texture in environment
- Lower feature threshold in VIO config
- Recalibrate camera

## NEVER do on VOXL
- Run commands without `adb shell`
- Use `rostopic echo` (use `px4-listener`)
- Use `roslaunch` (services via systemctl)
- Skip VIO initialization motion
- Assume ROS workspace exists
- Build packages (everything pre-installed)
- Blindly disable cameras without comparing to working backup
- Restore old backup configs without checking SDK compatibility (cal_file_2)
