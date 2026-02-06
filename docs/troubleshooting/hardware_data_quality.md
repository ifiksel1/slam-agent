# Hardware Mounting & Data Quality

Load when user reports noisy data, vibration issues, or SLAM failures in specific environments.

---

## Vibration Issues

**Symptoms**: Noisy point clouds (blurry when stationary), IMU oscillations, drift correlated with motor speed.

**Solutions**:
- Mount sensors on anti-vibration dampers (soft foam, gel pads)
- Increase distance from motors/props
- Balance propellers (critical!)
- Check for loose screws/connections

**Check vibration in data**:
```bash
rostopic echo /mavros/imu/data | grep -A3 "linear_acceleration"
# Should be ~9.8 m/s² in Z when stationary
# Oscillations >0.5 m/s² = too much vibration
```

## Sensor Alignment

- No flex in mounting plates (rigid mount required)
- LiDAR rotation axis must be level (use spirit level)
- Camera should not rotate during flight
- All mounting screws tight (use threadlocker)

---

## Insufficient Features

**Problem**: SLAM drifts in featureless environments.

**Symptoms**: Works in cluttered room, fails in empty hallway. Drift increases in open areas.

**Solutions**:
- Add temporary visual features (cardboard boxes, posters)
- Enable loop closure to reduce drift
- Consider hybrid LiDAR + camera SLAM
- Reduce max flight speed in featureless areas

---

## Dynamic Objects

**Problem**: Moving objects cause false odometry.

**Symptoms**: Position jumps when people walk by. Drift when following another vehicle.

**Solutions**:
- Use SLAM with dynamic object filtering (some algorithms support this)
- Avoid flying in crowded areas
- Increase sensor height to see over obstacles
- Use semantic SLAM (advanced)

---

## Motion Distortion

**Problem**: Point clouds distorted during fast motion. LiDAR scans take ~100ms; drone moves during scan.

**Solutions**:
- Most SLAM algorithms handle this (deskewing/motion compensation) - verify it's enabled in config
- Reduce max velocity during mapping
- Check that IMU data is being used for deskewing
