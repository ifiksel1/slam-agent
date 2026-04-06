# VOXL OpenVINS Pre-Flight Validation Report
**Date**: 2026-01-13  
**Device ID**: 7032714  
**System**: VOXL with PX4 + OpenVINS

---

## ❌ FLIGHT STATUS: **NOT READY - CRITICAL ISSUES FOUND**

**DO NOT FLY** - Vision data is not reaching PX4!

---

## Validation Results

### ✅ PASSING Tests

| Test | Status | Details |
|------|--------|---------|
| **VOXL Services** | ✅ PASS | All critical services running |
| **OpenVINS Server** | ✅ PASS | Running (18.5% CPU), healthy |
| **PX4** | ✅ PASS | Running (27.8% CPU) |
| **MAVLINK Server** | ✅ PASS | Running (7.4% CPU) |
| **Camera Server** | ✅ PASS | Running (105.6% CPU - normal) |
| **OpenVINS Data Flow** | ✅ PASS | IMU & Camera data flowing (150 health checks every 5s) |
| **Vision-Hub Config** | ✅ PASS | VIO enabled, using "ov" pipe, 3s warmup |
| **PX4 EKF2 Config** | ✅ PASS | EKF2_EV_CTRL = 15 (vision enabled for pos/vel) |

### ❌ FAILING Tests (CRITICAL)

| Test | Status | Issue |
|------|--------|-------|
| **Vision Odometry to PX4** | ❌ **CRITICAL FAIL** | PX4 reports "never published" |
| **Vision-Hub Processing** | ❌ **CRITICAL FAIL** | Timestamp errors (~3s too new) |
| **Local Pose Output** | ❌ **CRITICAL FAIL** | vvhub_body_wrt_local pipe empty |
| **OpenVINS Pose Readability** | ⚠️ DEGRADED | voxl-inspect-pose shows data format errors |

---

## Root Cause Analysis

### Primary Issue: OpenVINS → Vision-Hub → PX4 Pipeline Broken

**Symptoms**:
1. OpenVINS is generating data (visible in portal)
2. Vision-hub shows timestamp errors: `ERROR in rc_tf_ringbuf_get_value, timestamp too new. Requested time 2.81s newer than latest data`
3. PX4's `vehicle_visual_odometry` topic shows: **"never published"**
4. vvhub_body_wrt_local pipe is not producing data

**Likely Causes**:
- OpenVINS may not be fully initialized (still in warmup period)
- Timestamp synchronization issue between OpenVINS and vision-hub
- Vision-hub unable to process OpenVINS output format
- TF (transform) data missing or delayed

---

## Next Steps to Fix

### Step 1: Check OpenVINS Initialization Status
```bash
adb shell
journalctl -u voxl-open-vins-server -f
# Look for "Initialization complete" or similar messages
```

### Step 2: Restart Vision-Hub
```bash
adb shell
systemctl restart voxl-vision-hub
# Wait 10 seconds, then check logs
journalctl -u voxl-vision-hub -f
```

### Step 3: Move the Drone
OpenVINS requires motion to initialize. With the drone powered on and PROPS OFF:
- Pick up the drone
- Move it slowly in a figure-8 pattern for 10-15 seconds
- Keep camera pointed at textured environment (not ceiling/floor)
- Check if vision data starts flowing after motion

### Step 4: Verify Vision Data Reaches PX4
```bash
adb shell
# Wait 5 seconds after vision-hub restart, then check:
timeout 5 px4-listener vehicle_visual_odometry

# Should show pose data updating at ~30 Hz
# If still "never published" → deeper configuration issue
```

### Step 5: Check PX4 Local Position
```bash
adb shell
timeout 3 px4-listener vehicle_local_position

# Should show position updating when drone moves
```

---

## Configuration Summary

### OpenVINS Configuration
- **Service Status**: Running, healthy
- **Data Rate**: IMU ~200 Hz, Camera ~30 Hz
- **Health Checks**: 150 checks every 5 seconds (good)

### Vision-Hub Configuration
- **VIO Enabled**: Yes (`"en_vio": true`)
- **VIO Pipe**: `"ov"` (OpenVINS)
- **Warmup Period**: 3 seconds
- **Offboard Mode**: VFC (VIO Flight Controller)
- **Timestamp Issue**: CRITICAL - 2-3 second offset

### PX4 Configuration
- **EKF2_EV_CTRL**: 15 (all vision modes enabled)
  - Bit 0 (1): Horizontal position
  - Bit 1 (2): Horizontal velocity
  - Bit 2 (4): Vertical position
  - Bit 3 (8): Vertical velocity
- **Vision Data Status**: NOT RECEIVING

---

## Safety Checklist (DO NOT SKIP)

Before attempting first flight (AFTER fixing vision data flow):

### Bench Test (Props OFF)
- [ ] Restart vision-hub: `systemctl restart voxl-vision-hub`
- [ ] Initialize OpenVINS (move drone in figure-8 for 10s)
- [ ] Confirm `px4-listener vehicle_visual_odometry` shows data
- [ ] Confirm `px4-listener vehicle_local_position` updates when moved
- [ ] Check position accuracy: Move forward 1m, verify Z changes ~1m
- [ ] Coordinate frame test: Forward=+X, Left=+Y, Up=+Z
- [ ] Drift test: Stationary for 60s, drift <20cm
- [ ] Feature count check (in portal): >50 features tracked

### Ground Test (Props ON, Tethered)
- [ ] All bench tests pass
- [ ] Install propellers
- [ ] Tether drone (< 0.5m movement allowed)
- [ ] Clear 5m safety radius
- [ ] Arm in MANUAL mode
- [ ] Spin motors to ~20% throttle
- [ ] Switch to POSITION mode
- [ ] Observe position hold (drift <10cm)
- [ ] Small stick inputs tracked correctly
- [ ] Disarm successfully

### Flight Test (Only After Ground Test Passes)
- [ ] All previous tests pass
- [ ] Geofence configured (check `px4-param show GF_*`)
- [ ] Battery fully charged
- [ ] RC kill switch tested
- [ ] Safety pilot ready
- [ ] First hover: 30 seconds max, 0.5m altitude
- [ ] Monitor drift continuously
- [ ] Land immediately if drift >50cm

---

## Key VOXL-Specific Findings (For Documentation Update)

### New Information Discovered:
1. **VOXL uses MPA pipes** instead of ROS topics
   - `ov` = OpenVINS output pipe
   - `ov_extended` = Extended OpenVINS data (shown in portal)
   - `vvhub_body_wrt_local` = Vision-hub processed output
   
2. **voxl-inspect-pose tool issues**:
   - Shows "ERROR validating pose_vel_6dof_t data" for ov/ov_extended pipes
   - Packet size mismatches (324 bytes vs expected multiple of 84)
   - Tool may be out of sync with data format
   
3. **Vision-hub warmup period**:
   - Default 3 seconds (`vio_warmup_s`: 3)
   - Data won't be sent to PX4 until stable for 3 seconds
   - May need to be increased if OpenVINS struggles to initialize
   
4. **Timestamp synchronization critical**:
   - Vision-hub errors show ~2-3 second timestamp offset
   - This prevents data fusion
   - May indicate OpenVINS not initialized or TF data missing
   
5. **PX4 listener commands for VOXL**:
   - `px4-listener vehicle_visual_odometry` - Check if vision data arrives
   - `px4-listener vehicle_local_position` - Check if EKF fuses vision
   - `px4-listener estimator_status` - Check EKF health
   - `px4-param show <param>` - Check PX4 parameters
   
6. **Critical VOXL config file**:
   - `/etc/modalai/voxl-vision-hub.conf` - Controls vision→PX4 pipeline
   - JSON format, many tunable parameters
   - Key settings: `en_vio`, `vio_pipe`, `vio_warmup_s`

---

## Conclusion

**VERDICT**: ❌ **NOT READY FOR FLIGHT**

**Critical Issue**: Vision odometry is not reaching PX4. The drone would not be able to hold position and would likely crash if flown in POSITION mode without GPS.

**What's Working**:
- ✅ OpenVINS is running and processing camera/IMU data
- ✅ Portal visualization shows movement
- ✅ PX4 is configured to accept vision data

**What's Broken**:
- ❌ Vision-hub is not successfully forwarding data to PX4
- ❌ Timestamp synchronization errors
- ❌ PX4 never receives vision odometry messages

**Required Actions**:
1. Restart vision-hub service
2. Ensure OpenVINS fully initializes (requires motion)
3. Verify vision data reaches PX4 (px4-listener)
4. Complete full bench test validation
5. Only then proceed to tethered ground test

**DO NOT FLY** until `px4-listener vehicle_visual_odometry` shows data updating at ~30 Hz.

---

**Generated**: 2026-01-13 via automated validation  
**Next Validation**: After applying fixes above
