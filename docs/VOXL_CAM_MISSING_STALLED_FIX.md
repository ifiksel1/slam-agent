# VOXL "CAM_MISSING STALLED" Error - Complete Fix Guide

**Issue**: qVIO portal shows `ERR: CAM_MISSING STALLED` with QUAL: -1%, PTS: 0, CPU: 0%

**Date**: January 2026  
**System**: VOXL2 with qVIO, PX4 external FC  
**Resolution Time**: ~2 hours of systematic debugging

---

## 🎯 Quick Diagnosis

```bash
# Run these commands via adb shell to identify the issue:

# 1. Check qVIO CPU usage
adb shell "voxl-inspect-services | grep qvio"

# If CPU = 0% → Standby mode issue (see Fix #1)
# If CPU > 0% but portal shows error → Pipe/config issue (see Fix #2-4)
```

---

## 🔧 Fix #1: qVIO Standby Mode (Most Common Root Cause!)

**Symptoms**:
- qVIO service running but 0% CPU
- Portal shows "CAM_MISSING STALLED"
- No errors in qVIO logs
- Camera streams working fine in portal

**Root Cause**:
qVIO's `en_standby_mode: true` causes it to wait for CPU monitor state. If CPU monitor state file is missing or broken, qVIO never processes frames.

**Solution**:
```bash
# Check standby mode setting
adb shell "cat /etc/modalai/voxl-qvio-server.conf | grep en_standby_mode"

# If true, disable it:
adb shell "sed -i 's/\"en_standby_mode\":\s*true/\"en_standby_mode\": false/' /etc/modalai/voxl-qvio-server.conf"

# Restart qVIO
adb shell "systemctl restart voxl-qvio-server && sleep 2 && voxl-inspect-services | grep qvio"

# Verify CPU is now >0%
```

**Expected Result**: qVIO CPU should jump to 1-5% and start tracking.

---

## 🔧 Fix #2: SDK Version Mismatch (cal_file_2 Missing)

**Symptoms**:
- qVIO crashes on start (exit code 255)
- Error log: `ERROR: object missing cal_file_2`
- Happens after restoring old SDK backup configs

**Root Cause**:
Newer VOXL SDK versions require `"cal_file_2": ""` field in `/etc/modalai/vio_cams.conf`. Old backups don't have this field.

**Solution**:
```bash
# Check if field exists
adb shell "cat /etc/modalai/vio_cams.conf"

# If missing "cal_file_2", add it:
adb shell "cat > /etc/modalai/vio_cams.conf << 'EOF'
{
  \"cams\": [
    {
      \"enable\": true,
      \"name\": \"tracking_front\",
      \"pipe_for_preview\": \"tracking_front\",
      \"pipe_for_tracking\": \"tracking_front_misp_norm\",
      \"is_occluded_on_ground\": false,
      \"imu\": \"imu_apps\",
      \"cal_file\": \"opencv_tracking_front_intrinsics.yml\",
      \"cal_file_2\": \"\"
    }
  ]
}
EOF
"

# Restart qVIO
adb shell "systemctl restart voxl-qvio-server"
```

---

## 🔧 Fix #3: Wrong Pipe Name in vio_cams.conf

**Symptoms**:
- qVIO running (CPU >0%) but not getting camera data
- Camera pipes exist but qVIO not receiving

**Root Cause**:
Mismatch between pipe name in `vio_cams.conf` and actual camera server output pipe.

**Diagnosis**:
```bash
# Check available camera pipes
adb shell "ls -la /run/mpa/ | grep tracking_front_misp"

# Common pipes:
# - tracking_front_misp_norm         (standard)
# - tracking_front_misp_norm_ion     (ION memory)

# Check what qVIO is configured to use
adb shell "cat /etc/modalai/vio_cams.conf | grep pipe_for_tracking"
```

**Solution**:
Match the pipe name in `vio_cams.conf` to what actually exists:

```bash
# If your system has "tracking_front_misp_norm" (without _ion):
adb shell "cat > /etc/modalai/vio_cams.conf << 'EOF'
{
  \"cams\": [{
    \"enable\": true,
    \"name\": \"tracking_front\",
    \"pipe_for_preview\": \"tracking_front\",
    \"pipe_for_tracking\": \"tracking_front_misp_norm\",
    \"is_occluded_on_ground\": false,
    \"imu\": \"imu_apps\",
    \"cal_file\": \"opencv_tracking_front_intrinsics.yml\",
    \"cal_file_2\": \"\"
  }]
}
EOF
"

adb shell "systemctl restart voxl-qvio-server"
```

---

## 🔧 Fix #4: Compare with Working Backup

**When to Use**: Nothing else works, but you have a previous working backup

**Critical Lesson**: Don't assume optimization (like disabling cameras) will help. The working configuration might have ALL cameras enabled for a reason.

**Process**:
```bash
# 1. Check if you have a backup directory
ls ~/Downloads/voxl_backup*/etc_modalai/

# 2. Compare key config files:
# - voxl-qvio-server.conf (standby mode, parameters)
# - vio_cams.conf (camera selection, pipe names)
# - voxl-camera-server.conf (camera enables, FPS, exposure)
# - voxl-vision-hub.conf (VIO pipe selection, warmup)

# 3. Identify differences:
diff <current_file> <backup_file>

# 4. Restore backup configs:
adb push voxl_backup/etc_modalai/vio_cams.conf /etc/modalai/vio_cams.conf
adb push voxl_backup/etc_modalai/voxl-qvio-server.conf /etc/modalai/voxl-qvio-server.conf

# 5. Add missing SDK fields (if needed)
# Edit restored config to add "cal_file_2": ""

# 6. Restart services
adb shell "systemctl restart voxl-camera-server && sleep 3 && systemctl restart voxl-qvio-server && sleep 2 && systemctl restart voxl-vision-hub"
```

---

## 📊 Success Verification

After applying fixes, portal should show:

**qvio_overlay stream**:
- ✅ **QUAL**: +100% (good quality)
- ✅ **XYZ**: Non-zero values updating in real-time
- ✅ **PTS**: 20-50 (feature points - white squares visible on image)
- ✅ **Exposure(ms)**: 5-10ms (camera properly configured)
- ✅ **Gain**: 300-500 (auto-exposure working)
- ✅ **Quality**: 60-100 (high)
- ✅ **ERR**: 0x0 (no errors)
- ✅ **qVIO CPU**: 1-5% (actively processing)

**Services**:
```bash
adb shell "voxl-inspect-services"

# Expected:
# voxl-camera-server:  30-95% CPU (depends on # cameras)
# voxl-qvio-server:    1-5% CPU
# voxl-vision-hub:     1-7% CPU
```

**Logs**:
```bash
adb shell "journalctl -u voxl-vision-hub --since '1 minute ago' | grep -i error"
# Should show: No errors (or only minor warnings)
# Should NOT show: "timestamp too new" repeatedly
```

---

## 🚨 What NOT to Do (Common Mistakes)

1. **Don't assume motion will fix everything**
   - Motion initializes qVIO, but doesn't fix configuration issues
   - If CPU = 0%, motion won't help - fix standby mode first

2. **Don't disable cameras without checking backup**
   - The working configuration might need all cameras enabled
   - Disabling cameras can break internal dependencies

3. **Don't restore old backups blindly**
   - Check SDK compatibility first
   - Add missing fields like `cal_file_2`
   - Verify pipe names match current system

4. **Don't ignore CPU usage**
   - qVIO at 0% CPU = not processing frames (config issue)
   - qVIO at 1-5% CPU = normal operation
   - qVIO at >10% CPU = under load but working

5. **Don't skip comparing configs**
   - Always diff current vs working backup
   - Small differences (1-2 fields) can break everything

---

## 🔑 Key Lessons for AI Assistant

1. **Check working backup FIRST** before trying optimization
2. **qVIO 0% CPU = standby mode or crash**, not initialization
3. **Portal errors can have multiple root causes** - systematic diagnosis required
4. **SDK version matters** - old backups may need field updates
5. **"timestamp too new" means qVIO not producing current data**, not a time sync issue
6. **Feature tracking visible in portal = system working**, regardless of voxl-inspect-pose errors
7. **Don't assume motion fixes config issues** - only fixes initialization

---

## 📋 Systematic Troubleshooting Checklist

When user reports "CAM_MISSING STALLED":

```
[ ] 1. Check qVIO CPU (voxl-inspect-services)
      └─ If 0% → Check standby mode
      └─ If >0% → Continue to next step

[ ] 2. Check qVIO logs for crashes (journalctl -u voxl-qvio-server)
      └─ If "cal_file_2" error → SDK mismatch
      └─ If "disconnected" → Pipe issue

[ ] 3. Check vision-hub errors (journalctl -u voxl-vision-hub)
      └─ If "timestamp too new" → qVIO not producing current data
      └─ If no errors → qVIO not initialized

[ ] 4. Compare with working backup (if available)
      └─ Diff all config files
      └─ Identify what changed
      └─ Restore backup + add SDK compatibility fixes

[ ] 5. Verify portal shows proper exposure/gain
      └─ If 0.000ms/0 gain → Camera not configured
      └─ If >0 → Camera working, qVIO issue

[ ] 6. Restart services in order:
      └─ camera → qvio → vision-hub

[ ] 7. Check portal after 30 seconds
      └─ Success: PTS >20, QUAL >0%, ERR: 0x0
```

---

## 🎉 Success Criteria

System is ready for flight when:
- ✅ qVIO CPU: 1-5% (not 0%)
- ✅ Portal QUAL: >50%
- ✅ Portal PTS: >20 features
- ✅ Portal ERR: 0x0
- ✅ No "timestamp too new" errors for >30 seconds
- ✅ Feature tracking visible (white squares on image)
- ✅ Position (XYZ) updating when drone moves

---

**Reference**: This guide is based on real troubleshooting session from January 2026 with Starling 2 + external PX4.
