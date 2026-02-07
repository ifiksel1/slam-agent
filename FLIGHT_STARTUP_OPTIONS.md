# Flight Startup Options
**Pre-Flight Check Integration Guide**

---

## Overview

You have three ways to ensure the preflight check runs before each flight:

| Option | Method | Best For | Automation |
|--------|--------|----------|-----------|
| **Option 1** | Bash script wrapper | Manual flights | Manual |
| **Option 2** | Python script | Manual + logging | Manual |
| **Option 3** | Systemd service | Autonomous startup | Automatic at boot |

---

## Option 1: Bash Script Wrapper (Simplest)

### Usage
```bash
chmod +x /home/dev/slam-agent/start_slam_with_preflight.sh
/home/dev/slam-agent/start_slam_with_preflight.sh
```

### What It Does
1. **Launches SLAM** in background
2. **Waits 20 seconds** for SLAM to initialize and start generating data
3. **Runs preflight checks** (now with SLAM actually running and generating odometry)
4. If checks fail → SLAM stays running for debugging
5. If checks pass → SLAM continues running (ready for flight)

### When to Use
- **Quick manual flights**
- **Testing and development**
- **Simple startup procedure**

### Output Example
```
╔════════════════════════════════════════════════════════════════════╗
║         SLAM FLIGHT STARTUP WITH PRE-FLIGHT VERIFICATION         ║
╚════════════════════════════════════════════════════════════════════╝

[STEP 1] Running Pre-Flight System Check...
  ✓ Ouster sensor reachable (169.254.56.220)
  ✓ eth0 physical link UP
  ... (more checks)
  ✓ ALL CRITICAL CHECKS PASSED - SYSTEM READY FOR OPERATION

╔════════════════════════════════════════════════════════════════════╗
║  ✅ PRE-FLIGHT CHECK PASSED - LAUNCHING SLAM                      ║
╚════════════════════════════════════════════════════════════════════╝

Press ENTER to start SLAM, or Ctrl+C to cancel...
```

---

## Option 2: Python Script (Recommended for Manual Flights)

### Usage
```bash
chmod +x /home/dev/slam-agent/flight_startup.py
/home/dev/slam-agent/flight_startup.py
```

### What It Does
1. **Launches SLAM** in background
2. **Waits for initialization** (~20 seconds)
3. **Runs preflight checks** (verifies SLAM generating odometry)
4. If checks fail → SLAM stays running for debugging
5. If checks pass → SLAM continues running in foreground
6. Logs all operations
7. Shutdown with Ctrl+C

### When to Use
- **Regular manual flights with confirmation**
- **Want more control over launch**
- **Need logging for troubleshooting**
- **Multi-mission operations**

### Output Example
```
╔══════════════════════════════════════════════════════════════════╗
║        SLAM AUTONOMOUS FLIGHT STARTUP                           ║
╚══════════════════════════════════════════════════════════════════╝

STEP 1: RUNNING PRE-FLIGHT CHECKS
═══════════════════════════════════════════════════════════════════

✓ Ouster sensor reachable
✓ eth0 physical link UP
... (checks)
✓ ALL CRITICAL CHECKS PASSED

STEP 2: FLIGHT STARTUP CONFIRMATION
═══════════════════════════════════════════════════════════════════

System Status: ✅ READY FOR FLIGHT

About to launch SLAM for autonomous flight...
  • LiDAR: Connected and streaming
  • SLAM: Ready to process data
  • Odometry: Ready to output

Ready to launch SLAM? (yes/no): yes

STEP 3: LAUNCHING SLAM
═══════════════════════════════════════════════════════════════════

Starting SLAM...
Flight session started. Ctrl+C to shutdown...
```

---

## Option 3: Systemd Service (Autonomous/Automatic)

### For Completely Autonomous Startup

Create a systemd service that runs at boot:

```bash
sudo nano /etc/systemd/system/slam-flight.service
```

Paste this:
```ini
[Unit]
Description=SLAM Flight System with Pre-Flight Checks
After=network.target ros.service
Wants=ros.service

[Service]
Type=simple
User=dev
WorkingDirectory=/home/dev/slam_ws
ExecStartPre=/home/dev/slam-agent/preflight_check.sh
ExecStart=/bin/bash -c 'source devel/setup.bash && roslaunch fast_lio mapping_ouster64.launch rviz:=false'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Enable and Start
```bash
sudo systemctl daemon-reload
sudo systemctl enable slam-flight
sudo systemctl start slam-flight
```

### Monitor Status
```bash
# Check if running
sudo systemctl status slam-flight

# View logs
sudo journalctl -u slam-flight -f

# Stop
sudo systemctl stop slam-flight
```

### When to Use
- **Autonomous systems (drones, robots)**
- **System boots and SLAM starts automatically**
- **Field deployments without operator interaction**
- **Requires pre-flight to pass or system aborts**

---

## Recommended Setup for Your Use Case

### For Development/Testing ⭐ SIMPLEST
```bash
# Launches SLAM, waits for initialization, verifies, then keeps running
/home/dev/slam-agent/start_slam_with_preflight.sh
```

### For Regular Operations ⭐ RECOMMENDED
```bash
# Same as above but with better logging and control
/home/dev/slam-agent/flight_startup.py
```

### For Autonomous Deployment (Drone/Robot)
```bash
# Fully automatic at boot (requires systemd)
sudo systemctl start slam-flight
```

---

## Why This Order Works Better

**Check AFTER SLAM launches:**
- ✅ SLAM has time to initialize
- ✅ SLAM is actually generating odometry (real verification)
- ✅ Can detect SLAM failures immediately
- ✅ If checks fail, SLAM stays running so you can debug
- ✅ True "flight ready" confirmation

---

## Flight Checklist

**Before running any startup script:**

### Remote Check (from laptop)
```bash
# SSH to Jetson
ssh dev@<jetson-ip>

# Run startup script
/home/dev/slam-agent/flight_startup.py
```

### On Jetson (Direct)
```bash
# Terminal 1: Run preflight + SLAM
/home/dev/slam-agent/flight_startup.py

# Terminal 2: Monitor odometry
rostopic hz /Odometry

# Terminal 3: Record data (optional)
rosbag record /Odometry /cloud_registered /ouster/points
```

---

## Integrated into Launch File

If you want SLAM to always run with preflight:

```bash
cd /home/dev/slam_ws

# Use the integrated launch file
roslaunch fast_lio mapping_ouster64_with_preflight.launch
```

This combines everything in one command.

---

## Troubleshooting

### Preflight Fails on Startup
1. Check what failed: `preflight_check.sh` will show details
2. Fix the issue (usually network or config)
3. Re-run startup script

### SLAM Crashes After Startup
1. Check logs: `tail ~/.ros/log/latest/laserMapping*`
2. Verify `/ouster/points` is still publishing
3. Restart: `rosnode kill /laserMapping`

### Need to Abort Flight
- **For bash/python scripts:** Press `Ctrl+C`
- **For systemd service:** `sudo systemctl stop slam-flight`

---

## Pro Tips

### Create an Alias for Quick Launch
```bash
# Add to ~/.bashrc
alias start-slam='/home/dev/slam-agent/flight_startup.py'

# Then just type:
start-slam
```

### Monitor Multiple Terminals
```bash
# Terminal 1: Start SLAM with preflight
/home/dev/slam-agent/flight_startup.py

# Terminal 2: Monitor data
watch -n 0.1 'rostopic hz /Odometry'

# Terminal 3: Record mission
rosbag record -o flight_$(date +%Y%m%d_%H%M%S) \
  /Odometry /cloud_registered /ouster/imu
```

### Scheduled Flights
```bash
# Run preflight at specific time
(crontab -l 2>/dev/null; echo "0 9 * * * /home/dev/slam-agent/preflight_check.sh") | crontab -

# Or use at command
echo "/home/dev/slam-agent/flight_startup.py" | at 9:00 AM tomorrow
```

---

## Summary

**Recommended for You:** Option 2 (Python Script)
```bash
/home/dev/slam-agent/flight_startup.py
```

**Why:**
- ✅ Runs preflight automatically
- ✅ Still requires human confirmation (safe)
- ✅ Clear status messages
- ✅ Easy to use: one command
- ✅ Good logging for debugging

**Usage Before Each Flight:**
```bash
# Connect to Jetson
ssh dev@<jetson-ip>

# Start flight
/home/dev/slam-agent/flight_startup.py

# Wait for preflight to pass
# Type "yes" to confirm
# SLAM launches automatically
```

---

**Last Updated:** 2026-02-07
