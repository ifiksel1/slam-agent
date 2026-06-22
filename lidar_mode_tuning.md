# LiDAR Mode Tuning for FAST-LIO

## Key Finding (2026-02-21, Jetson Orin NX 8GB + Ouster OS1-64)

The LiDAR scan mode must be matched to the compute platform's processing capability. FAST-LIO processes each scan sequentially — if processing time exceeds the inter-scan interval, frames are dropped.

## Ouster Mode Performance on Jetson Orin NX

| Mode | Points/scan | Scan rate | FAST-LIO output | Notes |
|------|------------|-----------|-----------------|-------|
| 1024x10 | ~65K | 10Hz | **10Hz** | No drops, but low update rate |
| 1024x20 | ~32K | 20Hz | **14Hz** | Drops ~30% of frames, GPU can't keep up |
| 512x20 | ~16K | 20Hz | **20Hz** | No drops — sweet spot for Jetson |

## Tuning Approach

1. Start with highest resolution at desired scan rate (e.g., 1024x20)
2. Check if FAST-LIO output rate matches input rate (`ros2 topic hz /Odometry` inside container)
3. If dropping frames, halve horizontal resolution (1024→512) and retest
4. Target: FAST-LIO output rate == LiDAR scan rate (no drops)

## Config Files to Change

Two files must be updated together:
- `config/ouster_driver.yaml` → `lidar_mode: '512x20'` (actual driver config)
- `config/fast_lio_gpu.yaml` → `scan_rate: 20` (must match LiDAR rate)
- `docker-compose.yml` → `OUSTER_LIDAR_MODE` env var is **NOT used** by the driver; it's informational only

## Platform Guidelines (estimated)

| Platform | GPU | Recommended mode | Expected FAST-LIO rate |
|----------|-----|-----------------|----------------------|
| Jetson Orin NX 8GB | Ampere 1024-core | 512x20 | 20Hz |
| Jetson Orin NX 16GB | Ampere 1024-core | 512x20 or 1024x20 | 20Hz / test |
| Jetson AGX Orin 32GB | Ampere 2048-core | 1024x20 | 20Hz (likely) |
| x86 + RTX 3060+ | Desktop GPU | 1024x20 | 20Hz (likely) |
| Raspberry Pi / no GPU | CPU only | 512x10 | 10Hz (maybe) |

These are estimates — always verify with `ros2 topic hz /Odometry` inside the container.

## Full Pipeline Rates Achieved (Jetson Orin NX)

### 512x20 (unstable — SLAM drift in small rooms)
- Ouster: 20Hz, FAST-LIO: 20Hz (no drops), VP: 20Hz, LP: 50Hz
- BUT: insufficient point density caused SLAM tracking loss and 60m+ position jumps
- Not recommended for small indoor environments

### 1024x20 (best stability vs rate tradeoff)
- Ouster: 20Hz, FAST-LIO: ~14Hz (drops ~30%), VP: ~14Hz, LP: 20Hz
- Stable tracking, no drift
- FAST-LIO can't fully keep up but gracefully drops frames

### 1024x10 (most conservative)
- Ouster: 10Hz, FAST-LIO: 10Hz (no drops), VP: 10Hz, LP: 12.5Hz
- Rock solid but low update rate

## Key Discovery: SERIAL0_BAUD Controls LP Rate

ArduPilot's `SERIAL0_BAUD` parameter directly controls LP output rate:
- `SERIAL0_BAUD=115` (115200) → LP stuck at 5-12.5Hz regardless of stream rate commands
- `SERIAL0_BAUD=921` (921600) → LP at 20Hz with stream_rate=20
- Must reboot FC after changing (`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, command 246)
- `set_stream_rate` and `SET_MESSAGE_INTERVAL` commands are accepted but ignored at low baud

## FAST-LIO Output Rate is LiDAR-locked

FAST-LIO publishes /Odometry once per LiDAR scan — NOT at IMU rate.
IMU pre-integration happens internally for motion undistortion but the propagated
state is not published between scans. To get higher output (e.g., 50Hz VP),
would need code changes to publish IMU-propagated poses between scans.
See `laserMapping.cpp:sync_packages()` and `IMU_Processing.hpp:UndistortPcl()`.
Source: https://github.com/OmerMersin/FAST_LIO_GPU.git
