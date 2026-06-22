# COIN-LIO vs FAST-LIO GPU Comparison (2026-02-22)

## Setup
- **Sensor:** Ouster OS1-64, 1024x20 mode (20Hz)
- **Platform:** Jetson Orin NX 8GB
- **COIN-LIO:** ROS 1 Noetic, native on host
- **FAST-LIO GPU:** ROS 2 Humble, Docker container (`slam_gpu_system`)
- **Test:** 60s bag with hand-motion, replayed through both algorithms

## Matched Config (Fair Comparison)
Both use: `filter_size: 0.5m`, `det_range: 150m`, `point_filter_num: 4`, `max_iteration: 5`

## Results

| Metric | COIN-LIO | FAST-LIO GPU |
|--------|----------|--------------|
| Frequency | 20.0 Hz | 19.9 Hz |
| Trajectory Length | 6.3 m | 7.9 m |
| Net Displacement | 1.478 m | 0.130 m |
| Z Drift (range) | 0.240 m | 0.645 m |
| Computation (avg) | 35 ms | ~50 ms |

## Key Findings
1. Both run at full 20Hz with matching filter params
2. FAST-LIO GPU anchors to map better (less XY drift) but has more Z drift
3. COIN-LIO accumulates ~3cm/s XY drift in this scenario
4. FAST-LIO GPU "GPU" is misleading — only voxel downsampling uses CUDA, everything else is CPU
5. Without ground truth, hard to determine absolute accuracy winner

## Critical Config Discoveries

### Extrinsic Rotation (IMU-to-LiDAR)
- **ROS 1 Ouster driver:** Identity `[1,0,0, 0,1,0, 0,0,1]` (COIN-LIO default)
- **ROS 2 Ouster driver:** 180-deg yaw `[-1,0,0, 0,-1,0, 0,0,1]` (different frame convention)
- Using wrong extrinsic causes instant divergence (363km in 11s)

### Point Type (ROS 2 Ouster driver)
- `point_type: xyzi` → 32 bytes, only x/y/z/intensity — **BREAKS** per-point timestamps
- `point_type: original` → 48 bytes, includes `t`, `ring`, `reflectivity`, `ambient`, `range`
- FAST-LIO GPU needs `t` field for proper deskewing; without it, approximates from column index → poor odometry

### Bag Conversion Caveat
- ROS 1 ↔ ROS 2 bag conversion preserves field layout but NOT frame conventions
- Cannot replay ROS 2 driver data through COIN-LIO (wrong coordinate frame → divergence)
- Must record from ROS 1 driver for COIN-LIO, then convert to ROS 2 for FAST-LIO GPU comparison
