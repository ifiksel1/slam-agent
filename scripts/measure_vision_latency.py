#!/usr/bin/env python3
"""
SLAM Vision Pipeline Latency Measurement Tool

Measures end-to-end latency in the SLAM-to-MAVROS vision pose pipeline to
determine the optimal VISO_DELAY_MS ArduPilot parameter value.

Pipeline stages measured:
  1. SLAM Output (/Odometry) → Vision Bridge Processing
  2. Vision Bridge → MAVROS Vision Pose (/mavros/vision_pose/pose)
  3. MAVROS Vision Pose → EKF Local Position (/mavros/local_position/pose)
  4. EKF tracking delay via position cross-correlation
  5. Update rates (Hz) for vision_pose and local_position
  6. RMS position tracking error

Usage:
  measure_vision_latency.py [--duration SECONDS] [--json] [--container CONTAINER]

  --duration SECONDS    Measurement duration (default: 30s)
  --json               Output JSON format for MCP compatibility
  --container NAME     Run inside Docker container (via docker exec)
  --verbose            Show per-message latencies during collection

MCP Integration:
  run_diagnostic("measure_vision_latency", "--duration 30 --json")

Output:
  - Mean, median, p95, max latency for each pipeline segment
  - EKF tracking delay (via cross-correlation of position changes)
  - Update rates (Hz) for vision_pose and local_position
  - RMS position tracking error
  - Total end-to-end latency statistics
  - Suggested VISO_DELAY_MS value (ceil(p95_total + 10ms margin))
  - Comparison with current VISO_DELAY_MS if available

Author: SLAM Integration Team
ROS Version: ROS 2 Humble
Platform: ARM64 Jetson Orin NX
"""

import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import signal

# ROS 2 imports (only required when running in ROS environment)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # Dummy classes for help/container mode when ROS not available
    class Node:
        pass
    class Odometry:
        pass
    class PoseStamped:
        pass


class LatencyMeasurement:
    """Single latency measurement with timestamp."""

    def __init__(self, timestamp: float, latency_ms: float):
        self.timestamp = timestamp
        self.latency_ms = latency_ms


class VisionLatencyMeasurer(Node):
    """ROS 2 node to measure vision pipeline latency."""

    def __init__(self, duration: int = 30, verbose: bool = False):
        super().__init__('vision_latency_measurer')

        self.duration = duration
        self.verbose = verbose
        self.start_time = None
        self.is_complete = False

        # Message buffers (keep last 100 messages with timestamps)
        self.odom_buffer = deque(maxlen=100)
        self.vision_pose_buffer = deque(maxlen=100)
        self.local_pose_buffer = deque(maxlen=100)

        # Latency measurements
        self.slam_to_vision_latencies = []  # /Odometry → /mavros/vision_pose/pose
        self.vision_to_ekf_latencies = []   # /mavros/vision_pose/pose → /mavros/local_position/pose
        self.end_to_end_latencies = []      # /Odometry → /mavros/local_position/pose

        # Topic availability flags
        self.odom_received = False
        self.vision_pose_received = False
        self.local_pose_received = False

        # Time-series data for cross-correlation and Hz measurement
        self.vision_pose_timestamps = []  # For Hz calculation
        self.local_pose_timestamps = []   # For Hz calculation
        self.vision_positions = []        # (time, x, y, z) for cross-correlation
        self.local_positions = []         # (time, x, y, z) for cross-correlation

        # QoS profile compatible with MAVROS (BEST_EFFORT) and FAST-LIO
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            sensor_qos
        )

        self.vision_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.vision_pose_callback,
            sensor_qos
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            sensor_qos
        )

        # Timer to check completion
        self.timer = self.create_timer(0.5, self.check_completion)

        self.get_logger().info(f'Measuring vision pipeline latency for {duration}s...')
        self.get_logger().info('Waiting for topics: /Odometry, /mavros/vision_pose/pose, /mavros/local_position/pose')

    def odom_callback(self, msg: Odometry):
        """Store SLAM odometry messages with receive timestamp."""
        recv_time = time.time()

        # Extract timestamp from message header
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.odom_buffer.append((msg_time, recv_time, msg))

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('✓ /Odometry topic receiving data')

    def vision_pose_callback(self, msg: PoseStamped):
        """Store vision pose messages and compute SLAM→Vision latency."""
        recv_time = time.time()

        # Extract timestamp
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.vision_pose_buffer.append((msg_time, recv_time, msg))

        # Store for Hz calculation and cross-correlation (only during measurement)
        if self.start_time:
            self.vision_pose_timestamps.append(recv_time)
            pos = msg.pose.position
            self.vision_positions.append((recv_time, pos.x, pos.y, pos.z))

        if not self.vision_pose_received:
            self.vision_pose_received = True
            self.get_logger().info('✓ /mavros/vision_pose/pose topic receiving data')

        # Find closest SLAM odometry message
        if self.start_time and self.odom_buffer:
            closest_odom = self._find_closest_message(self.odom_buffer, msg_time)
            if closest_odom:
                odom_time = closest_odom[0]
                latency_ms = (msg_time - odom_time) * 1000.0

                # Only record if latency is reasonable (0-500ms)
                if 0 <= latency_ms <= 500:
                    self.slam_to_vision_latencies.append(latency_ms)

                    if self.verbose and len(self.slam_to_vision_latencies) % 10 == 0:
                        self.get_logger().info(
                            f'SLAM→Vision: {latency_ms:.1f}ms (n={len(self.slam_to_vision_latencies)})'
                        )

    def local_pose_callback(self, msg: PoseStamped):
        """Store local position messages and compute latencies."""
        recv_time = time.time()

        # Extract timestamp
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.local_pose_buffer.append((msg_time, recv_time, msg))

        # Store for Hz calculation and cross-correlation (only during measurement)
        if self.start_time:
            self.local_pose_timestamps.append(recv_time)
            pos = msg.pose.position
            self.local_positions.append((recv_time, pos.x, pos.y, pos.z))

        if not self.local_pose_received:
            self.local_pose_received = True
            self.get_logger().info('✓ /mavros/local_position/pose topic receiving data')

        # Compute vision→EKF latency
        if self.start_time and self.vision_pose_buffer:
            closest_vision = self._find_closest_message(self.vision_pose_buffer, msg_time)
            if closest_vision:
                vision_time = closest_vision[0]
                latency_ms = (msg_time - vision_time) * 1000.0

                if 0 <= latency_ms <= 500:
                    self.vision_to_ekf_latencies.append(latency_ms)

        # Compute end-to-end latency (SLAM→EKF)
        if self.start_time and self.odom_buffer:
            closest_odom = self._find_closest_message(self.odom_buffer, msg_time)
            if closest_odom:
                odom_time = closest_odom[0]
                latency_ms = (msg_time - odom_time) * 1000.0

                if 0 <= latency_ms <= 1000:
                    self.end_to_end_latencies.append(latency_ms)

                    if self.verbose and len(self.end_to_end_latencies) % 10 == 0:
                        self.get_logger().info(
                            f'End-to-End: {latency_ms:.1f}ms (n={len(self.end_to_end_latencies)})'
                        )

    def _find_closest_message(self, buffer: deque, target_time: float) -> Optional[Tuple]:
        """Find message in buffer with timestamp closest to target."""
        if not buffer:
            return None

        closest = None
        min_diff = float('inf')

        for msg_tuple in buffer:
            msg_time = msg_tuple[0]
            diff = abs(msg_time - target_time)
            if diff < min_diff:
                min_diff = diff
                closest = msg_tuple

        # Only return if within 200ms window
        if min_diff <= 0.2:
            return closest
        return None

    def check_completion(self):
        """Check if measurement is complete."""
        # Start timer once all topics are receiving
        if self.start_time is None:
            if self.odom_received and self.vision_pose_received:
                self.start_time = time.time()
                self.get_logger().info(f'All required topics active. Starting {self.duration}s measurement...')

        # Check if duration elapsed
        if self.start_time and (time.time() - self.start_time) >= self.duration:
            self.is_complete = True
            self.get_logger().info('Measurement complete.')


def compute_statistics(latencies: List[float]) -> Dict:
    """Compute statistical summary of latency measurements."""
    if not latencies:
        return {
            'count': 0,
            'mean_ms': 0.0,
            'median_ms': 0.0,
            'p95_ms': 0.0,
            'max_ms': 0.0,
            'min_ms': 0.0
        }

    sorted_latencies = sorted(latencies)
    n = len(sorted_latencies)

    return {
        'count': n,
        'mean_ms': statistics.mean(latencies),
        'median_ms': statistics.median(latencies),
        'p95_ms': sorted_latencies[int(n * 0.95)] if n > 0 else 0.0,
        'max_ms': max(latencies),
        'min_ms': min(latencies)
    }


def compute_update_rate_hz(timestamps: List[float]) -> float:
    """Compute average update rate in Hz from timestamps."""
    if len(timestamps) < 2:
        return 0.0

    # Calculate intervals between consecutive timestamps
    intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]

    # Filter out outliers (gaps > 1 second suggest data loss)
    valid_intervals = [dt for dt in intervals if 0.001 < dt < 1.0]

    if not valid_intervals:
        return 0.0

    mean_interval = statistics.mean(valid_intervals)
    return 1.0 / mean_interval if mean_interval > 0 else 0.0


def compute_ekf_tracking_delay(vision_positions: List[Tuple], local_positions: List[Tuple]) -> Dict:
    """
    Compute EKF tracking delay via cross-correlation of position changes.

    Strategy:
    1. Compute velocity (position delta) for both vision_pose and local_position
    2. Resample both signals to uniform time grid
    3. Cross-correlate the velocity signals
    4. Find time lag that maximizes correlation (EKF tracking delay)

    Returns dict with mean, p50, p95, max delay in milliseconds.
    """
    if len(vision_positions) < 10 or len(local_positions) < 10:
        return {'mean': 0.0, 'p50': 0.0, 'p95': 0.0, 'max': 0.0, 'samples': 0}

    try:
        # Convert to numpy arrays
        vision_times = np.array([p[0] for p in vision_positions])
        vision_pos = np.array([[p[1], p[2], p[3]] for p in vision_positions])

        local_times = np.array([p[0] for p in local_positions])
        local_pos = np.array([[p[1], p[2], p[3]] for p in local_positions])

        # Compute position magnitude changes (velocity proxy)
        vision_vel = np.sqrt(np.sum(np.diff(vision_pos, axis=0)**2, axis=1))
        local_vel = np.sqrt(np.sum(np.diff(local_pos, axis=0)**2, axis=1))

        # Time vectors for velocity (mid-points between position samples)
        vision_vel_times = (vision_times[:-1] + vision_times[1:]) / 2.0
        local_vel_times = (local_times[:-1] + local_times[1:]) / 2.0

        # Determine common time range
        t_start = max(vision_vel_times[0], local_vel_times[0])
        t_end = min(vision_vel_times[-1], local_vel_times[-1])

        if t_end <= t_start:
            return {'mean': 0.0, 'p50': 0.0, 'p95': 0.0, 'max': 0.0, 'samples': 0}

        # Resample to uniform 100 Hz grid
        sample_rate = 100.0  # Hz
        t_uniform = np.arange(t_start, t_end, 1.0/sample_rate)

        if len(t_uniform) < 50:  # Need at least 0.5s of data
            return {'mean': 0.0, 'p50': 0.0, 'p95': 0.0, 'max': 0.0, 'samples': 0}

        # Interpolate velocity signals to uniform grid
        vision_vel_uniform = np.interp(t_uniform, vision_vel_times, vision_vel)
        local_vel_uniform = np.interp(t_uniform, local_vel_times, local_vel)

        # Normalize signals (zero mean, unit variance)
        vision_vel_norm = (vision_vel_uniform - np.mean(vision_vel_uniform)) / (np.std(vision_vel_uniform) + 1e-9)
        local_vel_norm = (local_vel_uniform - np.mean(local_vel_uniform)) / (np.std(local_vel_uniform) + 1e-9)

        # Cross-correlate (local follows vision, so local is the delayed signal)
        correlation = signal.correlate(local_vel_norm, vision_vel_norm, mode='full', method='auto')
        lags = signal.correlation_lags(len(local_vel_norm), len(vision_vel_norm), mode='full')

        # Convert lags to time (milliseconds)
        lag_times_ms = lags / sample_rate * 1000.0

        # Find peak correlation (but only for positive lags, since EKF follows vision)
        positive_mask = lag_times_ms >= 0
        positive_corr = correlation[positive_mask]
        positive_lags = lag_times_ms[positive_mask]

        if len(positive_corr) == 0:
            return {'mean': 0.0, 'p50': 0.0, 'p95': 0.0, 'max': 0.0, 'samples': 0}

        # Find lag at peak correlation
        peak_idx = np.argmax(positive_corr)
        delay_ms = positive_lags[peak_idx]

        # For robustness, also compute weighted average around peak
        # Take top 10% of correlations
        threshold = np.percentile(positive_corr, 90)
        strong_mask = positive_corr >= threshold

        if np.sum(strong_mask) > 0:
            strong_lags = positive_lags[strong_mask]
            strong_corr = positive_corr[strong_mask]

            # Weighted mean
            delay_mean = np.average(strong_lags, weights=strong_corr)
            delay_p50 = np.median(strong_lags)
            delay_p95 = np.percentile(strong_lags, 95)
            delay_max = np.max(strong_lags)
        else:
            delay_mean = delay_ms
            delay_p50 = delay_ms
            delay_p95 = delay_ms
            delay_max = delay_ms

        return {
            'mean': float(delay_mean),
            'p50': float(delay_p50),
            'p95': float(delay_p95),
            'max': float(delay_max),
            'samples': len(vision_positions)
        }

    except Exception as e:
        # If cross-correlation fails, return zeros
        return {'mean': 0.0, 'p50': 0.0, 'p95': 0.0, 'max': 0.0, 'samples': 0, 'error': str(e)}


def compute_position_rms_error(vision_positions: List[Tuple], local_positions: List[Tuple]) -> float:
    """
    Compute RMS position error between vision_pose and local_position.

    Synchronizes positions by timestamp, then computes 3D Euclidean distance.
    Returns RMS error in meters.
    """
    if len(vision_positions) < 2 or len(local_positions) < 2:
        return 0.0

    try:
        # For each vision position, find closest local position in time
        errors = []

        for v_time, v_x, v_y, v_z in vision_positions:
            # Find closest local position
            min_dt = float('inf')
            closest_local = None

            for l_time, l_x, l_y, l_z in local_positions:
                dt = abs(l_time - v_time)
                if dt < min_dt:
                    min_dt = dt
                    closest_local = (l_x, l_y, l_z)

            # Only use if synchronized within 100ms
            if min_dt < 0.1 and closest_local:
                l_x, l_y, l_z = closest_local
                error = math.sqrt((v_x - l_x)**2 + (v_y - l_y)**2 + (v_z - l_z)**2)
                errors.append(error)

        if not errors:
            return 0.0

        # Compute RMS
        rms = math.sqrt(sum(e**2 for e in errors) / len(errors))
        return rms

    except Exception:
        return 0.0


def get_current_viso_delay() -> Optional[int]:
    """Read current VISO_DELAY_MS parameter from MAVROS if available."""
    try:
        # Try to read from mavros/param/get service
        result = subprocess.run(
            ['ros2', 'service', 'call', '/mavros/param/get',
             'mavros_msgs/srv/ParamGet', '{param_id: "VISO_DELAY_MS"}'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode == 0 and 'value' in result.stdout:
            # Parse the integer value
            for line in result.stdout.split('\n'):
                if 'integer:' in line:
                    value = int(line.split(':')[1].strip())
                    return value
    except Exception:
        pass

    return None


def suggest_viso_delay(p95_total_ms: float, current_value: Optional[int] = None) -> int:
    """Generate VISO_DELAY_MS recommendation (just the value)."""
    # Add 10ms safety margin and round up
    suggested = math.ceil(p95_total_ms + 10.0)
    return suggested


def format_compact_json_output(results: Dict) -> Dict:
    """Format results for compact JSON output matching requested schema."""
    slam_vision = results['latencies']['slam_to_vision']
    ekf_delay = results['ekf_tracking_delay']

    output = {
        'vision_pose_hz': results['vision_pose_hz'],
        'local_position_hz': results['local_position_hz'],
        'slam_to_vision_bridge_ms': {
            'mean': slam_vision['mean_ms'],
            'p50': slam_vision['median_ms'],
            'p95': slam_vision['p95_ms'],
            'max': slam_vision['max_ms']
        },
        'ekf_tracking_delay_ms': {
            'mean': ekf_delay['mean'],
            'p50': ekf_delay['p50'],
            'p95': ekf_delay['p95'],
            'max': ekf_delay['max']
        },
        'position_rms_error_m': results['position_rms_error_m'],
        'suggested_viso_delay_ms': results['suggested_viso_delay_ms'],
        'samples': slam_vision['count'],
        'duration_s': results['measurement_duration_s']
    }

    # Add current viso delay if available
    if results.get('current_viso_delay_ms') is not None:
        output['current_viso_delay_ms'] = results['current_viso_delay_ms']

    return output


def print_human_readable_report(results: Dict):
    """Print human-readable latency report."""
    print('\n' + '='*70)
    print('SLAM Vision Pipeline Latency Measurement Report')
    print('='*70)
    print(f'Measurement Duration: {results["measurement_duration_s"]}s')
    print(f'Timestamp: {results["timestamp"]}')
    print()

    # Update rates
    print('Topic Update Rates:')
    print(f'  /mavros/vision_pose/pose:     {results["vision_pose_hz"]:6.1f} Hz')
    print(f'  /mavros/local_position/pose:  {results["local_position_hz"]:6.1f} Hz')
    print()

    # SLAM → Vision latency
    slam_vision = results['latencies']['slam_to_vision']
    print('Stage 1: SLAM Output → Vision Bridge')
    print(f'  Topic: /Odometry → /mavros/vision_pose/pose')
    print(f'  Samples: {slam_vision["count"]}')
    if slam_vision['count'] > 0:
        print(f'  Mean:    {slam_vision["mean_ms"]:6.1f} ms')
        print(f'  Median:  {slam_vision["median_ms"]:6.1f} ms')
        print(f'  P95:     {slam_vision["p95_ms"]:6.1f} ms')
        print(f'  Max:     {slam_vision["max_ms"]:6.1f} ms')
    else:
        print('  [No data collected]')
    print()

    # EKF tracking delay
    ekf_delay = results['ekf_tracking_delay']
    print('Stage 2: EKF Tracking Delay (Cross-Correlation)')
    print(f'  Measures how long EKF takes to track vision pose changes')
    if ekf_delay['samples'] > 0:
        print(f'  Mean:    {ekf_delay["mean"]:6.1f} ms')
        print(f'  Median:  {ekf_delay["p50"]:6.1f} ms')
        print(f'  P95:     {ekf_delay["p95"]:6.1f} ms')
        print(f'  Max:     {ekf_delay["max"]:6.1f} ms')
        print(f'  Samples: {ekf_delay["samples"]}')
    else:
        print('  [Insufficient data for cross-correlation analysis]')
    print()

    # Position tracking error
    print('Position Tracking Accuracy:')
    print(f'  RMS Error: {results["position_rms_error_m"]:6.3f} m')
    print()

    # Vision → EKF latency (legacy)
    vision_ekf = results['latencies']['vision_to_ekf']
    print('Legacy: Vision Pose → EKF Local Position (Timestamp-Based)')
    print(f'  Topic: /mavros/vision_pose/pose → /mavros/local_position/pose')
    print(f'  Samples: {vision_ekf["count"]}')
    if vision_ekf['count'] > 0:
        print(f'  Mean:    {vision_ekf["mean_ms"]:6.1f} ms')
        print(f'  Median:  {vision_ekf["median_ms"]:6.1f} ms')
        print(f'  P95:     {vision_ekf["p95_ms"]:6.1f} ms')
        print(f'  Max:     {vision_ekf["max_ms"]:6.1f} ms')
    else:
        print('  [No data collected - /mavros/local_position/pose may not be publishing]')
    print()

    # End-to-end latency (legacy)
    e2e = results['latencies']['end_to_end']
    print('Legacy: Total Pipeline SLAM → EKF (Timestamp-Based)')
    print(f'  Topic: /Odometry → /mavros/local_position/pose')
    print(f'  Samples: {e2e["count"]}')
    if e2e['count'] > 0:
        print(f'  Mean:    {e2e["mean_ms"]:6.1f} ms')
        print(f'  Median:  {e2e["median_ms"]:6.1f} ms')
        print(f'  P95:     {e2e["p95_ms"]:6.1f} ms')
        print(f'  Max:     {e2e["max_ms"]:6.1f} ms')
    else:
        print('  [No data collected]')
    print()

    # Recommendation
    print('─'*70)
    print('VISO_DELAY_MS Recommendation:')
    print(f'  Suggested Value: {results["suggested_viso_delay_ms"]} ms')
    print(f'  Basis: SLAM→Vision P95 ({slam_vision["p95_ms"]:.1f}ms) + EKF delay P95 ({ekf_delay["p95"]:.1f}ms) + 10ms margin')

    if results.get('current_viso_delay_ms') is not None:
        current = results['current_viso_delay_ms']
        suggested = results['suggested_viso_delay_ms']
        diff = current - suggested

        print(f'  Current Value:   {current} ms')

        if diff < -20:
            print(f'  Assessment:      TOO LOW')
            print(f'\n  ⚠ WARNING: Current value ({current}ms) is {-diff:.0f}ms below recommended.')
            print('             May cause EKF to use stale vision data.')
        elif diff > 50:
            print(f'  Assessment:      TOO HIGH')
            print(f'\n  ⚠ WARNING: Current value ({current}ms) is {diff:.0f}ms above recommended.')
            print('             Excessive delay reduces fusion responsiveness.')
        else:
            print(f'  Assessment:      ACCEPTABLE')
            print(f'\n  ✓ Current value ({current}ms) is within acceptable range.')
    else:
        print('  Current Value:   [Not available - MAVROS not responding]')

    print('='*70)
    print()

    # Error conditions
    if results.get('errors'):
        print('⚠ Errors/Warnings:')
        for error in results['errors']:
            print(f'  - {error}')
        print()


def run_in_container(container: str, args: List[str]) -> int:
    """Execute this script inside a Docker container."""
    script_path = '/home/dev/slam-agent/scripts/measure_vision_latency.py'

    cmd = [
        'docker', 'exec', '-i', container,
        'bash', '-c',
        f'source /opt/ros/humble/setup.bash && python3 {script_path} {" ".join(args)}'
    ]

    result = subprocess.run(cmd)
    return result.returncode


def main():
    parser = argparse.ArgumentParser(
        description='Measure SLAM-to-MAVROS vision pipeline latency',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Measure for 30 seconds with human-readable output
  measure_vision_latency.py --duration 30

  # JSON output for MCP
  measure_vision_latency.py --duration 30 --json

  # Run inside Docker container
  measure_vision_latency.py --container slam-gpu --duration 30 --json

  # Verbose mode (show per-message latencies)
  measure_vision_latency.py --duration 30 --verbose
        """
    )

    parser.add_argument(
        '--duration',
        type=int,
        default=30,
        help='Measurement duration in seconds (default: 30)'
    )

    parser.add_argument(
        '--json',
        action='store_true',
        help='Output results in JSON format for MCP compatibility'
    )

    parser.add_argument(
        '--container',
        type=str,
        help='Run inside specified Docker container (via docker exec)'
    )

    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Show per-message latencies during collection'
    )

    args = parser.parse_args()

    # If container specified, delegate to container execution
    if args.container:
        container_args = []
        if args.duration != 30:
            container_args.extend(['--duration', str(args.duration)])
        if args.json:
            container_args.append('--json')
        if args.verbose:
            container_args.append('--verbose')

        return run_in_container(args.container, container_args)

    # Check if ROS 2 is available
    if not ROS_AVAILABLE:
        print('ERROR: ROS 2 (rclpy) not available in current environment', file=sys.stderr)
        print('', file=sys.stderr)
        print('This script must be run either:', file=sys.stderr)
        print('  1. Inside a ROS 2 environment (source /opt/ros/humble/setup.bash)', file=sys.stderr)
        print('  2. With --container flag to run inside a Docker container', file=sys.stderr)
        print('', file=sys.stderr)
        print('Example: measure_vision_latency.py --container slam-gpu --duration 30', file=sys.stderr)
        return 1

    # Initialize ROS 2
    rclpy.init()

    measurer = VisionLatencyMeasurer(duration=args.duration, verbose=args.verbose)
    executor = MultiThreadedExecutor()
    executor.add_node(measurer)

    # Wait for topics with timeout
    timeout = 5.0
    start_wait = time.time()

    while not (measurer.odom_received and measurer.vision_pose_received):
        if time.time() - start_wait > timeout:
            print('ERROR: Timeout waiting for topics', file=sys.stderr)
            print('  /Odometry:', '✓' if measurer.odom_received else '✗', file=sys.stderr)
            print('  /mavros/vision_pose/pose:', '✓' if measurer.vision_pose_received else '✗', file=sys.stderr)

            rclpy.shutdown()
            return 1

        executor.spin_once(timeout_sec=0.1)

    # Run measurement
    while not measurer.is_complete:
        executor.spin_once(timeout_sec=0.1)

    # Compute statistics
    slam_vision_stats = compute_statistics(measurer.slam_to_vision_latencies)
    vision_ekf_stats = compute_statistics(measurer.vision_to_ekf_latencies)
    e2e_stats = compute_statistics(measurer.end_to_end_latencies)

    # Compute update rates
    vision_pose_hz = compute_update_rate_hz(measurer.vision_pose_timestamps)
    local_position_hz = compute_update_rate_hz(measurer.local_pose_timestamps)

    # Compute EKF tracking delay via cross-correlation
    ekf_tracking_delay = compute_ekf_tracking_delay(
        measurer.vision_positions,
        measurer.local_positions
    )

    # Compute position RMS error
    position_rms_error = compute_position_rms_error(
        measurer.vision_positions,
        measurer.local_positions
    )

    # Get current VISO_DELAY_MS
    current_viso = get_current_viso_delay()

    # Compute suggested VISO_DELAY_MS based on P95 of (SLAM→Vision + EKF tracking delay)
    combined_p95 = slam_vision_stats['p95_ms'] + ekf_tracking_delay['p95']
    suggested_viso = suggest_viso_delay(combined_p95, current_viso)

    # Build results
    results = {
        'timestamp': datetime.now().isoformat(),
        'measurement_duration_s': args.duration,
        'vision_pose_hz': vision_pose_hz,
        'local_position_hz': local_position_hz,
        'latencies': {
            'slam_to_vision': slam_vision_stats,
            'vision_to_ekf': vision_ekf_stats,
            'end_to_end': e2e_stats
        },
        'ekf_tracking_delay': ekf_tracking_delay,
        'position_rms_error_m': position_rms_error,
        'suggested_viso_delay_ms': suggested_viso,
        'current_viso_delay_ms': current_viso,
        'errors': []
    }

    # Add warnings for missing data
    if slam_vision_stats['count'] == 0:
        results['errors'].append('No SLAM→Vision latency data collected (topics not synchronized?)')

    if e2e_stats['count'] == 0:
        results['errors'].append('No end-to-end latency data collected')

    if not measurer.local_pose_received:
        results['errors'].append('/mavros/local_position/pose not publishing (EKF may not be fusing vision data)')

    if ekf_tracking_delay['samples'] == 0:
        results['errors'].append('Insufficient data for EKF tracking delay cross-correlation analysis')

    # Shutdown ROS
    rclpy.shutdown()

    # Output results
    if args.json:
        compact_output = format_compact_json_output(results)
        print(json.dumps(compact_output, indent=2))
    else:
        print_human_readable_report(results)

    # Exit with error if no data collected
    if slam_vision_stats['count'] == 0:
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
