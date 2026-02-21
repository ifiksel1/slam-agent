#!/usr/bin/env python3
"""
Flight Data Analysis Dashboard Generator

Generates self-contained HTML dashboards from ROS 1 (.bag) or ROS 2 (mcap) flight logs.

Usage:
  flight_analysis.py 001                    # Full HTML report for flight 001
  flight_analysis.py 001 --json             # JSON summary only
  flight_analysis.py --bag /path/to/bag     # Analyze arbitrary bag file
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple, Iterator
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta
import warnings
import base64
from io import BytesIO

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from scipy import signal, interpolate
from jinja2 import Template

warnings.filterwarnings('ignore')

# ==============================================================================
# Constants
# ==============================================================================

FLIGHT_DATA_ROOT = Path("/home/dev/slam-agent/flights")

# Topic names
TOPICS = {
    'slam_odom': '/fast_lio_gpu/odometry',
    'ekf_pose': '/mavros/local_position/pose',
    'state': '/mavros/state',
    'battery': '/mavros/battery',
    'rc_out': '/mavros/rc/out',
    'rc_in': '/mavros/rc/in',
    'imu': '/mavros/imu/data',
    'imu_raw': '/mavros/imu/data_raw',
    'setpoint_attitude': '/mavros/setpoint_raw/target_attitude',
    'esc_telemetry': '/mavros/esc_telemetry/telemetry',
    'esc_status': '/mavros/esc_status/status',
    'waypoints': '/mavros/mission/waypoints',
}

# ArduPilot vibration thresholds (m/s²)
VIBRATION_WARNING_THRESHOLD = 60.0
VIBRATION_CRITICAL_THRESHOLD = 100.0

# Ideal hover percentage range
HOVER_PCT_MIN = 40.0
HOVER_PCT_MAX = 60.0

# Typical quadcopter motor KV for RPM estimation fallback
DEFAULT_MOTOR_KV = 2400

# ==============================================================================
# Data Structures
# ==============================================================================

@dataclass
class FlightSummary:
    """Flight summary metrics"""
    flight_id: str
    duration_sec: float
    distance_m: float
    max_speed_mps: float
    max_altitude_m: float
    avg_altitude_m: float
    battery_start_pct: float
    battery_end_pct: float
    battery_consumed_mah: float
    primary_flight_mode: str
    topics_available: List[str]
    timestamp: str

@dataclass
class Message:
    """Generic message container"""
    topic: str
    timestamp: float
    data: Dict[str, Any]

# ==============================================================================
# Bag Reader Abstraction
# ==============================================================================

class BagReader:
    """Unified interface for reading ROS 1 .bag and ROS 2 .mcap files"""

    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        self.is_ros2 = self._detect_format()
        self._reader = None
        self._topics = set()

    def _detect_format(self) -> bool:
        """Auto-detect bag format from file extension"""
        if self.bag_path.is_dir():
            # ROS 2 directory-based bag
            return True
        elif self.bag_path.suffix == '.mcap':
            return True
        elif self.bag_path.suffix == '.bag':
            return False
        else:
            raise ValueError(f"Unknown bag format: {self.bag_path}")

    def open(self):
        """Open the bag file"""
        if self.is_ros2:
            self._open_ros2()
        else:
            self._open_ros1()

    def close(self):
        """Close the bag file"""
        if self._reader is not None:
            if self.is_ros2:
                # mcap reader doesn't need explicit close
                pass
            else:
                self._reader.close()

    def _open_ros1(self):
        """Open ROS 1 .bag file"""
        try:
            import rosbag
        except ImportError:
            raise ImportError("rosbag not available. Install with: pip install rosbag roslz4")

        self._reader = rosbag.Bag(str(self.bag_path))
        self._topics = set(self._reader.get_type_and_topic_info()[1].keys())

    def _open_ros2(self):
        """Open ROS 2 .mcap file"""
        try:
            from mcap.reader import make_reader
        except ImportError:
            raise ImportError("mcap libraries not available. Install with: pip install mcap mcap-ros2-support")

        # Resolve mcap file path (ROS 2 bags can be directories containing .mcap)
        if self.bag_path.is_dir():
            mcap_files = list(self.bag_path.glob("*.mcap"))
            if not mcap_files:
                raise FileNotFoundError(f"No .mcap files found in {self.bag_path}")
            self._mcap_path = mcap_files[0]
        else:
            self._mcap_path = self.bag_path

        # Quick scan for topics
        with open(self._mcap_path, 'rb') as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary and summary.channels:
                self._topics = {ch.topic for ch in summary.channels.values()}

    def get_topics(self) -> List[str]:
        """Get list of available topics"""
        return sorted(self._topics)

    def read_messages(self, topics: Optional[List[str]] = None) -> Iterator[Message]:
        """
        Read messages from bag file

        Args:
            topics: List of topics to read (None = all topics)

        Yields:
            Message objects with normalized data
        """
        if self.is_ros2:
            yield from self._read_ros2_messages(topics)
        else:
            yield from self._read_ros1_messages(topics)

    def _read_ros1_messages(self, topics: Optional[List[str]]) -> Iterator[Message]:
        """Read ROS 1 messages"""
        for topic, msg, t in self._reader.read_messages(topics=topics):
            timestamp = t.to_sec()
            data = self._ros1_msg_to_dict(msg)
            yield Message(topic=topic, timestamp=timestamp, data=data)

    def _read_ros2_messages(self, topics: Optional[List[str]]) -> Iterator[Message]:
        """Read ROS 2 messages from mcap"""
        from mcap_ros2.reader import read_ros2_messages

        for msg in read_ros2_messages(str(self._mcap_path), topics=topics):
            # mcap Message.log_time is nanoseconds since epoch
            timestamp = msg.message.log_time * 1e-9
            data = self._ros2_msg_to_dict(msg.ros_msg)
            yield Message(topic=msg.channel.topic, timestamp=timestamp, data=data)

    def _ros1_msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS 1 message to dictionary"""
        result = {}

        # Handle common message types
        if hasattr(msg, '__slots__'):
            for slot in msg.__slots__:
                value = getattr(msg, slot)

                # Recursively convert nested messages
                if hasattr(value, '__slots__'):
                    result[slot] = self._ros1_msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[slot] = [self._ros1_msg_to_dict(v) if hasattr(v, '__slots__') else v
                                   for v in value]
                else:
                    result[slot] = value

        return result

    def _ros2_msg_to_dict(self, msg) -> Dict[str, Any]:
        """Convert ROS 2 message to dictionary"""
        result = {}

        # Handle ROS 2 message structure
        if hasattr(msg, 'get_fields_and_field_types'):
            for field_name in msg.get_fields_and_field_types().keys():
                value = getattr(msg, field_name)

                # Recursively convert nested messages
                if hasattr(value, 'get_fields_and_field_types'):
                    result[field_name] = self._ros2_msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    result[field_name] = [self._ros2_msg_to_dict(v) if hasattr(v, 'get_fields_and_field_types') else v
                                         for v in value]
                else:
                    result[field_name] = value

        return result

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

# ==============================================================================
# Data Extraction
# ==============================================================================

class FlightDataExtractor:
    """Extract and organize flight data from bag file"""

    def __init__(self, bag_path: str):
        self.bag_path = bag_path
        self.data = {}
        self.start_time = None
        self.end_time = None

    def extract(self) -> Dict[str, Any]:
        """Extract all data from bag file"""
        with BagReader(self.bag_path) as bag:
            available_topics = bag.get_topics()

            # Initialize data structures for available topics
            for topic_key, topic_name in TOPICS.items():
                if topic_name in available_topics:
                    self.data[topic_key] = {'timestamps': [], 'messages': []}

            # Read all messages
            for msg in bag.read_messages():
                # Update time range
                if self.start_time is None:
                    self.start_time = msg.timestamp
                self.end_time = msg.timestamp

                # Store message if it's a topic we care about
                for topic_key, topic_name in TOPICS.items():
                    if msg.topic == topic_name:
                        self.data[topic_key]['timestamps'].append(msg.timestamp)
                        self.data[topic_key]['messages'].append(msg.data)
                        break

        # Convert lists to numpy arrays where appropriate
        self._process_extracted_data()

        return self.data

    def _process_extracted_data(self):
        """Post-process extracted data into analyzable format"""
        # Convert timestamps to relative time from start
        if self.start_time is not None:
            for topic_data in self.data.values():
                if topic_data['timestamps']:
                    topic_data['timestamps'] = np.array(topic_data['timestamps']) - self.start_time

# ==============================================================================
# Analysis Functions
# ==============================================================================

class FlightAnalyzer:
    """Analyze flight data and generate metrics"""

    def __init__(self, data: Dict[str, Any], flight_id: str):
        self.data = data
        self.flight_id = flight_id
        self.summary = None

    def analyze_all(self) -> Dict[str, Any]:
        """Run all analysis sections"""
        results = {
            'summary': self._analyze_summary(),
            'slam_vs_ekf': self._analyze_slam_vs_ekf(),
            'motors': self._analyze_motors(),
            'atc': self._analyze_atc_performance(),
            'power': self._analyze_power(),
            'vibration': self._analyze_vibration(),
            'topic_rates': self._analyze_topic_rates(),
        }

        self.summary = results['summary']
        return results

    def _analyze_summary(self) -> Dict[str, Any]:
        """Analyze flight summary metrics"""
        summary = {
            'flight_id': self.flight_id,
            'duration_sec': 0.0,
            'distance_m': 0.0,
            'max_speed_mps': 0.0,
            'max_altitude_m': 0.0,
            'avg_altitude_m': 0.0,
            'battery_start_pct': 0.0,
            'battery_end_pct': 0.0,
            'battery_consumed_mah': 0.0,
            'primary_flight_mode': 'UNKNOWN',
            'topics_available': list(self.data.keys()),
            'timestamp': datetime.now().isoformat(),
        }

        # Duration from SLAM odometry
        if 'slam_odom' in self.data and self.data['slam_odom']['timestamps']:
            times = self.data['slam_odom']['timestamps']
            summary['duration_sec'] = float(times[-1] - times[0])

        # Distance and altitude from SLAM trajectory
        if 'slam_odom' in self.data and self.data['slam_odom']['messages']:
            positions = []
            for msg in self.data['slam_odom']['messages']:
                try:
                    pos = msg['pose']['pose']['position']
                    positions.append([pos['x'], pos['y'], pos['z']])
                except (KeyError, TypeError):
                    continue

            if positions:
                positions = np.array(positions)

                # Calculate distance traveled
                distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
                summary['distance_m'] = float(np.sum(distances))

                # Altitude stats
                summary['max_altitude_m'] = float(np.max(positions[:, 2]))
                summary['avg_altitude_m'] = float(np.mean(positions[:, 2]))

                # Max speed
                if 'slam_odom' in self.data:
                    dt = np.diff(self.data['slam_odom']['timestamps'])
                    speeds = distances / (dt + 1e-9)
                    summary['max_speed_mps'] = float(np.max(speeds))

        # Battery metrics
        if 'battery' in self.data and self.data['battery']['messages']:
            msgs = self.data['battery']['messages']

            # Battery percentage
            if 'percentage' in msgs[0]:
                summary['battery_start_pct'] = float(msgs[0]['percentage'])
                summary['battery_end_pct'] = float(msgs[-1]['percentage'])
            elif 'remaining' in msgs[0]:
                summary['battery_start_pct'] = float(msgs[0]['remaining'] * 100)
                summary['battery_end_pct'] = float(msgs[-1]['remaining'] * 100)

            # Current integration for mAh consumed
            if 'current' in msgs[0]:
                times = self.data['battery']['timestamps']
                currents = np.array([m['current'] for m in msgs])
                dt = np.diff(times)
                # mAh = sum(current[A] * dt[s] * 1000 / 3600)
                summary['battery_consumed_mah'] = float(np.sum(currents[:-1] * dt) * 1000 / 3600)

        # Primary flight mode
        if 'state' in self.data and self.data['state']['messages']:
            modes = [msg.get('mode', 'UNKNOWN') for msg in self.data['state']['messages']]
            if modes:
                # Find most common mode
                from collections import Counter
                summary['primary_flight_mode'] = Counter(modes).most_common(1)[0][0]

        return summary

    def _analyze_slam_vs_ekf(self) -> Dict[str, Any]:
        """Compare SLAM odometry vs EKF local position"""
        result = {
            'available': False,
            'slam_trajectory': [],
            'ekf_trajectory': [],
            'position_error': [],
            'error_stats': {},
        }

        if 'slam_odom' not in self.data or 'ekf_pose' not in self.data:
            return result

        # Extract SLAM trajectory
        slam_times = self.data['slam_odom']['timestamps']
        slam_positions = []
        for msg in self.data['slam_odom']['messages']:
            try:
                pos = msg['pose']['pose']['position']
                slam_positions.append([pos['x'], pos['y'], pos['z']])
            except (KeyError, TypeError):
                slam_positions.append([np.nan, np.nan, np.nan])
        slam_positions = np.array(slam_positions)

        # Extract EKF trajectory
        ekf_times = self.data['ekf_pose']['timestamps']
        ekf_positions = []
        for msg in self.data['ekf_pose']['messages']:
            try:
                pos = msg['pose']['position']
                ekf_positions.append([pos['x'], pos['y'], pos['z']])
            except (KeyError, TypeError):
                ekf_positions.append([np.nan, np.nan, np.nan])
        ekf_positions = np.array(ekf_positions)

        # Interpolate EKF to SLAM timestamps for comparison
        if len(ekf_times) > 1 and len(slam_times) > 1:
            ekf_interp = np.zeros_like(slam_positions)
            for i in range(3):  # x, y, z
                valid_mask = ~np.isnan(ekf_positions[:, i])
                if np.sum(valid_mask) > 1:
                    f = interpolate.interp1d(ekf_times[valid_mask],
                                            ekf_positions[valid_mask, i],
                                            bounds_error=False,
                                            fill_value=np.nan)
                    ekf_interp[:, i] = f(slam_times)

            # Calculate position error
            position_error = np.sqrt(np.sum((slam_positions - ekf_interp)**2, axis=1))

            result['available'] = True
            result['slam_trajectory'] = slam_positions.tolist()
            result['ekf_trajectory'] = ekf_interp.tolist()
            result['position_error'] = position_error.tolist()
            result['error_stats'] = {
                'mean': float(np.nanmean(position_error)),
                'std': float(np.nanstd(position_error)),
                'max': float(np.nanmax(position_error)),
                'rms': float(np.sqrt(np.nanmean(position_error**2))),
            }

        return result

    def _analyze_motors(self) -> Dict[str, Any]:
        """Analyze motor performance and balance"""
        result = {
            'available': False,
            'motor_pwm': {},
            'motor_balance': {},
            'hover_percentage': [],
            'rpm_available': False,
            'motor_rpm': {},
        }

        if 'rc_out' not in self.data:
            return result

        result['available'] = True
        times = self.data['rc_out']['timestamps']

        # Extract PWM values for motors 1-4 (typically channels 0-3)
        motor_pwms = [[], [], [], []]
        for msg in self.data['rc_out']['messages']:
            try:
                channels = msg.get('channels', [])
                for i in range(min(4, len(channels))):
                    motor_pwms[i].append(channels[i])
            except (KeyError, TypeError):
                continue

        # Convert to numpy arrays
        for i in range(4):
            if motor_pwms[i]:
                result['motor_pwm'][f'motor_{i+1}'] = {
                    'timestamps': times.tolist(),
                    'pwm': motor_pwms[i],
                }

        # Calculate motor balance (std dev across motors)
        if all(motor_pwms):
            motor_array = np.array(motor_pwms)
            avg_pwm = np.mean(motor_array, axis=0)
            std_pwm = np.std(motor_array, axis=0)

            result['motor_balance'] = {
                'timestamps': times.tolist(),
                'average_pwm': avg_pwm.tolist(),
                'std_pwm': std_pwm.tolist(),
            }

            # Hover percentage: (avg_pwm - 1000) / 1000 * 100
            hover_pct = (avg_pwm - 1000) / 1000 * 100
            result['hover_percentage'] = {
                'timestamps': times.tolist(),
                'percentage': hover_pct.tolist(),
            }

        # RPM from ESC telemetry if available
        if 'esc_telemetry' in self.data:
            result['rpm_available'] = True
            rpm_times = self.data['esc_telemetry']['timestamps']
            motor_rpms = [[], [], [], []]

            for msg in self.data['esc_telemetry']['messages']:
                try:
                    esc_data = msg.get('esc_telemetry', [])
                    for i in range(min(4, len(esc_data))):
                        motor_rpms[i].append(esc_data[i].get('rpm', 0))
                except (KeyError, TypeError, AttributeError):
                    continue

            for i in range(4):
                if motor_rpms[i]:
                    result['motor_rpm'][f'motor_{i+1}'] = {
                        'timestamps': rpm_times.tolist(),
                        'rpm': motor_rpms[i],
                    }

        return result

    def _analyze_atc_performance(self) -> Dict[str, Any]:
        """Analyze attitude controller tracking performance"""
        result = {
            'available': False,
            'tracking_error': {},
        }

        if 'imu' not in self.data or 'setpoint_attitude' not in self.data:
            return result

        result['available'] = True

        # Get actual angular velocity from IMU (gyro)
        imu_times = self.data['imu']['timestamps']
        actual_rates = {'roll': [], 'pitch': [], 'yaw': []}

        for msg in self.data['imu']['messages']:
            try:
                angular_vel = msg['angular_velocity']
                actual_rates['roll'].append(angular_vel['x'])
                actual_rates['pitch'].append(angular_vel['y'])
                actual_rates['yaw'].append(angular_vel['z'])
            except (KeyError, TypeError):
                actual_rates['roll'].append(np.nan)
                actual_rates['pitch'].append(np.nan)
                actual_rates['yaw'].append(np.nan)

        # Get desired rates from setpoint
        setpoint_times = self.data['setpoint_attitude']['timestamps']
        desired_rates = {'roll': [], 'pitch': [], 'yaw': []}

        for msg in self.data['setpoint_attitude']['messages']:
            try:
                body_rate = msg.get('body_rate', {})
                desired_rates['roll'].append(body_rate.get('x', 0))
                desired_rates['pitch'].append(body_rate.get('y', 0))
                desired_rates['yaw'].append(body_rate.get('z', 0))
            except (KeyError, TypeError):
                desired_rates['roll'].append(0)
                desired_rates['pitch'].append(0)
                desired_rates['yaw'].append(0)

        # Interpolate desired rates to actual timestamps
        for axis in ['roll', 'pitch', 'yaw']:
            desired_array = np.array(desired_rates[axis])
            actual_array = np.array(actual_rates[axis])

            if len(setpoint_times) > 1:
                f = interpolate.interp1d(setpoint_times, desired_array,
                                        bounds_error=False,
                                        fill_value=0)
                desired_interp = f(imu_times)

                # Calculate tracking error
                error = actual_array - desired_interp

                result['tracking_error'][axis] = {
                    'timestamps': imu_times.tolist(),
                    'desired': desired_interp.tolist(),
                    'actual': actual_array.tolist(),
                    'error': error.tolist(),
                    'rms': float(np.sqrt(np.nanmean(error**2))),
                }

        return result

    def _analyze_power(self) -> Dict[str, Any]:
        """Analyze power consumption and efficiency"""
        result = {
            'available': False,
            'voltage': [],
            'current': [],
            'power': [],
            'energy_wh': 0.0,
            'energy_mah': 0.0,
            'efficiency_wh_per_km': 0.0,
        }

        if 'battery' not in self.data:
            return result

        result['available'] = True
        times = self.data['battery']['timestamps']

        voltages = []
        currents = []
        for msg in self.data['battery']['messages']:
            voltages.append(msg.get('voltage', 0))
            currents.append(msg.get('current', 0))

        voltages = np.array(voltages)
        currents = np.array(currents)
        powers = voltages * currents

        result['voltage'] = {
            'timestamps': times.tolist(),
            'values': voltages.tolist(),
        }
        result['current'] = {
            'timestamps': times.tolist(),
            'values': currents.tolist(),
        }
        result['power'] = {
            'timestamps': times.tolist(),
            'values': powers.tolist(),
        }

        # Calculate energy consumption
        if len(times) > 1:
            dt = np.diff(times)
            # Wh = sum(power[W] * dt[s] / 3600)
            energy_wh = np.sum(powers[:-1] * dt / 3600)
            # mAh = sum(current[A] * dt[s] * 1000 / 3600)
            energy_mah = np.sum(currents[:-1] * dt * 1000 / 3600)

            result['energy_wh'] = float(energy_wh)
            result['energy_mah'] = float(energy_mah)

            # Efficiency (Wh/km)
            if self.summary and self.summary['distance_m'] > 0:
                distance_km = self.summary['distance_m'] / 1000
                result['efficiency_wh_per_km'] = float(energy_wh / distance_km)

        return result

    def _analyze_vibration(self) -> Dict[str, Any]:
        """Analyze vibration levels from raw IMU data"""
        result = {
            'available': False,
            'accel_rms': {},
            'fft': {},
            'clipping_detected': False,
        }

        if 'imu_raw' not in self.data:
            return result

        result['available'] = True
        times = self.data['imu_raw']['timestamps']

        # Extract raw accelerometer data
        accel_x, accel_y, accel_z = [], [], []
        for msg in self.data['imu_raw']['messages']:
            try:
                accel = msg['linear_acceleration']
                accel_x.append(accel['x'])
                accel_y.append(accel['y'])
                accel_z.append(accel['z'])
            except (KeyError, TypeError):
                accel_x.append(0)
                accel_y.append(0)
                accel_z.append(0)

        accel_x = np.array(accel_x)
        accel_y = np.array(accel_y)
        accel_z = np.array(accel_z)

        # Calculate RMS for each axis
        for axis, data in [('x', accel_x), ('y', accel_y), ('z', accel_z)]:
            rms = np.sqrt(np.mean(data**2))
            result['accel_rms'][axis] = {
                'value': float(rms),
                'warning': rms > VIBRATION_WARNING_THRESHOLD,
                'critical': rms > VIBRATION_CRITICAL_THRESHOLD,
            }

        # FFT analysis
        if len(times) > 100:
            # Calculate sampling rate
            dt = np.median(np.diff(times))
            fs = 1.0 / dt

            # Compute FFT for each axis
            for axis, data in [('x', accel_x), ('y', accel_y), ('z', accel_z)]:
                # Remove DC component
                data_ac = data - np.mean(data)

                # FFT
                n = len(data_ac)
                fft_vals = np.fft.rfft(data_ac)
                fft_freqs = np.fft.rfftfreq(n, dt)
                fft_magnitude = np.abs(fft_vals) / n

                result['fft'][axis] = {
                    'frequencies': fft_freqs.tolist(),
                    'magnitude': fft_magnitude.tolist(),
                }

        # Clipping detection (accelerometer saturation)
        max_accel = max(np.max(np.abs(accel_x)),
                       np.max(np.abs(accel_y)),
                       np.max(np.abs(accel_z)))
        result['clipping_detected'] = max_accel > 156.0  # ~16g typical IMU limit

        return result

    def _analyze_topic_rates(self) -> Dict[str, Any]:
        """Analyze message rates for all topics"""
        result = {}

        for topic_key, topic_data in self.data.items():
            if not topic_data['timestamps']:
                continue

            times = topic_data['timestamps']
            if len(times) < 2:
                continue

            # Calculate instantaneous rates
            dt = np.diff(times)
            rates = 1.0 / (dt + 1e-9)

            # Detect gaps (rate < 1 Hz)
            gaps = np.where(rates < 1.0)[0]

            result[topic_key] = {
                'mean_rate_hz': float(np.mean(rates)),
                'std_rate_hz': float(np.std(rates)),
                'min_rate_hz': float(np.min(rates)),
                'max_rate_hz': float(np.max(rates)),
                'gap_count': int(len(gaps)),
                'total_messages': len(times),
            }

        return result

# ==============================================================================
# Plotting Functions
# ==============================================================================

class FlightPlotter:
    """Generate plots for flight analysis"""

    def __init__(self, analysis_results: Dict[str, Any], data: Dict[str, Any]):
        self.analysis = analysis_results
        self.data = data

    def plot_all(self) -> Dict[str, str]:
        """Generate all plots and return as base64 encoded strings"""
        plots = {}

        plots['flight_summary'] = self._plot_flight_summary()
        plots['slam_vs_ekf_3d'] = self._plot_slam_vs_ekf_3d()
        plots['slam_vs_ekf_error'] = self._plot_slam_vs_ekf_error()
        plots['motor_pwm'] = self._plot_motor_pwm()
        plots['motor_balance'] = self._plot_motor_balance()
        plots['hover_percentage'] = self._plot_hover_percentage()
        plots['atc_performance'] = self._plot_atc_performance()
        plots['power_analysis'] = self._plot_power_analysis()
        plots['vibration_time'] = self._plot_vibration_time()
        plots['vibration_fft'] = self._plot_vibration_fft()
        plots['topic_rates'] = self._plot_topic_rates()

        return plots

    def _fig_to_base64(self, fig: Figure) -> str:
        """Convert matplotlib figure to base64 encoded PNG"""
        buf = BytesIO()
        fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode('utf-8')
        plt.close(fig)
        return f"data:image/png;base64,{img_base64}"

    def _plot_flight_summary(self) -> str:
        """Plot flight summary timeline"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))

        summary = self.analysis['summary']

        # Battery timeline
        if 'battery' in self.data and self.data['battery']['timestamps']:
            times = np.array(self.data['battery']['timestamps'])
            percentages = []
            for msg in self.data['battery']['messages']:
                pct = msg.get('percentage', msg.get('remaining', 0) * 100)
                percentages.append(pct)

            ax1.plot(times / 60, percentages, 'b-', linewidth=2)
            ax1.set_ylabel('Battery (%)')
            ax1.set_title('Flight Timeline')
            ax1.grid(True, alpha=0.3)
            ax1.set_xlim(left=0)

        # Flight mode timeline
        if 'state' in self.data and self.data['state']['timestamps']:
            times = np.array(self.data['state']['timestamps'])
            modes = [msg.get('mode', 'UNKNOWN') for msg in self.data['state']['messages']]

            # Encode modes as integers for plotting
            unique_modes = sorted(set(modes))
            mode_map = {mode: i for i, mode in enumerate(unique_modes)}
            mode_values = [mode_map[m] for m in modes]

            ax2.plot(times / 60, mode_values, 'g-', linewidth=2)
            ax2.set_ylabel('Flight Mode')
            ax2.set_xlabel('Time (minutes)')
            ax2.set_yticks(range(len(unique_modes)))
            ax2.set_yticklabels(unique_modes)
            ax2.grid(True, alpha=0.3)
            ax2.set_xlim(left=0)

        return self._fig_to_base64(fig)

    def _plot_slam_vs_ekf_3d(self) -> str:
        """Plot 3D trajectory comparison using Plotly"""
        import plotly.graph_objects as go

        slam_vs_ekf = self.analysis['slam_vs_ekf']

        if not slam_vs_ekf['available']:
            # Return empty plot
            fig = go.Figure()
            fig.add_annotation(text="SLAM vs EKF data not available",
                             xref="paper", yref="paper",
                             x=0.5, y=0.5, showarrow=False)
            return fig.to_html(include_plotlyjs='cdn', div_id='slam_3d')

        slam_traj = np.array(slam_vs_ekf['slam_trajectory'])
        ekf_traj = np.array(slam_vs_ekf['ekf_trajectory'])

        fig = go.Figure()

        # SLAM trajectory
        fig.add_trace(go.Scatter3d(
            x=slam_traj[:, 0], y=slam_traj[:, 1], z=slam_traj[:, 2],
            mode='lines',
            name='SLAM',
            line=dict(color='blue', width=3),
        ))

        # EKF trajectory
        fig.add_trace(go.Scatter3d(
            x=ekf_traj[:, 0], y=ekf_traj[:, 1], z=ekf_traj[:, 2],
            mode='lines',
            name='EKF',
            line=dict(color='red', width=3),
        ))

        fig.update_layout(
            title='3D Trajectory: SLAM vs EKF',
            scene=dict(
                xaxis_title='X (m)',
                yaxis_title='Y (m)',
                zaxis_title='Z (m)',
                aspectmode='data',
            ),
            height=600,
        )

        return fig.to_html(include_plotlyjs='cdn', div_id='slam_3d')

    def _plot_slam_vs_ekf_error(self) -> str:
        """Plot position error over time"""
        fig, ax = plt.subplots(figsize=(12, 4))

        slam_vs_ekf = self.analysis['slam_vs_ekf']

        if not slam_vs_ekf['available']:
            ax.text(0.5, 0.5, 'SLAM vs EKF data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        if 'slam_odom' in self.data:
            times = np.array(self.data['slam_odom']['timestamps'])
            error = np.array(slam_vs_ekf['position_error'])

            ax.plot(times / 60, error, 'r-', linewidth=2, label='Position Error')
            ax.axhline(slam_vs_ekf['error_stats']['mean'],
                      color='b', linestyle='--', label=f"Mean: {slam_vs_ekf['error_stats']['mean']:.3f}m")
            ax.set_xlabel('Time (minutes)')
            ax.set_ylabel('Position Error (m)')
            ax.set_title('SLAM vs EKF Position Error')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_xlim(left=0)

        return self._fig_to_base64(fig)

    def _plot_motor_pwm(self) -> str:
        """Plot motor PWM time series"""
        motors = self.analysis['motors']

        if not motors['available']:
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.text(0.5, 0.5, 'Motor data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()

        colors = ['b', 'g', 'r', 'orange']
        for i in range(4):
            motor_key = f'motor_{i+1}'
            if motor_key in motors['motor_pwm']:
                data = motors['motor_pwm'][motor_key]
                times = np.array(data['timestamps'])
                pwm = np.array(data['pwm'])

                axes[i].plot(times / 60, pwm, colors[i], linewidth=1.5)
                axes[i].set_title(f'Motor {i+1}')
                axes[i].set_ylabel('PWM (µs)')
                axes[i].set_xlabel('Time (minutes)')
                axes[i].grid(True, alpha=0.3)
                axes[i].set_xlim(left=0)
                axes[i].set_ylim([1000, 2000])

        fig.suptitle('Motor PWM Signals')
        plt.tight_layout()

        return self._fig_to_base64(fig)

    def _plot_motor_balance(self) -> str:
        """Plot motor balance (std dev across motors)"""
        fig, ax = plt.subplots(figsize=(12, 4))

        motors = self.analysis['motors']

        if not motors['available'] or 'motor_balance' not in motors:
            ax.text(0.5, 0.5, 'Motor balance data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        balance = motors['motor_balance']
        times = np.array(balance['timestamps'])
        std_pwm = np.array(balance['std_pwm'])

        ax.plot(times / 60, std_pwm, 'purple', linewidth=2)
        ax.set_xlabel('Time (minutes)')
        ax.set_ylabel('PWM Std Dev (µs)')
        ax.set_title('Motor Balance (Lower = Better)')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)

        return self._fig_to_base64(fig)

    def _plot_hover_percentage(self) -> str:
        """Plot hover percentage with ideal range"""
        fig, ax = plt.subplots(figsize=(12, 4))

        motors = self.analysis['motors']

        if not motors['available'] or 'hover_percentage' not in motors:
            ax.text(0.5, 0.5, 'Hover percentage data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        hover = motors['hover_percentage']
        times = np.array(hover['timestamps'])
        percentage = np.array(hover['percentage'])

        ax.plot(times / 60, percentage, 'b-', linewidth=2)
        ax.axhspan(HOVER_PCT_MIN, HOVER_PCT_MAX, alpha=0.3, color='green',
                  label=f'Ideal Range ({HOVER_PCT_MIN:.0f}-{HOVER_PCT_MAX:.0f}%)')
        ax.set_xlabel('Time (minutes)')
        ax.set_ylabel('Hover %')
        ax.set_title('Hover Percentage Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)
        ax.set_ylim([0, 100])

        return self._fig_to_base64(fig)

    def _plot_atc_performance(self) -> str:
        """Plot ATC tracking performance"""
        atc = self.analysis['atc']

        if not atc['available']:
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.text(0.5, 0.5, 'ATC data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        fig, axes = plt.subplots(3, 1, figsize=(12, 9))

        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            if axis not in atc['tracking_error']:
                continue

            data = atc['tracking_error'][axis]
            times = np.array(data['timestamps'])
            desired = np.array(data['desired'])
            actual = np.array(data['actual'])

            axes[i].plot(times / 60, desired, 'b--', linewidth=1.5, label='Desired')
            axes[i].plot(times / 60, actual, 'r-', linewidth=1.5, label='Actual')
            axes[i].set_ylabel(f'{axis.title()} Rate (rad/s)')
            axes[i].set_title(f'{axis.title()} Tracking (RMS Error: {data["rms"]:.4f} rad/s)')
            axes[i].legend()
            axes[i].grid(True, alpha=0.3)
            axes[i].set_xlim(left=0)

        axes[-1].set_xlabel('Time (minutes)')
        fig.suptitle('Attitude Controller Performance')
        plt.tight_layout()

        return self._fig_to_base64(fig)

    def _plot_power_analysis(self) -> str:
        """Plot power consumption"""
        power = self.analysis['power']

        if not power['available']:
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.text(0.5, 0.5, 'Power data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        fig, axes = plt.subplots(3, 1, figsize=(12, 9))

        # Voltage
        times = np.array(power['voltage']['timestamps'])
        voltage = np.array(power['voltage']['values'])
        axes[0].plot(times / 60, voltage, 'b-', linewidth=2)
        axes[0].set_ylabel('Voltage (V)')
        axes[0].set_title('Battery Voltage')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_xlim(left=0)

        # Current
        current = np.array(power['current']['values'])
        axes[1].plot(times / 60, current, 'r-', linewidth=2)
        axes[1].set_ylabel('Current (A)')
        axes[1].set_title('Current Draw')
        axes[1].grid(True, alpha=0.3)
        axes[1].set_xlim(left=0)

        # Power
        power_w = np.array(power['power']['values'])
        axes[2].plot(times / 60, power_w, 'g-', linewidth=2)
        axes[2].set_ylabel('Power (W)')
        axes[2].set_xlabel('Time (minutes)')
        axes[2].set_title(f'Power Draw (Total: {power["energy_wh"]:.2f} Wh, '
                         f'Efficiency: {power.get("efficiency_wh_per_km", 0):.1f} Wh/km)')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_xlim(left=0)

        plt.tight_layout()

        return self._fig_to_base64(fig)

    def _plot_vibration_time(self) -> str:
        """Plot vibration time domain"""
        vib = self.analysis['vibration']

        if not vib['available']:
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.text(0.5, 0.5, 'Vibration data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        fig, axes = plt.subplots(3, 1, figsize=(12, 9))

        if 'imu_raw' in self.data:
            times = np.array(self.data['imu_raw']['timestamps'])

            for i, axis in enumerate(['x', 'y', 'z']):
                accel = []
                for msg in self.data['imu_raw']['messages']:
                    try:
                        accel.append(msg['linear_acceleration'][axis])
                    except (KeyError, TypeError):
                        accel.append(0)

                accel = np.array(accel)
                axes[i].plot(times / 60, accel, linewidth=1)
                axes[i].set_ylabel(f'Accel {axis.upper()} (m/s²)')

                # Add threshold lines
                axes[i].axhline(VIBRATION_WARNING_THRESHOLD, color='orange',
                              linestyle='--', label='Warning')
                axes[i].axhline(-VIBRATION_WARNING_THRESHOLD, color='orange', linestyle='--')
                axes[i].axhline(VIBRATION_CRITICAL_THRESHOLD, color='red',
                              linestyle='--', label='Critical')
                axes[i].axhline(-VIBRATION_CRITICAL_THRESHOLD, color='red', linestyle='--')

                # Add RMS value
                rms = vib['accel_rms'][axis]
                axes[i].set_title(f'{axis.upper()} Axis (RMS: {rms["value"]:.2f} m/s²)')
                axes[i].grid(True, alpha=0.3)
                axes[i].set_xlim(left=0)
                if i == 0:
                    axes[i].legend()

        axes[-1].set_xlabel('Time (minutes)')
        fig.suptitle('Vibration Time Domain')
        plt.tight_layout()

        return self._fig_to_base64(fig)

    def _plot_vibration_fft(self) -> str:
        """Plot vibration FFT"""
        vib = self.analysis['vibration']

        if not vib['available'] or 'fft' not in vib or not vib['fft']:
            fig, ax = plt.subplots(figsize=(12, 4))
            ax.text(0.5, 0.5, 'FFT data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        fig, axes = plt.subplots(3, 1, figsize=(12, 9))

        for i, axis in enumerate(['x', 'y', 'z']):
            if axis not in vib['fft']:
                continue

            fft_data = vib['fft'][axis]
            freqs = np.array(fft_data['frequencies'])
            magnitude = np.array(fft_data['magnitude'])

            # Only plot up to 200 Hz
            mask = freqs <= 200
            axes[i].semilogy(freqs[mask], magnitude[mask], linewidth=1.5)
            axes[i].set_ylabel(f'{axis.upper()} Magnitude')
            axes[i].set_title(f'{axis.upper()} Axis FFT')
            axes[i].grid(True, alpha=0.3, which='both')

        axes[-1].set_xlabel('Frequency (Hz)')
        fig.suptitle('Vibration Frequency Domain')
        plt.tight_layout()

        return self._fig_to_base64(fig)

    def _plot_topic_rates(self) -> str:
        """Plot topic message rates"""
        fig, ax = plt.subplots(figsize=(12, 6))

        rates = self.analysis['topic_rates']

        if not rates:
            ax.text(0.5, 0.5, 'Topic rate data not available',
                   ha='center', va='center', transform=ax.transAxes)
            return self._fig_to_base64(fig)

        topics = sorted(rates.keys())
        mean_rates = [rates[t]['mean_rate_hz'] for t in topics]
        std_rates = [rates[t]['std_rate_hz'] for t in topics]

        y_pos = np.arange(len(topics))
        ax.barh(y_pos, mean_rates, xerr=std_rates, alpha=0.7)
        ax.set_yticks(y_pos)
        ax.set_yticklabels(topics)
        ax.set_xlabel('Message Rate (Hz)')
        ax.set_title('Topic Message Rates')
        ax.grid(True, alpha=0.3, axis='x')

        return self._fig_to_base64(fig)

# ==============================================================================
# HTML Report Generation
# ==============================================================================

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Flight Analysis Report - {{ summary.flight_id }}</title>
    <script src="https://cdn.plot.ly/plotly-2.26.0.min.js"></script>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }
        h2 {
            color: #34495e;
            margin-top: 40px;
            border-bottom: 2px solid #ecf0f1;
            padding-bottom: 8px;
        }
        .summary-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        .summary-card {
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 6px;
            border-left: 4px solid #3498db;
        }
        .summary-card h3 {
            margin: 0 0 10px 0;
            color: #7f8c8d;
            font-size: 14px;
            text-transform: uppercase;
        }
        .summary-card .value {
            font-size: 28px;
            font-weight: bold;
            color: #2c3e50;
        }
        .summary-card .unit {
            font-size: 16px;
            color: #7f8c8d;
            margin-left: 5px;
        }
        .plot {
            margin: 20px 0;
        }
        .warning {
            background-color: #fff3cd;
            border-left: 4px solid #ffc107;
            padding: 15px;
            margin: 20px 0;
            border-radius: 4px;
        }
        .critical {
            background-color: #f8d7da;
            border-left: 4px solid #dc3545;
            padding: 15px;
            margin: 20px 0;
            border-radius: 4px;
        }
        .success {
            background-color: #d4edda;
            border-left: 4px solid #28a745;
            padding: 15px;
            margin: 20px 0;
            border-radius: 4px;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }
        th, td {
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }
        th {
            background-color: #3498db;
            color: white;
        }
        tr:hover {
            background-color: #f5f5f5;
        }
        .timestamp {
            color: #7f8c8d;
            font-size: 14px;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Flight Analysis Report</h1>
        <div class="timestamp">Generated: {{ summary.timestamp }}</div>

        <h2>Flight Summary</h2>
        <div class="summary-grid">
            <div class="summary-card">
                <h3>Flight ID</h3>
                <div class="value">{{ summary.flight_id }}</div>
            </div>
            <div class="summary-card">
                <h3>Duration</h3>
                <div class="value">{{ "%.1f"|format(summary.duration_sec / 60) }}<span class="unit">min</span></div>
            </div>
            <div class="summary-card">
                <h3>Distance</h3>
                <div class="value">{{ "%.1f"|format(summary.distance_m) }}<span class="unit">m</span></div>
            </div>
            <div class="summary-card">
                <h3>Max Speed</h3>
                <div class="value">{{ "%.1f"|format(summary.max_speed_mps) }}<span class="unit">m/s</span></div>
            </div>
            <div class="summary-card">
                <h3>Max Altitude</h3>
                <div class="value">{{ "%.1f"|format(summary.max_altitude_m) }}<span class="unit">m</span></div>
            </div>
            <div class="summary-card">
                <h3>Battery Used</h3>
                <div class="value">{{ "%.0f"|format(summary.battery_start_pct - summary.battery_end_pct) }}<span class="unit">%</span></div>
            </div>
            <div class="summary-card">
                <h3>Energy Consumed</h3>
                <div class="value">{{ "%.0f"|format(summary.battery_consumed_mah) }}<span class="unit">mAh</span></div>
            </div>
            <div class="summary-card">
                <h3>Primary Mode</h3>
                <div class="value" style="font-size: 20px;">{{ summary.primary_flight_mode }}</div>
            </div>
        </div>

        {% if plots.flight_summary %}
        <div class="plot">
            <img src="{{ plots.flight_summary }}" alt="Flight Summary" style="width: 100%;">
        </div>
        {% endif %}

        <h2>SLAM vs EKF Comparison</h2>
        {% if analysis.slam_vs_ekf.available %}
        <div class="summary-grid">
            <div class="summary-card">
                <h3>Mean Error</h3>
                <div class="value">{{ "%.3f"|format(analysis.slam_vs_ekf.error_stats.mean) }}<span class="unit">m</span></div>
            </div>
            <div class="summary-card">
                <h3>Max Error</h3>
                <div class="value">{{ "%.3f"|format(analysis.slam_vs_ekf.error_stats.max) }}<span class="unit">m</span></div>
            </div>
            <div class="summary-card">
                <h3>RMS Error</h3>
                <div class="value">{{ "%.3f"|format(analysis.slam_vs_ekf.error_stats.rms) }}<span class="unit">m</span></div>
            </div>
        </div>

        {% if analysis.slam_vs_ekf.error_stats.rms < 0.5 %}
        <div class="success">
            <strong>Excellent:</strong> SLAM-EKF agreement is very good (RMS < 0.5m)
        </div>
        {% elif analysis.slam_vs_ekf.error_stats.rms < 1.0 %}
        <div class="warning">
            <strong>Good:</strong> SLAM-EKF agreement is acceptable (RMS < 1.0m)
        </div>
        {% else %}
        <div class="critical">
            <strong>Warning:</strong> Large SLAM-EKF disagreement detected (RMS ≥ 1.0m). Check VISO configuration.
        </div>
        {% endif %}

        <div class="plot">
            {{ plots.slam_vs_ekf_3d|safe }}
        </div>

        {% if plots.slam_vs_ekf_error %}
        <div class="plot">
            <img src="{{ plots.slam_vs_ekf_error }}" alt="Position Error" style="width: 100%;">
        </div>
        {% endif %}
        {% else %}
        <div class="warning">SLAM vs EKF data not available in this flight log.</div>
        {% endif %}

        <h2>Motor Analysis</h2>
        {% if analysis.motors.available %}

        {% if plots.motor_pwm %}
        <div class="plot">
            <img src="{{ plots.motor_pwm }}" alt="Motor PWM" style="width: 100%;">
        </div>
        {% endif %}

        {% if plots.motor_balance %}
        <div class="plot">
            <img src="{{ plots.motor_balance }}" alt="Motor Balance" style="width: 100%;">
        </div>
        {% endif %}

        {% if plots.hover_percentage %}
        <div class="plot">
            <img src="{{ plots.hover_percentage }}" alt="Hover Percentage" style="width: 100%;">
        </div>
        {% endif %}

        {% else %}
        <div class="warning">Motor data not available in this flight log.</div>
        {% endif %}

        <h2>Attitude Controller Performance</h2>
        {% if analysis.atc.available %}

        <div class="summary-grid">
            {% for axis in ['roll', 'pitch', 'yaw'] %}
            {% if axis in analysis.atc.tracking_error %}
            <div class="summary-card">
                <h3>{{ axis|title }} RMS Error</h3>
                <div class="value">{{ "%.4f"|format(analysis.atc.tracking_error[axis].rms) }}<span class="unit">rad/s</span></div>
            </div>
            {% endif %}
            {% endfor %}
        </div>

        {% if plots.atc_performance %}
        <div class="plot">
            <img src="{{ plots.atc_performance }}" alt="ATC Performance" style="width: 100%;">
        </div>
        {% endif %}

        {% else %}
        <div class="warning">ATC performance data not available in this flight log.</div>
        {% endif %}

        <h2>Power Analysis</h2>
        {% if analysis.power.available %}

        <div class="summary-grid">
            <div class="summary-card">
                <h3>Energy Consumed</h3>
                <div class="value">{{ "%.2f"|format(analysis.power.energy_wh) }}<span class="unit">Wh</span></div>
            </div>
            <div class="summary-card">
                <h3>Capacity Used</h3>
                <div class="value">{{ "%.0f"|format(analysis.power.energy_mah) }}<span class="unit">mAh</span></div>
            </div>
            {% if analysis.power.efficiency_wh_per_km > 0 %}
            <div class="summary-card">
                <h3>Efficiency</h3>
                <div class="value">{{ "%.1f"|format(analysis.power.efficiency_wh_per_km) }}<span class="unit">Wh/km</span></div>
            </div>
            {% endif %}
        </div>

        {% if plots.power_analysis %}
        <div class="plot">
            <img src="{{ plots.power_analysis }}" alt="Power Analysis" style="width: 100%;">
        </div>
        {% endif %}

        {% else %}
        <div class="warning">Power data not available in this flight log.</div>
        {% endif %}

        <h2>Vibration Analysis</h2>
        {% if analysis.vibration.available %}

        <div class="summary-grid">
            {% for axis in ['x', 'y', 'z'] %}
            <div class="summary-card">
                <h3>{{ axis|upper }} Axis RMS</h3>
                <div class="value">{{ "%.1f"|format(analysis.vibration.accel_rms[axis].value) }}<span class="unit">m/s²</span></div>
            </div>
            {% endfor %}
        </div>

        {% set has_warning = namespace(value=false) %}
        {% set has_critical = namespace(value=false) %}
        {% for axis in ['x', 'y', 'z'] %}
            {% if analysis.vibration.accel_rms[axis].critical %}
                {% set has_critical.value = true %}
            {% elif analysis.vibration.accel_rms[axis].warning %}
                {% set has_warning.value = true %}
            {% endif %}
        {% endfor %}

        {% if has_critical.value %}
        <div class="critical">
            <strong>Critical:</strong> Vibration levels exceed 100 m/s² threshold. Check propeller balance and motor mounting.
        </div>
        {% elif has_warning.value %}
        <div class="warning">
            <strong>Warning:</strong> Vibration levels exceed 60 m/s² threshold. Monitor propeller condition.
        </div>
        {% else %}
        <div class="success">
            <strong>Good:</strong> Vibration levels are within acceptable range.
        </div>
        {% endif %}

        {% if analysis.vibration.clipping_detected %}
        <div class="critical">
            <strong>Clipping Detected:</strong> IMU accelerometer saturation detected. Reduce vibration sources.
        </div>
        {% endif %}

        {% if plots.vibration_time %}
        <div class="plot">
            <img src="{{ plots.vibration_time }}" alt="Vibration Time Domain" style="width: 100%;">
        </div>
        {% endif %}

        {% if plots.vibration_fft %}
        <div class="plot">
            <img src="{{ plots.vibration_fft }}" alt="Vibration FFT" style="width: 100%;">
        </div>
        {% endif %}

        {% else %}
        <div class="warning">Vibration data not available in this flight log.</div>
        {% endif %}

        <h2>Topic Message Rates</h2>
        {% if analysis.topic_rates %}

        <table>
            <thead>
                <tr>
                    <th>Topic</th>
                    <th>Mean Rate (Hz)</th>
                    <th>Std Dev (Hz)</th>
                    <th>Total Messages</th>
                    <th>Gaps Detected</th>
                </tr>
            </thead>
            <tbody>
                {% for topic, stats in analysis.topic_rates.items() %}
                <tr>
                    <td>{{ topic }}</td>
                    <td>{{ "%.1f"|format(stats.mean_rate_hz) }}</td>
                    <td>{{ "%.1f"|format(stats.std_rate_hz) }}</td>
                    <td>{{ stats.total_messages }}</td>
                    <td>{{ stats.gap_count }}</td>
                </tr>
                {% endfor %}
            </tbody>
        </table>

        {% if plots.topic_rates %}
        <div class="plot">
            <img src="{{ plots.topic_rates }}" alt="Topic Rates" style="width: 100%;">
        </div>
        {% endif %}

        {% else %}
        <div class="warning">Topic rate data not available.</div>
        {% endif %}

        <hr style="margin: 40px 0; border: none; border-top: 2px solid #ecf0f1;">
        <p style="text-align: center; color: #7f8c8d;">
            Generated by SLAM Agent Flight Analysis Tool
        </p>
    </div>
</body>
</html>
"""

def generate_html_report(analysis_results: Dict[str, Any],
                        plots: Dict[str, str],
                        output_path: Path):
    """Generate self-contained HTML report"""
    template = Template(HTML_TEMPLATE)

    html_content = template.render(
        summary=analysis_results['summary'],
        analysis=analysis_results,
        plots=plots,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_content)

# ==============================================================================
# Main Entry Point
# ==============================================================================

def find_flight_bag(flight_id: str) -> Path:
    """Find bag file for given flight ID"""
    # Look for flight directory matching pattern
    flight_dirs = list(FLIGHT_DATA_ROOT.glob(f"{flight_id}_*"))

    if not flight_dirs:
        raise FileNotFoundError(f"No flight directory found for ID: {flight_id}")

    flight_dir = flight_dirs[0]
    bag_dir = flight_dir / "bag"

    # Search in bag/ subdirectory first, then flight_dir root
    search_dirs = [bag_dir, flight_dir] if bag_dir.exists() else [flight_dir]

    for search_dir in search_dirs:
        # ROS 2 mcap files (ros2 bag record creates a directory with metadata.yaml + .mcap)
        mcap_files = list(search_dir.glob("**/*.mcap"))
        if mcap_files:
            return mcap_files[0]

        # ROS 1 .bag files
        bag_files = list(search_dir.glob("**/*.bag"))
        if bag_files:
            return bag_files[0]

    raise FileNotFoundError(f"No bag files found in {flight_dir}")

def main():
    parser = argparse.ArgumentParser(description='Flight Data Analysis Dashboard Generator')
    parser.add_argument('flight_id', nargs='?', help='Flight ID (e.g., 001)')
    parser.add_argument('--bag', help='Path to bag file (overrides flight ID)')
    parser.add_argument('--json', action='store_true', help='Output JSON summary only')
    parser.add_argument('--output', help='Output directory (default: auto)')

    args = parser.parse_args()

    # Determine bag file path
    if args.bag:
        bag_path = Path(args.bag)
        flight_id = bag_path.stem
    elif args.flight_id:
        bag_path = find_flight_bag(args.flight_id)
        flight_id = args.flight_id
    else:
        parser.error("Either flight_id or --bag must be provided")

    if not bag_path.exists():
        print(f"Error: Bag file not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Analyzing flight: {flight_id}")
    print(f"Bag file: {bag_path}")

    # Extract data
    print("Extracting data from bag file...")
    extractor = FlightDataExtractor(str(bag_path))
    data = extractor.extract()

    # Analyze
    print("Running analysis...")
    analyzer = FlightAnalyzer(data, flight_id)
    analysis_results = analyzer.analyze_all()

    # Determine output directory
    if args.output:
        output_dir = Path(args.output)
    elif args.bag:
        output_dir = bag_path.parent / "reports"
    else:
        flight_dirs = list(FLIGHT_DATA_ROOT.glob(f"{flight_id}_*"))
        output_dir = flight_dirs[0] / "reports"

    output_dir.mkdir(parents=True, exist_ok=True)

    # Write JSON summary
    summary_path = output_dir / "summary.json"
    with open(summary_path, 'w') as f:
        json.dump({
            'summary': asdict(FlightSummary(**analysis_results['summary'])),
            'slam_vs_ekf': {
                'available': analysis_results['slam_vs_ekf']['available'],
                'error_stats': analysis_results['slam_vs_ekf'].get('error_stats', {}),
            },
            'motors': {
                'available': analysis_results['motors']['available'],
            },
            'atc': {
                'available': analysis_results['atc']['available'],
            },
            'power': {
                'available': analysis_results['power']['available'],
                'energy_wh': analysis_results['power'].get('energy_wh', 0),
                'efficiency_wh_per_km': analysis_results['power'].get('efficiency_wh_per_km', 0),
            },
            'vibration': {
                'available': analysis_results['vibration']['available'],
                'accel_rms': analysis_results['vibration'].get('accel_rms', {}),
            },
        }, f, indent=2)

    print(f"Summary written to: {summary_path}")

    if args.json:
        # JSON-only mode
        print(json.dumps({
            'success': True,
            'flight_id': flight_id,
            'summary_path': str(summary_path),
            'summary': analysis_results['summary'],
        }, indent=2))
    else:
        # Generate plots
        print("Generating plots...")
        plotter = FlightPlotter(analysis_results, data)
        plots = plotter.plot_all()

        # Generate HTML report
        print("Generating HTML report...")
        report_path = output_dir / "flight_report.html"
        generate_html_report(analysis_results, plots, report_path)

        print(f"\nReport generated successfully!")
        print(f"HTML Report: {report_path}")
        print(f"JSON Summary: {summary_path}")

        # Print summary stats
        summary = analysis_results['summary']
        print(f"\nFlight Summary:")
        print(f"  Duration: {summary['duration_sec']/60:.1f} minutes")
        print(f"  Distance: {summary['distance_m']:.1f} m")
        print(f"  Max Speed: {summary['max_speed_mps']:.1f} m/s")
        print(f"  Max Altitude: {summary['max_altitude_m']:.1f} m")
        print(f"  Battery Used: {summary['battery_start_pct']-summary['battery_end_pct']:.0f}%")
        print(f"  Energy: {summary['battery_consumed_mah']:.0f} mAh")

    return 0

if __name__ == '__main__':
    sys.exit(main())
