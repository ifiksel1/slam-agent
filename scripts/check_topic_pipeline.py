#!/usr/bin/env python3
"""
Topic Pipeline Validator
=========================

Validates complete ROS topic pipeline for SLAM integration:
Sensor → SLAM → vision_to_mavros → MAVROS → Flight Controller

Checks topic existence, rates, message types, frame IDs, and latency.

Usage:
    # Auto-detect pipeline:
    ./check_topic_pipeline.py

    # Specify topics explicitly:
    ./check_topic_pipeline.py --sensor /ouster/points --slam /odometry/filtered \
        --vision /mavros/vision_pose/pose --mavros /mavros/local_position/pose

    # Check minimum rates:
    ./check_topic_pipeline.py --min-rate-sensor 5 --min-rate-slam 10

    # Monitor for duration:
    ./check_topic_pipeline.py --duration 10

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import sys
import argparse
from collections import defaultdict
import time
import os

# Check if ROS environment is sourced
if 'ROS_DISTRO' not in os.environ:
    print("ERROR: ROS environment not sourced.")
    print()
    print("Please source your ROS environment first:")
    print("  ROS1 (Noetic):  source /opt/ros/noetic/setup.bash")
    print("  ROS2 (Humble):  source /opt/ros/humble/setup.bash")
    print("  ROS2 (Foxy):    source /opt/ros/foxy/setup.bash")
    print()
    print("Then source your workspace:")
    print("  ROS1: source ~/catkin_ws/devel/setup.bash")
    print("  ROS2: source ~/ros2_ws/install/setup.bash")
    sys.exit(1)

try:
    import rospy
    from sensor_msgs.msg import PointCloud2, Imu, Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    import rostopic
except ImportError as e:
    print(f"ERROR: Failed to import ROS Python modules: {e}")
    print()
    print("ROS environment detected but Python modules not available.")
    print()
    print("Troubleshooting:")
    print("  1. Verify ROS is sourced: echo $ROS_DISTRO")
    print("  2. Check Python path: echo $PYTHONPATH")
    print("  3. Install ROS Python packages:")
    print("     ROS1: sudo apt install python3-rospy python3-sensor-msgs")
    print("     ROS2: sudo apt install python3-rclpy")
    print("  4. Verify ROS installation: rosversion -d (ROS1) or ros2 --version (ROS2)")
    sys.exit(1)


class TopicPipelineValidator:
    """Validates ROS topic pipeline for SLAM integration."""
    
    def __init__(self, sensor_topic=None, slam_topic=None, vision_topic=None,
                 mavros_topic=None, duration=5.0, min_rates=None):
        """
        Initialize validator.
        
        Args:
            sensor_topic: Sensor data topic (e.g., /ouster/points)
            slam_topic: SLAM odometry topic
            vision_topic: Vision pose topic to MAVROS
            mavros_topic: MAVROS local position topic
            duration: Monitoring duration (seconds)
            min_rates: Dict of minimum rates for each topic
        """
        self.sensor_topic = sensor_topic
        self.slam_topic = slam_topic
        self.vision_topic = vision_topic
        self.mavros_topic = mavros_topic
        self.duration = duration
        self.min_rates = min_rates or {}
        
        self.topic_data = defaultdict(lambda: {
            'count': 0,
            'first_time': None,
            'last_time': None,
            'frame_ids': set(),
            'timestamps': []
        })
        
        self.errors = []
        self.warnings = []
        
    def run(self):
        """Run pipeline validation."""
        print(f"\n{'='*70}")
        print("Topic Pipeline Validator")
        print(f"{'='*70}\n")
        
        rospy.init_node('topic_pipeline_validator', anonymous=True)
        
        # Auto-detect topics if not specified
        if not all([self.sensor_topic, self.slam_topic, self.vision_topic, self.mavros_topic]):
            self._auto_detect_topics()
        
        print("Monitoring pipeline:")
        print(f"  Sensor:  {self.sensor_topic or 'NOT DETECTED'}")
        print(f"  SLAM:    {self.slam_topic or 'NOT DETECTED'}")
        print(f"  Vision:  {self.vision_topic or 'NOT DETECTED'}")
        print(f"  MAVROS:  {self.mavros_topic or 'NOT DETECTED'}")
        print()
        
        # Subscribe to all topics
        subscribers = []
        
        if self.sensor_topic:
            sub = rospy.Subscriber(self.sensor_topic, rospy.AnyMsg,
                                 lambda msg: self._callback('sensor', msg),
                                 queue_size=10)
            subscribers.append(sub)
        
        if self.slam_topic:
            sub = rospy.Subscriber(self.slam_topic, rospy.AnyMsg,
                                 lambda msg: self._callback('slam', msg),
                                 queue_size=10)
            subscribers.append(sub)
        
        if self.vision_topic:
            sub = rospy.Subscriber(self.vision_topic, rospy.AnyMsg,
                                 lambda msg: self._callback('vision', msg),
                                 queue_size=10)
            subscribers.append(sub)
        
        if self.mavros_topic:
            sub = rospy.Subscriber(self.mavros_topic, rospy.AnyMsg,
                                 lambda msg: self._callback('mavros', msg),
                                 queue_size=10)
            subscribers.append(sub)
        
        # Monitor for duration
        print(f"Collecting data for {self.duration} seconds...")
        print()
        rospy.sleep(self.duration)
        
        # Unsubscribe
        for sub in subscribers:
            sub.unregister()
        
        # Analyze results
        print(f"{'='*70}")
        print("VALIDATION RESULTS")
        print(f"{'='*70}\n")
        
        self._check_topic_existence()
        self._check_topic_rates()
        self._check_frame_ids()
        self._check_timestamp_consistency()
        self._check_pipeline_latency()
        
        # Summary
        self._print_summary()
        
        return len(self.errors) == 0
    
    def _auto_detect_topics(self):
        """Auto-detect pipeline topics from available topics."""
        print("Auto-detecting pipeline topics...")
        
        try:
            # Get all topics
            topics = rospy.get_published_topics()
            topic_dict = {name: msg_type for name, msg_type in topics}
            
            # Detect sensor topic (point cloud or image)
            if not self.sensor_topic:
                for topic, msg_type in topic_dict.items():
                    if 'PointCloud2' in msg_type and 'points' in topic:
                        self.sensor_topic = topic
                        break
                    elif 'Image' in msg_type and 'image_raw' in topic:
                        self.sensor_topic = topic
                        break
            
            # Detect SLAM topic (odometry)
            if not self.slam_topic:
                for topic, msg_type in topic_dict.items():
                    if 'Odometry' in msg_type:
                        if any(kw in topic for kw in ['slam', 'lio', 'fast', 'odometry']):
                            self.slam_topic = topic
                            break
            
            # Detect vision topic
            if not self.vision_topic:
                for topic in topic_dict:
                    if 'mavros' in topic and 'vision_pose' in topic:
                        self.vision_topic = topic
                        break
            
            # Detect MAVROS topic
            if not self.mavros_topic:
                for topic in topic_dict:
                    if 'mavros' in topic and 'local_position' in topic and 'pose' in topic:
                        self.mavros_topic = topic
                        break
        
        except Exception as e:
            print(f"⚠ Auto-detection failed: {e}")
        
        print("✓ Auto-detection complete\n")
    
    def _callback(self, topic_type, msg):
        """Generic callback for topic monitoring."""
        now = rospy.Time.now()
        data = self.topic_data[topic_type]
        
        data['count'] += 1
        
        if data['first_time'] is None:
            data['first_time'] = now
        data['last_time'] = now
        
        # Try to extract frame_id and timestamp
        try:
            if hasattr(msg, 'header'):
                if hasattr(msg.header, 'frame_id'):
                    data['frame_ids'].add(msg.header.frame_id)
                if hasattr(msg.header, 'stamp'):
                    data['timestamps'].append(msg.header.stamp.to_sec())
            elif hasattr(msg, 'pose'):
                if hasattr(msg.pose, 'header'):
                    data['frame_ids'].add(msg.pose.header.frame_id)
                    data['timestamps'].append(msg.pose.header.stamp.to_sec())
        except:
            pass
    
    def _check_topic_existence(self):
        """Check that all pipeline topics exist and are publishing."""
        print("1. Topic Existence Check")
        print("-" * 70)
        
        topics = {
            'Sensor': ('sensor', self.sensor_topic),
            'SLAM': ('slam', self.slam_topic),
            'Vision': ('vision', self.vision_topic),
            'MAVROS': ('mavros', self.mavros_topic)
        }
        
        for name, (topic_type, topic_name) in topics.items():
            if not topic_name:
                print(f"⚠ {name:<15} NOT CONFIGURED")
                self.warnings.append(f"{name} topic not configured")
                continue
            
            data = self.topic_data[topic_type]
            
            if data['count'] == 0:
                print(f"✗ {name:<15} {topic_name:<40} NO DATA")
                self.errors.append(f"{name} topic not publishing: {topic_name}")
            else:
                print(f"✓ {name:<15} {topic_name:<40} OK")
        
        print()
    
    def _check_topic_rates(self):
        """Check topic publication rates."""
        print("2. Topic Rate Check")
        print("-" * 70)
        
        min_rates = {
            'sensor': self.min_rates.get('sensor', 5),
            'slam': self.min_rates.get('slam', 10),
            'vision': self.min_rates.get('vision', 10),
            'mavros': self.min_rates.get('mavros', 10)
        }
        
        for topic_type in ['sensor', 'slam', 'vision', 'mavros']:
            data = self.topic_data[topic_type]
            
            if data['count'] == 0:
                continue
            
            # Calculate rate
            if data['first_time'] and data['last_time']:
                duration = (data['last_time'] - data['first_time']).to_sec()
                if duration > 0:
                    rate = data['count'] / duration
                else:
                    rate = 0
            else:
                rate = 0
            
            min_rate = min_rates[topic_type]
            
            if rate < min_rate:
                status = "✗ TOO SLOW"
                self.errors.append(f"{topic_type} rate too slow: {rate:.1f} Hz (min: {min_rate})")
            elif rate < min_rate * 1.5:
                status = "⚠ LOW"
                self.warnings.append(f"{topic_type} rate low: {rate:.1f} Hz")
            else:
                status = "✓ OK"
            
            topic_name = {
                'sensor': self.sensor_topic,
                'slam': self.slam_topic,
                'vision': self.vision_topic,
                'mavros': self.mavros_topic
            }[topic_type]
            
            print(f"{status} {topic_type.capitalize():<10} {rate:>6.1f} Hz  (min: {min_rate} Hz)")
        
        print()
    
    def _check_frame_ids(self):
        """Check frame ID consistency."""
        print("3. Frame ID Check")
        print("-" * 70)
        
        for topic_type in ['sensor', 'slam', 'vision', 'mavros']:
            data = self.topic_data[topic_type]
            frame_ids = data['frame_ids']
            
            if not frame_ids:
                print(f"⚠ {topic_type.capitalize():<10} No frame_id detected")
                continue
            
            if len(frame_ids) == 1:
                frame_id = list(frame_ids)[0]
                print(f"✓ {topic_type.capitalize():<10} frame_id: {frame_id}")
            else:
                print(f"⚠ {topic_type.capitalize():<10} Multiple frame_ids: {frame_ids}")
                self.warnings.append(f"{topic_type} has inconsistent frame_ids")
        
        # Check expected frame IDs
        slam_frames = self.topic_data['slam']['frame_ids']
        vision_frames = self.topic_data['vision']['frame_ids']
        
        if slam_frames and vision_frames:
            if not slam_frames.intersection(vision_frames):
                print(f"\n⚠ SLAM and Vision frame_ids don't match:")
                print(f"  SLAM: {slam_frames}")
                print(f"  Vision: {vision_frames}")
                self.warnings.append("SLAM and Vision frame_id mismatch")
        
        print()
    
    def _check_timestamp_consistency(self):
        """Check timestamp consistency and monotonicity."""
        print("4. Timestamp Consistency Check")
        print("-" * 70)
        
        for topic_type in ['sensor', 'slam', 'vision', 'mavros']:
            data = self.topic_data[topic_type]
            timestamps = data['timestamps']
            
            if len(timestamps) < 2:
                continue
            
            # Check monotonicity
            non_monotonic = 0
            for i in range(1, len(timestamps)):
                if timestamps[i] < timestamps[i-1]:
                    non_monotonic += 1
            
            if non_monotonic > 0:
                pct = 100 * non_monotonic / len(timestamps)
                print(f"⚠ {topic_type.capitalize():<10} {non_monotonic} non-monotonic timestamps ({pct:.1f}%)")
                self.warnings.append(f"{topic_type} has non-monotonic timestamps")
            else:
                print(f"✓ {topic_type.capitalize():<10} Timestamps are monotonic")
            
            # Check for large gaps
            gaps = []
            for i in range(1, len(timestamps)):
                gap = timestamps[i] - timestamps[i-1]
                if gap > 1.0:  # >1 second gap
                    gaps.append(gap)
            
            if gaps:
                print(f"  ⚠ {len(gaps)} large gaps detected (max: {max(gaps):.2f}s)")
                self.warnings.append(f"{topic_type} has data gaps")
        
        print()
    
    def _check_pipeline_latency(self):
        """Check latency through the pipeline."""
        print("5. Pipeline Latency Check")
        print("-" * 70)
        
        # Compare timestamps between stages
        sensor_ts = self.topic_data['sensor']['timestamps']
        slam_ts = self.topic_data['slam']['timestamps']
        vision_ts = self.topic_data['vision']['timestamps']
        mavros_ts = self.topic_data['mavros']['timestamps']
        
        if len(sensor_ts) > 0 and len(slam_ts) > 0:
            # Approximate latency (using median timestamps)
            sensor_median = sorted(sensor_ts)[len(sensor_ts)//2]
            slam_median = sorted(slam_ts)[len(slam_ts)//2]
            
            latency = slam_median - sensor_median
            
            if abs(latency) < 0.5:
                print(f"✓ Sensor → SLAM latency:  {latency*1000:>6.1f} ms")
            else:
                print(f"⚠ Sensor → SLAM latency:  {latency*1000:>6.1f} ms (high)")
                self.warnings.append(f"High sensor-SLAM latency: {latency:.3f}s")
        
        if len(slam_ts) > 0 and len(vision_ts) > 0:
            slam_median = sorted(slam_ts)[len(slam_ts)//2]
            vision_median = sorted(vision_ts)[len(vision_ts)//2]
            
            latency = vision_median - slam_median
            
            if abs(latency) < 0.1:
                print(f"✓ SLAM → Vision latency:  {latency*1000:>6.1f} ms")
            else:
                print(f"⚠ SLAM → Vision latency:  {latency*1000:>6.1f} ms (high)")
                self.warnings.append(f"High SLAM-Vision latency: {latency:.3f}s")
        
        print()
    
    def _print_summary(self):
        """Print validation summary."""
        print(f"{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")
        
        print(f"Pipeline Status:")
        for topic_type in ['sensor', 'slam', 'vision', 'mavros']:
            count = self.topic_data[topic_type]['count']
            if count > 0:
                print(f"  ✓ {topic_type.capitalize()}: {count} messages")
            else:
                print(f"  ✗ {topic_type.capitalize()}: NO DATA")
        
        print()
        print(f"Errors: {len(self.errors)}")
        print(f"Warnings: {len(self.warnings)}")
        print()
        
        if self.errors:
            print("❌ ERRORS:")
            for error in self.errors:
                print(f"  • {error}")
            print()
        
        if self.warnings:
            print("⚠️  WARNINGS:")
            for warning in self.warnings:
                print(f"  • {warning}")
            print()
        
        if not self.errors and not self.warnings:
            print("✅ Pipeline is healthy!")
        elif not self.errors:
            print("⚠️  Pipeline has warnings but no critical errors")
        else:
            print("❌ Pipeline has errors that need to be fixed")
            print("\nTroubleshooting:")
            print("  1. Check that all nodes are running: rosnode list")
            print("  2. Verify topic names: rostopic list")
            print("  3. Check topic rates: rostopic hz <topic_name>")
            print("  4. Verify frame_ids match between SLAM and vision_to_mavros")
        
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Validate ROS topic pipeline for SLAM integration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect pipeline:
  %(prog)s

  # Specify topics:
  %(prog)s --sensor /ouster/points --slam /odometry/filtered

  # Set minimum rates:
  %(prog)s --min-rate-sensor 10 --min-rate-slam 20

  # Monitor for longer:
  %(prog)s --duration 10
        """
    )
    
    parser.add_argument(
        '--sensor',
        help='Sensor data topic (e.g., /ouster/points)'
    )
    
    parser.add_argument(
        '--slam',
        help='SLAM odometry topic'
    )
    
    parser.add_argument(
        '--vision',
        help='Vision pose topic (e.g., /mavros/vision_pose/pose)'
    )
    
    parser.add_argument(
        '--mavros',
        help='MAVROS local position topic'
    )
    
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=5.0,
        help='Monitoring duration in seconds (default: 5.0)'
    )
    
    parser.add_argument(
        '--min-rate-sensor',
        type=float,
        default=5,
        help='Minimum sensor rate (Hz, default: 5)'
    )
    
    parser.add_argument(
        '--min-rate-slam',
        type=float,
        default=10,
        help='Minimum SLAM rate (Hz, default: 10)'
    )
    
    parser.add_argument(
        '--min-rate-vision',
        type=float,
        default=10,
        help='Minimum vision rate (Hz, default: 10)'
    )
    
    parser.add_argument(
        '--min-rate-mavros',
        type=float,
        default=10,
        help='Minimum MAVROS rate (Hz, default: 10)'
    )
    
    args = parser.parse_args()
    
    min_rates = {
        'sensor': args.min_rate_sensor,
        'slam': args.min_rate_slam,
        'vision': args.min_rate_vision,
        'mavros': args.min_rate_mavros
    }
    
    try:
        validator = TopicPipelineValidator(
            sensor_topic=args.sensor,
            slam_topic=args.slam,
            vision_topic=args.vision,
            mavros_topic=args.mavros,
            duration=args.duration,
            min_rates=min_rates
        )
        
        success = validator.run()
        sys.exit(0 if success else 1)
        
    except rospy.ROSInterruptException:
        print("\nShutdown requested by ROS")
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

