#!/usr/bin/env python3
"""
Sensor Time Synchronization Checker
====================================

Monitors multiple ROS topics and calculates time offsets between sensor timestamps.
Useful for diagnosing time synchronization issues in SLAM systems.

Common Sensor Topics:
    /mavros/imu/data        - Flight controller IMU
    /ouster/points          - Ouster LiDAR point cloud
    /ouster/imu             - Ouster LiDAR built-in IMU
    /velodyne_points        - Velodyne LiDAR point cloud
    /livox/lidar            - Livox LiDAR point cloud
    /livox/imu              - Livox LiDAR built-in IMU
    /camera/image_raw       - Camera images
    /camera/imu             - Camera IMU (e.g., RealSense D435i)

IMPORTANT: Check synchronization between DIFFERENT sensors, not within the same sensor!
    ✓ /ouster/points vs /mavros/imu/data    - LiDAR vs FC (different sensors)
    ✓ /camera/image_raw vs /mavros/imu/data - Camera vs FC (different sensors)
    ✗ /ouster/points vs /ouster/imu         - Same sensor (hardware-synced already!)
    ✗ /camera/image_raw vs /camera/imu      - Same sensor (hardware-synced already!)

Usage:
    # LiDAR vs Flight Controller IMU (if using FC IMU for SLAM):
    ./check_sensor_time_sync.py /ouster/points /mavros/imu/data

    # Camera vs Flight Controller IMU (for VIO):
    ./check_sensor_time_sync.py /camera/image_raw /mavros/imu/data

    # LiDAR vs Camera (for multi-sensor SLAM like LVI-SAM):
    ./check_sensor_time_sync.py /ouster/points /camera/image_raw

    # Check multiple sensors:
    ./check_sensor_time_sync.py /ouster/points /camera/image_raw /mavros/imu/data

    # With logging:
    ./check_sensor_time_sync.py /ouster/points /ouster/imu --log sync_log.csv

    # Specify duration:
    ./check_sensor_time_sync.py /ouster/points /mavros/imu/data --duration 10

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import rospy
import sys
import argparse
from collections import defaultdict
import numpy as np
from datetime import datetime
import csv

# Import common ROS message types
try:
    from sensor_msgs.msg import PointCloud2, Imu, Image, CompressedImage, LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, TwistStamped
except ImportError:
    print("ERROR: ROS sensor_msgs not found. Make sure ROS is installed and sourced.")
    sys.exit(1)


class SensorTimeSyncChecker:
    """Monitors multiple sensor topics and calculates time offsets."""
    
    def __init__(self, topics, duration=5.0, log_file=None):
        """
        Initialize the time sync checker.
        
        Args:
            topics: List of topic names to monitor
            duration: How long to collect data (seconds)
            log_file: Optional CSV file to log detailed data
        """
        self.topics = topics
        self.duration = duration
        self.log_file = log_file
        
        # Storage for timestamps: {topic_name: [timestamps]}
        self.timestamps = defaultdict(list)
        
        # Track message counts
        self.msg_counts = defaultdict(int)
        
        # For CSV logging
        self.csv_writer = None
        self.csv_file = None
        
        # Start time
        self.start_time = None
        
        print(f"\n{'='*70}")
        print(f"Sensor Time Synchronization Checker")
        print(f"{'='*70}")
        print(f"Monitoring {len(topics)} topics for {duration} seconds...")
        print(f"Topics: {', '.join(topics)}")
        if log_file:
            print(f"Logging to: {log_file}")
        print(f"{'='*70}\n")
        
    def generic_callback(self, msg, topic_name):
        """Generic callback that extracts timestamp from any message with a header."""
        if not self.start_time:
            self.start_time = rospy.Time.now()
            
        # Check if we've exceeded duration
        if (rospy.Time.now() - self.start_time).to_sec() > self.duration:
            return
            
        # Extract timestamp
        try:
            if hasattr(msg, 'header'):
                timestamp = msg.header.stamp.to_sec()
            elif hasattr(msg, 'stamp'):
                timestamp = msg.stamp.to_sec()
            else:
                rospy.logwarn(f"Topic {topic_name} has no timestamp field")
                return
                
            self.timestamps[topic_name].append(timestamp)
            self.msg_counts[topic_name] += 1
            
            # Log to CSV if enabled
            if self.csv_writer:
                self.csv_writer.writerow([
                    topic_name,
                    timestamp,
                    rospy.Time.now().to_sec(),
                    self.msg_counts[topic_name]
                ])
                
        except Exception as e:
            rospy.logwarn(f"Error extracting timestamp from {topic_name}: {e}")
    
    def get_message_class(self, topic_name):
        """Determine the message type for a topic."""
        try:
            # Wait for topic to appear
            topic_type = rospy.wait_for_message(topic_name, rospy.AnyMsg, timeout=2.0)
            
            # Get the actual message class
            topic_info = rospy.get_published_topics()
            for name, msg_type in topic_info:
                if name == topic_name:
                    # Parse the message type string
                    if 'PointCloud2' in msg_type:
                        return PointCloud2
                    elif 'Imu' in msg_type:
                        return Imu
                    elif 'Image' in msg_type and 'Compressed' not in msg_type:
                        return Image
                    elif 'CompressedImage' in msg_type:
                        return CompressedImage
                    elif 'Odometry' in msg_type:
                        return Odometry
                    elif 'PoseStamped' in msg_type:
                        return PoseStamped
                    elif 'TwistStamped' in msg_type:
                        return TwistStamped
                    elif 'LaserScan' in msg_type:
                        return LaserScan
                    else:
                        # Default to generic message
                        return rospy.AnyMsg
            
            # If not found, try common types
            return rospy.AnyMsg
            
        except rospy.ROSException:
            rospy.logwarn(f"Timeout waiting for topic {topic_name}. Is it publishing?")
            return None
    
    def run(self):
        """Start monitoring topics and collect data."""
        rospy.init_node('sensor_time_sync_checker', anonymous=True)
        
        # Setup CSV logging if requested
        if self.log_file:
            self.csv_file = open(self.log_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Topic', 'Sensor_Timestamp', 'Receive_Time', 'Msg_Count'])
        
        # Subscribe to all topics
        subscribers = []
        for topic in self.topics:
            msg_class = self.get_message_class(topic)
            if msg_class:
                sub = rospy.Subscriber(
                    topic,
                    msg_class,
                    lambda msg, t=topic: self.generic_callback(msg, t),
                    queue_size=100
                )
                subscribers.append(sub)
                print(f"✓ Subscribed to {topic}")
            else:
                print(f"✗ Could not subscribe to {topic} (timeout or not publishing)")
        
        if not subscribers:
            print("\nERROR: Could not subscribe to any topics!")
            print("Make sure the topics are publishing and ROS is running.")
            return
        
        print(f"\nCollecting data for {self.duration} seconds...")
        print("(Press Ctrl+C to stop early)\n")
        
        # Collect data
        try:
            rospy.sleep(self.duration)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        
        # Cleanup
        for sub in subscribers:
            sub.unregister()
        
        if self.csv_file:
            self.csv_file.close()
            print(f"\n✓ Detailed log saved to {self.log_file}")
        
        # Analyze and display results
        self.analyze_results()
    
    def analyze_results(self):
        """Analyze collected timestamps and display results."""
        print(f"\n{'='*70}")
        print(f"RESULTS")
        print(f"{'='*70}\n")
        
        # Check if we got any data
        if not any(self.timestamps.values()):
            print("ERROR: No messages received on any topic!")
            print("\nTroubleshooting:")
            print("  1. Check that topics are publishing: rostopic list")
            print("  2. Check message rate: rostopic hz <topic_name>")
            print("  3. Verify topic names are correct")
            return
        
        # Display message counts
        print("Message Counts:")
        print(f"{'Topic':<50} {'Count':>10} {'Rate (Hz)':>10}")
        print("-" * 70)
        for topic in self.topics:
            count = self.msg_counts.get(topic, 0)
            rate = count / self.duration if self.duration > 0 else 0
            status = "✓" if count > 0 else "✗"
            print(f"{status} {topic:<48} {count:>10} {rate:>10.1f}")
        
        # Calculate offsets between all pairs
        print(f"\n{'='*70}")
        print("Time Offsets Between Sensors:")
        print(f"{'='*70}\n")
        
        topics_with_data = [t for t in self.topics if self.timestamps[t]]
        
        if len(topics_with_data) < 2:
            print("⚠ Need at least 2 topics with data to calculate offsets")
            return
        
        # For each pair of topics
        for i, topic1 in enumerate(topics_with_data):
            for topic2 in topics_with_data[i+1:]:
                self.calculate_pairwise_offset(topic1, topic2)
        
        # Overall assessment
        print(f"\n{'='*70}")
        print("Assessment:")
        print(f"{'='*70}\n")
        
        max_offset = 0
        for i, topic1 in enumerate(topics_with_data):
            for topic2 in topics_with_data[i+1:]:
                ts1 = np.array(self.timestamps[topic1])
                ts2 = np.array(self.timestamps[topic2])
                
                # Find closest timestamps
                offsets = []
                for t1 in ts1[:min(100, len(ts1))]:  # Sample first 100
                    closest_idx = np.argmin(np.abs(ts2 - t1))
                    offset = abs(ts2[closest_idx] - t1)
                    offsets.append(offset)
                
                if offsets:
                    mean_offset = np.mean(offsets) * 1000  # Convert to ms
                    max_offset = max(max_offset, mean_offset)
        
        print(f"Maximum mean offset: {max_offset:.2f} ms\n")
        
        if max_offset < 10:
            print("✓ GOOD: All sensors well synchronized (<10ms)")
            print("  → Should work well for SLAM")
        elif max_offset < 50:
            print("⚠ WARNING: Moderate offset (10-50ms)")
            print("  → May cause minor issues in fast motion")
            print("  → Consider improving synchronization for best performance")
        else:
            print("✗ POOR: Large offset (>50ms)")
            print("  → Will cause SLAM errors!")
            print("  → Solutions:")
            print("     1. Use hardware-synchronized sensors")
            print("     2. Check sensor driver timestamp settings")
            print("     3. Use time calibration (e.g., Kalibr for camera-IMU)")
            print("     4. Check system clock synchronization (NTP)")
        
        print()
    
    def calculate_pairwise_offset(self, topic1, topic2):
        """Calculate offset between two topics."""
        ts1 = np.array(self.timestamps[topic1])
        ts2 = np.array(self.timestamps[topic2])
        
        # Find overlapping time range
        t1_start, t1_end = ts1[0], ts1[-1]
        t2_start, t2_end = ts2[0], ts2[-1]
        
        overlap_start = max(t1_start, t2_start)
        overlap_end = min(t1_end, t2_end)
        
        if overlap_start >= overlap_end:
            print(f"⚠ {topic1} <-> {topic2}")
            print(f"   No overlapping timestamps!")
            print()
            return
        
        # Calculate offsets by finding nearest timestamps
        offsets = []
        for t1 in ts1:
            if overlap_start <= t1 <= overlap_end:
                # Find closest timestamp in topic2
                closest_idx = np.argmin(np.abs(ts2 - t1))
                offset = ts2[closest_idx] - t1
                offsets.append(offset)
        
        if not offsets:
            print(f"⚠ {topic1} <-> {topic2}")
            print(f"   Could not calculate offsets")
            print()
            return
        
        offsets = np.array(offsets) * 1000  # Convert to milliseconds
        
        # Calculate statistics
        mean_offset = np.mean(offsets)
        std_offset = np.std(offsets)
        min_offset = np.min(offsets)
        max_offset = np.max(offsets)
        abs_mean = np.mean(np.abs(offsets))
        
        # Display results
        print(f"Topic 1: {topic1}")
        print(f"Topic 2: {topic2}")
        print(f"-" * 70)
        print(f"  Mean offset:     {mean_offset:>8.2f} ms")
        print(f"  Std deviation:   {std_offset:>8.2f} ms")
        print(f"  Min offset:      {min_offset:>8.2f} ms")
        print(f"  Max offset:      {max_offset:>8.2f} ms")
        print(f"  Abs mean offset: {abs_mean:>8.2f} ms")
        
        # Status
        if abs_mean < 10:
            status = "✓ GOOD"
            color = ""
        elif abs_mean < 50:
            status = "⚠ WARNING"
            color = ""
        else:
            status = "✗ POOR"
            color = ""
        
        print(f"  Status: {color}{status}")
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Check time synchronization between ROS sensor topics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # LiDAR vs Flight Controller IMU (if using FC IMU for SLAM):
  %(prog)s /ouster/points /mavros/imu/data

  # Camera vs Flight Controller IMU (for VIO):
  %(prog)s /camera/image_raw /mavros/imu/data

  # LiDAR vs Camera (for multi-sensor SLAM like LVI-SAM):
  %(prog)s /ouster/points /camera/image_raw

  # Check multiple sensors at once:
  %(prog)s /ouster/points /camera/image_raw /mavros/imu/data

  # With logging and custom duration:
  %(prog)s /ouster/points /mavros/imu/data --log sync_log.csv --duration 10
  
  # Multi-LiDAR systems:
  %(prog)s /ouster/points /velodyne_points
        """
    )
    
    parser.add_argument(
        'topics',
        nargs='+',
        help='ROS topics to monitor (at least 2)'
    )
    
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=5.0,
        help='Data collection duration in seconds (default: 5.0)'
    )
    
    parser.add_argument(
        '--log', '-l',
        type=str,
        default=None,
        help='Log detailed data to CSV file'
    )
    
    args = parser.parse_args()
    
    # Validate inputs
    if len(args.topics) < 2:
        print("ERROR: Need at least 2 topics to compare")
        print("Usage: ./check_sensor_time_sync.py <topic1> <topic2> [topic3 ...]")
        sys.exit(1)
    
    if args.duration <= 0:
        print("ERROR: Duration must be positive")
        sys.exit(1)
    
    try:
        # Create and run checker
        checker = SensorTimeSyncChecker(
            topics=args.topics,
            duration=args.duration,
            log_file=args.log
        )
        checker.run()
        
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

