#!/usr/bin/env python3
"""
SLAM Bag File Analyzer
=======================

Analyzes ROS bag files from SLAM integration flights.
Extracts trajectory statistics, topic rates, data quality metrics, and generates plots.

Usage:
    # Analyze bag file:
    ./analyze_slam_bag.py flight_test.bag

    # Specify SLAM odometry topic:
    ./analyze_slam_bag.py flight_test.bag --slam-topic /fast_lio/odometry

    # Generate plots:
    ./analyze_slam_bag.py flight_test.bag --plot

    # Export detailed report:
    ./analyze_slam_bag.py flight_test.bag --report flight_report.txt

    # Analyze specific time range (seconds):
    ./analyze_slam_bag.py flight_test.bag --start 10 --end 60

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import sys
import argparse
import rosbag
from collections import defaultdict
import numpy as np
from datetime import datetime, timedelta

try:
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import PointCloud2, Imu
except ImportError:
    print("ERROR: ROS not found. Make sure ROS is installed and sourced.")
    sys.exit(1)

# Optional plotting
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class SLAMBagAnalyzer:
    """Analyzes SLAM bag files for trajectory and quality metrics."""
    
    def __init__(self, bag_file, slam_topic=None, start_time=None, end_time=None, 
                 generate_plots=False, report_file=None):
        """
        Initialize analyzer.
        
        Args:
            bag_file: Path to bag file
            slam_topic: SLAM odometry topic (auto-detect if None)
            start_time: Start time in seconds (from bag start)
            end_time: End time in seconds (from bag start)
            generate_plots: Generate matplotlib plots
            report_file: Export detailed report to file
        """
        self.bag_file = bag_file
        self.slam_topic = slam_topic
        self.start_time = start_time
        self.end_time = end_time
        self.generate_plots = generate_plots
        self.report_file = report_file
        
        self.bag = None
        self.bag_start = None
        self.bag_end = None
        self.bag_duration = None
        
        # Data storage
        self.trajectory = []  # [(time, x, y, z, qx, qy, qz, qw)]
        self.velocities = []  # [(time, vx, vy, vz)]
        self.topic_stats = defaultdict(lambda: {'count': 0, 'rates': []})
        self.data_gaps = defaultdict(list)  # [(start_time, end_time, duration)]
        
        # Analysis results
        self.total_distance = 0
        self.max_speed = 0
        self.avg_speed = 0
        self.drift_estimate = 0
        
        if generate_plots and not MATPLOTLIB_AVAILABLE:
            print("⚠ matplotlib not available. Install with: pip install matplotlib")
            self.generate_plots = False
    
    def run(self):
        """Run bag file analysis."""
        print(f"\n{'='*70}")
        print("SLAM Bag File Analyzer")
        print(f"{'='*70}\n")
        
        print(f"Bag file: {self.bag_file}")
        
        # Open bag
        if not self._open_bag():
            return False
        
        # Auto-detect SLAM topic if not specified
        if not self.slam_topic:
            self._detect_slam_topic()
        
        print(f"SLAM topic: {self.slam_topic or 'NOT DETECTED'}")
        print()
        
        if not self.slam_topic:
            print("✗ No SLAM topic found in bag file")
            print("  Specify manually with --slam-topic")
            return False
        
        # Extract data
        print("Extracting data from bag file...")
        self._extract_data()
        
        # Analyze
        print(f"\n{'='*70}")
        print("ANALYSIS RESULTS")
        print(f"{'='*70}\n")
        
        self._analyze_trajectory()
        self._analyze_topic_rates()
        self._analyze_data_gaps()
        self._analyze_sensor_quality()
        
        # Generate plots
        if self.generate_plots:
            self._generate_plots()
        
        # Export report
        if self.report_file:
            self._export_report()
        
        # Summary
        self._print_summary()
        
        self.bag.close()
        return True
    
    def _open_bag(self):
        """Open and validate bag file."""
        try:
            self.bag = rosbag.Bag(self.bag_file, 'r')
            
            # Get bag info
            info = self.bag.get_type_and_topic_info()
            
            self.bag_start = self.bag.get_start_time()
            self.bag_end = self.bag.get_end_time()
            self.bag_duration = self.bag_end - self.bag_start
            
            print(f"Bag duration: {self.bag_duration:.1f} seconds ({self.bag_duration/60:.1f} minutes)")
            print(f"Start time: {datetime.fromtimestamp(self.bag_start).strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"Topics: {len(info.topics)}")
            
            if self.start_time or self.end_time:
                start = self.start_time or 0
                end = self.end_time or self.bag_duration
                print(f"Analyzing time range: {start:.1f}s to {end:.1f}s")
            
            print()
            return True
            
        except Exception as e:
            print(f"✗ Failed to open bag file: {e}")
            return False
    
    def _detect_slam_topic(self):
        """Auto-detect SLAM odometry topic."""
        info = self.bag.get_type_and_topic_info()
        
        # Look for common SLAM topic patterns
        for topic, topic_info in info.topics.items():
            if 'Odometry' in topic_info.msg_type:
                if any(kw in topic for kw in ['slam', 'lio', 'fast', 'odometry', 'odom']):
                    # Exclude mavros odometry
                    if 'mavros' not in topic:
                        self.slam_topic = topic
                        break
    
    def _extract_data(self):
        """Extract data from bag file."""
        # Determine time range
        start_abs = self.bag_start + (self.start_time or 0)
        end_abs = self.bag_start + (self.end_time or self.bag_duration)
        
        # Track message counts and timing for all topics
        topic_last_time = defaultdict(lambda: None)
        
        for topic, msg, t in self.bag.read_messages():
            msg_time = t.to_sec()
            
            # Check time range
            if msg_time < start_abs or msg_time > end_abs:
                continue
            
            # Extract SLAM trajectory
            if topic == self.slam_topic:
                try:
                    # Handle different message types
                    if hasattr(msg, 'pose'):
                        if hasattr(msg.pose, 'pose'):
                            pose = msg.pose.pose  # PoseWithCovarianceStamped
                        else:
                            pose = msg.pose  # PoseStamped or Odometry
                    else:
                        continue
                    
                    x = pose.position.x
                    y = pose.position.y
                    z = pose.position.z
                    qx = pose.orientation.x
                    qy = pose.orientation.y
                    qz = pose.orientation.z
                    qw = pose.orientation.w
                    
                    self.trajectory.append((msg_time, x, y, z, qx, qy, qz, qw))
                    
                    # Extract velocity if available (Odometry messages)
                    if hasattr(msg, 'twist'):
                        if hasattr(msg.twist, 'twist'):
                            twist = msg.twist.twist
                        else:
                            twist = msg.twist
                        
                        vx = twist.linear.x
                        vy = twist.linear.y
                        vz = twist.linear.z
                        self.velocities.append((msg_time, vx, vy, vz))
                        
                except Exception as e:
                    pass
            
            # Track topic statistics
            self.topic_stats[topic]['count'] += 1
            
            # Detect gaps
            if topic_last_time[topic] is not None:
                gap = msg_time - topic_last_time[topic]
                if gap > 1.0:  # >1 second gap
                    self.data_gaps[topic].append((topic_last_time[topic], msg_time, gap))
            
            topic_last_time[topic] = msg_time
        
        print(f"✓ Extracted {len(self.trajectory)} SLAM poses")
        if self.velocities:
            print(f"✓ Extracted {len(self.velocities)} velocity measurements")
        print()
    
    def _analyze_trajectory(self):
        """Analyze SLAM trajectory."""
        print("1. Trajectory Analysis")
        print("-" * 70)
        
        if len(self.trajectory) < 2:
            print("⚠ Insufficient trajectory data")
            print()
            return
        
        # Calculate total distance
        distances = []
        speeds = []
        
        for i in range(1, len(self.trajectory)):
            t1, x1, y1, z1 = self.trajectory[i-1][:4]
            t2, x2, y2, z2 = self.trajectory[i][:4]
            
            dist = np.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
            distances.append(dist)
            self.total_distance += dist
            
            dt = t2 - t1
            if dt > 0:
                speed = dist / dt
                speeds.append(speed)
        
        self.max_speed = max(speeds) if speeds else 0
        self.avg_speed = np.mean(speeds) if speeds else 0
        
        # Calculate drift (straight-line distance vs traveled distance)
        start_pos = np.array(self.trajectory[0][1:4])
        end_pos = np.array(self.trajectory[-1][1:4])
        straight_line_dist = np.linalg.norm(end_pos - start_pos)
        
        self.drift_estimate = self.total_distance - straight_line_dist
        drift_pct = (self.drift_estimate / self.total_distance * 100) if self.total_distance > 0 else 0
        
        # Get position bounds
        positions = np.array([traj[1:4] for traj in self.trajectory])
        min_pos = positions.min(axis=0)
        max_pos = positions.max(axis=0)
        
        print(f"Distance traveled:     {self.total_distance:>8.2f} m")
        print(f"Straight-line distance:{straight_line_dist:>8.2f} m")
        print(f"Drift estimate:        {self.drift_estimate:>8.2f} m ({drift_pct:.1f}%)")
        print()
        print(f"Average speed:         {self.avg_speed:>8.2f} m/s")
        print(f"Maximum speed:         {self.max_speed:>8.2f} m/s")
        print()
        print(f"Position bounds:")
        print(f"  X: [{min_pos[0]:>7.2f}, {max_pos[0]:>7.2f}] m  (range: {max_pos[0]-min_pos[0]:.2f} m)")
        print(f"  Y: [{min_pos[1]:>7.2f}, {max_pos[1]:>7.2f}] m  (range: {max_pos[1]-min_pos[1]:.2f} m)")
        print(f"  Z: [{min_pos[2]:>7.2f}, {max_pos[2]:>7.2f}] m  (range: {max_pos[2]-min_pos[2]:.2f} m)")
        
        # Check for position jumps
        large_jumps = [(i, d) for i, d in enumerate(distances) if d > 5.0]
        if large_jumps:
            print()
            print(f"⚠ {len(large_jumps)} large position jumps detected (>5m):")
            for i, dist in large_jumps[:5]:  # Show first 5
                t = self.trajectory[i][0] - self.bag_start
                print(f"  Time {t:.1f}s: {dist:.2f}m jump")
            if len(large_jumps) > 5:
                print(f"  ... and {len(large_jumps)-5} more")
        
        print()
    
    def _analyze_topic_rates(self):
        """Analyze topic publication rates."""
        print("2. Topic Rate Analysis")
        print("-" * 70)
        
        # Calculate rates
        for topic in sorted(self.topic_stats.keys()):
            count = self.topic_stats[topic]['count']
            
            if count == 0:
                continue
            
            duration = (self.end_time or self.bag_duration) - (self.start_time or 0)
            rate = count / duration if duration > 0 else 0
            
            # Categorize topics
            if 'slam' in topic or 'lio' in topic or 'fast' in topic:
                category = "SLAM"
            elif 'vision' in topic:
                category = "Vision"
            elif 'mavros' in topic:
                category = "MAVROS"
            elif 'points' in topic or 'lidar' in topic:
                category = "Sensor"
            elif 'imu' in topic:
                category = "IMU"
            elif 'camera' in topic or 'image' in topic:
                category = "Camera"
            else:
                category = "Other"
            
            print(f"[{category:>8}] {topic:<45} {rate:>6.1f} Hz  ({count} msgs)")
        
        print()
    
    def _analyze_data_gaps(self):
        """Analyze data gaps and dropouts."""
        print("3. Data Gap Analysis")
        print("-" * 70)
        
        gaps_found = False
        
        for topic in sorted(self.data_gaps.keys()):
            gaps = self.data_gaps[topic]
            
            if not gaps:
                continue
            
            gaps_found = True
            
            total_gap_time = sum(g[2] for g in gaps)
            max_gap = max(gaps, key=lambda x: x[2])
            
            print(f"{topic}")
            print(f"  Gaps: {len(gaps)}")
            print(f"  Total gap time: {total_gap_time:.1f}s")
            print(f"  Largest gap: {max_gap[2]:.1f}s at t={max_gap[0]-self.bag_start:.1f}s")
        
        if not gaps_found:
            print("✓ No significant data gaps detected (>1s)")
        
        print()
    
    def _analyze_sensor_quality(self):
        """Analyze sensor data quality."""
        print("4. Sensor Quality Analysis")
        print("-" * 70)
        
        # Analyze velocity consistency
        if self.velocities:
            velocities = np.array([v[1:] for v in self.velocities])
            vel_magnitudes = np.linalg.norm(velocities, axis=1)
            
            print(f"Velocity statistics:")
            print(f"  Mean: {np.mean(vel_magnitudes):.2f} m/s")
            print(f"  Std:  {np.std(vel_magnitudes):.2f} m/s")
            print(f"  Max:  {np.max(vel_magnitudes):.2f} m/s")
            
            # Check for velocity outliers
            vel_threshold = np.mean(vel_magnitudes) + 3*np.std(vel_magnitudes)
            outliers = np.sum(vel_magnitudes > vel_threshold)
            if outliers > 0:
                print(f"  ⚠ {outliers} velocity outliers detected")
        
        # Analyze trajectory smoothness
        if len(self.trajectory) > 2:
            positions = np.array([traj[1:4] for traj in self.trajectory])
            
            # Calculate acceleration (change in velocity)
            velocities_calc = np.diff(positions, axis=0)
            accelerations = np.diff(velocities_calc, axis=0)
            accel_magnitudes = np.linalg.norm(accelerations, axis=1)
            
            print()
            print(f"Trajectory smoothness:")
            print(f"  Mean acceleration: {np.mean(accel_magnitudes):.3f} m/s²")
            print(f"  Max acceleration:  {np.max(accel_magnitudes):.3f} m/s²")
            
            # High acceleration indicates jitter/jumps
            if np.max(accel_magnitudes) > 10:
                print(f"  ⚠ High accelerations detected (possible jumps)")
        
        print()
    
    def _generate_plots(self):
        """Generate visualization plots."""
        print("5. Generating Plots")
        print("-" * 70)
        
        if not self.trajectory:
            print("⚠ No trajectory data to plot")
            return
        
        try:
            # Create figure with subplots
            fig = plt.figure(figsize=(16, 12))
            
            # 1. 3D Trajectory
            ax1 = fig.add_subplot(2, 3, 1, projection='3d')
            positions = np.array([traj[1:4] for traj in self.trajectory])
            ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=1)
            ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                       c='g', s=100, marker='o', label='Start')
            ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
                       c='r', s=100, marker='x', label='End')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('3D Trajectory')
            ax1.legend()
            
            # 2. 2D Trajectory (top view)
            ax2 = fig.add_subplot(2, 3, 2)
            ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=1)
            ax2.scatter(positions[0, 0], positions[0, 1], c='g', s=100, marker='o', label='Start')
            ax2.scatter(positions[-1, 0], positions[-1, 1], c='r', s=100, marker='x', label='End')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('2D Trajectory (Top View)')
            ax2.axis('equal')
            ax2.grid(True)
            ax2.legend()
            
            # 3. Altitude over time
            ax3 = fig.add_subplot(2, 3, 3)
            times = np.array([traj[0] - self.bag_start for traj in self.trajectory])
            altitudes = positions[:, 2]
            ax3.plot(times, altitudes, 'b-', linewidth=1)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Altitude (m)')
            ax3.set_title('Altitude Over Time')
            ax3.grid(True)
            
            # 4. Speed over time
            ax4 = fig.add_subplot(2, 3, 4)
            if self.velocities:
                vel_times = np.array([v[0] - self.bag_start for v in self.velocities])
                velocities = np.array([v[1:] for v in self.velocities])
                speeds = np.linalg.norm(velocities, axis=1)
                ax4.plot(vel_times, speeds, 'b-', linewidth=1)
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Speed (m/s)')
            ax4.set_title('Speed Over Time')
            ax4.grid(True)
            
            # 5. Position components over time
            ax5 = fig.add_subplot(2, 3, 5)
            ax5.plot(times, positions[:, 0], 'r-', label='X', linewidth=1)
            ax5.plot(times, positions[:, 1], 'g-', label='Y', linewidth=1)
            ax5.plot(times, positions[:, 2], 'b-', label='Z', linewidth=1)
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Position (m)')
            ax5.set_title('Position Components Over Time')
            ax5.legend()
            ax5.grid(True)
            
            # 6. Distance traveled over time
            ax6 = fig.add_subplot(2, 3, 6)
            cumulative_dist = np.cumsum([0] + [np.sqrt((positions[i, 0]-positions[i-1, 0])**2 + 
                                                        (positions[i, 1]-positions[i-1, 1])**2 + 
                                                        (positions[i, 2]-positions[i-1, 2])**2)
                                               for i in range(1, len(positions))])
            ax6.plot(times, cumulative_dist, 'b-', linewidth=1)
            ax6.set_xlabel('Time (s)')
            ax6.set_ylabel('Distance (m)')
            ax6.set_title('Cumulative Distance Traveled')
            ax6.grid(True)
            
            plt.tight_layout()
            
            # Save figure
            plot_filename = self.bag_file.replace('.bag', '_analysis.png')
            plt.savefig(plot_filename, dpi=150)
            print(f"✓ Plots saved to: {plot_filename}")
            
        except Exception as e:
            print(f"✗ Failed to generate plots: {e}")
        
        print()
    
    def _export_report(self):
        """Export detailed analysis report."""
        print("6. Exporting Report")
        print("-" * 70)
        
        try:
            with open(self.report_file, 'w') as f:
                f.write("="*70 + "\n")
                f.write("SLAM BAG FILE ANALYSIS REPORT\n")
                f.write("="*70 + "\n\n")
                
                f.write(f"Bag file: {self.bag_file}\n")
                f.write(f"Analysis date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Duration: {self.bag_duration:.1f}s ({self.bag_duration/60:.1f} min)\n")
                f.write(f"SLAM topic: {self.slam_topic}\n")
                f.write("\n")
                
                f.write("-"*70 + "\n")
                f.write("TRAJECTORY SUMMARY\n")
                f.write("-"*70 + "\n")
                f.write(f"Total distance:        {self.total_distance:.2f} m\n")
                f.write(f"Average speed:         {self.avg_speed:.2f} m/s\n")
                f.write(f"Maximum speed:         {self.max_speed:.2f} m/s\n")
                f.write(f"Drift estimate:        {self.drift_estimate:.2f} m\n")
                f.write(f"Number of poses:       {len(self.trajectory)}\n")
                f.write("\n")
                
                f.write("-"*70 + "\n")
                f.write("TOPIC RATES\n")
                f.write("-"*70 + "\n")
                for topic in sorted(self.topic_stats.keys()):
                    count = self.topic_stats[topic]['count']
                    duration = (self.end_time or self.bag_duration) - (self.start_time or 0)
                    rate = count / duration if duration > 0 else 0
                    f.write(f"{topic:<50} {rate:>6.1f} Hz\n")
                f.write("\n")
                
                if any(self.data_gaps.values()):
                    f.write("-"*70 + "\n")
                    f.write("DATA GAPS\n")
                    f.write("-"*70 + "\n")
                    for topic, gaps in sorted(self.data_gaps.items()):
                        if gaps:
                            f.write(f"{topic}: {len(gaps)} gaps\n")
                            for start, end, duration in gaps:
                                f.write(f"  {start-self.bag_start:.1f}s - {end-self.bag_start:.1f}s ({duration:.1f}s)\n")
                    f.write("\n")
            
            print(f"✓ Report saved to: {self.report_file}")
            
        except Exception as e:
            print(f"✗ Failed to export report: {e}")
        
        print()
    
    def _print_summary(self):
        """Print analysis summary."""
        print(f"{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")
        
        print(f"Bag Duration:     {self.bag_duration:.1f}s ({self.bag_duration/60:.1f} min)")
        print(f"Distance Traveled:{self.total_distance:>8.2f} m")
        print(f"Average Speed:    {self.avg_speed:>8.2f} m/s")
        print(f"Max Speed:        {self.max_speed:>8.2f} m/s")
        
        if self.drift_estimate > 0:
            drift_pct = (self.drift_estimate / self.total_distance * 100)
            if drift_pct < 5:
                status = "✓ GOOD"
            elif drift_pct < 15:
                status = "⚠ MODERATE"
            else:
                status = "✗ HIGH"
            print(f"Drift Estimate:   {self.drift_estimate:>8.2f} m ({drift_pct:.1f}%) {status}")
        
        print()
        
        # Data quality assessment
        gaps_total = sum(len(gaps) for gaps in self.data_gaps.values())
        
        if gaps_total == 0:
            print("✅ Data Quality: GOOD")
            print("  - No significant data gaps")
        elif gaps_total < 5:
            print("⚠️  Data Quality: MODERATE")
            print(f"  - {gaps_total} data gaps detected")
        else:
            print("❌ Data Quality: POOR")
            print(f"  - {gaps_total} data gaps detected")
        
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Analyze SLAM bag files for trajectory and quality metrics',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic analysis:
  %(prog)s flight_test.bag

  # Specify SLAM topic:
  %(prog)s flight_test.bag --slam-topic /fast_lio/odometry

  # Generate plots and report:
  %(prog)s flight_test.bag --plot --report flight_report.txt

  # Analyze specific time range:
  %(prog)s flight_test.bag --start 10 --end 60

  # Full analysis:
  %(prog)s flight_test.bag --slam-topic /odometry --plot --report report.txt
        """
    )
    
    parser.add_argument(
        'bag_file',
        help='Path to bag file'
    )
    
    parser.add_argument(
        '--slam-topic',
        help='SLAM odometry topic (auto-detect if not specified)'
    )
    
    parser.add_argument(
        '--start',
        type=float,
        help='Start time in seconds (from bag start)'
    )
    
    parser.add_argument(
        '--end',
        type=float,
        help='End time in seconds (from bag start)'
    )
    
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Generate matplotlib plots (requires matplotlib)'
    )
    
    parser.add_argument(
        '--report', '-r',
        metavar='FILE',
        help='Export detailed report to file'
    )
    
    args = parser.parse_args()
    
    try:
        analyzer = SLAMBagAnalyzer(
            bag_file=args.bag_file,
            slam_topic=args.slam_topic,
            start_time=args.start,
            end_time=args.end,
            generate_plots=args.plot,
            report_file=args.report
        )
        
        success = analyzer.run()
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

