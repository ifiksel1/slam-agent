#!/usr/bin/env python3
"""
TF Tree Validator
=================

Validates ROS TF tree for SLAM integration systems.
Checks for missing transforms, incorrect rates, circular dependencies, and common naming issues.

Usage:
    # Basic check (auto-detects required frames from topics):
    ./check_tf_tree.py

    # Specify required frames explicitly:
    ./check_tf_tree.py --frames map odom base_link os_sensor camera_link

    # Check specific source->target transform:
    ./check_tf_tree.py --source map --target base_link

    # Detailed output:
    ./check_tf_tree.py --verbose

    # Generate visualization:
    ./check_tf_tree.py --visualize

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import rospy
import tf2_ros
import sys
import argparse
from collections import defaultdict
import time
import subprocess
import os

try:
    import tf2_msgs.msg
    from geometry_msgs.msg import TransformStamped
except ImportError:
    print("ERROR: ROS tf2 not found. Make sure ROS is installed and sourced.")
    sys.exit(1)


class TFTreeValidator:
    """Validates TF tree for SLAM integration."""
    
    def __init__(self, required_frames=None, verbose=False, visualize=False):
        """
        Initialize validator.
        
        Args:
            required_frames: List of frame names that must exist
            verbose: Print detailed information
            visualize: Generate visual TF tree diagram
        """
        self.required_frames = required_frames or []
        self.verbose = verbose
        self.visualize = visualize
        
        # Common frame patterns for auto-detection
        self.common_frames = [
            'map', 'odom', 'base_link', 'base_footprint',
            # LiDAR frames
            'os_sensor', 'os1_sensor', 'os1_lidar', 'ouster',
            'velodyne', 'velodyne_base_link',
            'livox_frame', 'livox',
            # Camera frames
            'camera_link', 'camera_optical_frame',
            'realsense_link', 'd435i_link',
            'zed_left_camera_frame',
            # IMU frames
            'imu_link', 'imu_frame',
        ]
        
        self.tf_buffer = None
        self.tf_listener = None
        self.all_frames = set()
        self.transform_rates = defaultdict(list)
        self.errors = []
        self.warnings = []
        
    def run(self):
        """Run TF tree validation."""
        print(f"\n{'='*70}")
        print("TF Tree Validator")
        print(f"{'='*70}\n")
        
        rospy.init_node('tf_tree_validator', anonymous=True)
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        print("Collecting TF data for 3 seconds...")
        rospy.sleep(3.0)
        
        # Get all available frames
        self.all_frames = set(self.tf_buffer.all_frames_as_string().split('\n'))
        self.all_frames.discard('')  # Remove empty string
        
        print(f"Found {len(self.all_frames)} frames in TF tree\n")
        
        if self.verbose:
            print("All frames:")
            for frame in sorted(self.all_frames):
                print(f"  - {frame}")
            print()
        
        # Auto-detect required frames if not specified
        if not self.required_frames:
            self.required_frames = self._detect_required_frames()
        
        # Run checks
        print(f"{'='*70}")
        print("VALIDATION RESULTS")
        print(f"{'='*70}\n")
        
        self._check_required_frames()
        self._check_frame_connectivity()
        self._check_transform_rates()
        self._check_common_issues()
        self._check_circular_dependencies()
        
        # Generate visualization if requested
        if self.visualize:
            self._generate_visualization()
        
        # Summary
        self._print_summary()
        
        return len(self.errors) == 0
    
    def _detect_required_frames(self):
        """Auto-detect which frames should exist based on common patterns."""
        detected = []
        
        # Always need these core frames
        core_frames = ['map', 'odom', 'base_link']
        for frame in core_frames:
            if frame in self.all_frames:
                detected.append(frame)
        
        # Check for sensor frames
        for frame in self.all_frames:
            for common in self.common_frames:
                if common in frame.lower():
                    detected.append(frame)
                    break
        
        return list(set(detected))
    
    def _check_required_frames(self):
        """Check that all required frames exist."""
        print("1. Required Frames Check")
        print("-" * 70)
        
        if not self.required_frames:
            print("⚠ WARNING: No required frames specified")
            print("  Using auto-detected frames from TF tree\n")
            self.warnings.append("No required frames specified")
            return
        
        missing_frames = []
        for frame in self.required_frames:
            if frame in self.all_frames:
                print(f"✓ {frame:<30} FOUND")
            else:
                print(f"✗ {frame:<30} MISSING")
                missing_frames.append(frame)
                self.errors.append(f"Missing required frame: {frame}")
        
        if missing_frames:
            print(f"\n⚠ Missing frames: {', '.join(missing_frames)}")
            print("  Check your URDF or static_transform_publisher nodes")
        
        print()
    
    def _check_frame_connectivity(self):
        """Check that frames are properly connected."""
        print("2. Frame Connectivity Check")
        print("-" * 70)
        
        # Check critical connections
        critical_pairs = [
            ('map', 'odom'),
            ('odom', 'base_link'),
        ]
        
        for source, target in critical_pairs:
            if source not in self.all_frames or target not in self.all_frames:
                continue
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    source, target, rospy.Time(0), rospy.Duration(1.0)
                )
                age = (rospy.Time.now() - transform.header.stamp).to_sec()
                
                if age < 1.0:
                    print(f"✓ {source} → {target:<20} Connected (age: {age:.3f}s)")
                else:
                    print(f"⚠ {source} → {target:<20} Old data (age: {age:.3f}s)")
                    self.warnings.append(f"Transform {source}→{target} has old data")
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print(f"✗ {source} → {target:<20} NOT CONNECTED")
                self.errors.append(f"Cannot transform {source}→{target}: {str(e)}")
        
        # Check sensor frames connected to base_link
        if 'base_link' in self.all_frames:
            sensor_frames = [f for f in self.all_frames 
                           if any(s in f for s in ['sensor', 'lidar', 'camera', 'imu'])]
            
            for sensor_frame in sensor_frames:
                try:
                    self.tf_buffer.lookup_transform(
                        'base_link', sensor_frame, rospy.Time(0), rospy.Duration(1.0)
                    )
                    print(f"✓ base_link → {sensor_frame:<20} Connected")
                except:
                    print(f"✗ base_link → {sensor_frame:<20} NOT CONNECTED")
                    self.errors.append(f"Sensor frame {sensor_frame} not connected to base_link")
        
        print()
    
    def _check_transform_rates(self):
        """Check transform publication rates."""
        print("3. Transform Rate Check")
        print("-" * 70)
        
        # Monitor transform rates for 2 seconds
        print("Monitoring transform rates for 2 seconds...")
        
        rate_counts = defaultdict(int)
        start_time = time.time()
        
        def tf_callback(msg):
            for transform in msg.transforms:
                key = f"{transform.header.frame_id}→{transform.child_frame_id}"
                rate_counts[key] += 1
        
        sub = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, tf_callback, queue_size=100)
        rospy.sleep(2.0)
        sub.unregister()
        
        duration = time.time() - start_time
        
        if not rate_counts:
            print("⚠ WARNING: No TF messages received on /tf topic")
            print("  Transforms may be static (check /tf_static)")
            self.warnings.append("No dynamic transforms detected")
            print()
            return
        
        # Check rates
        for transform, count in sorted(rate_counts.items()):
            rate = count / duration
            
            if rate < 1.0:
                status = "⚠ SLOW"
                self.warnings.append(f"{transform} rate too slow: {rate:.1f} Hz")
            elif rate < 10.0:
                status = "⚠ LOW"
                self.warnings.append(f"{transform} rate low: {rate:.1f} Hz")
            elif rate > 200.0:
                status = "⚠ HIGH"
                self.warnings.append(f"{transform} rate very high: {rate:.1f} Hz")
            else:
                status = "✓ OK"
            
            print(f"{status} {transform:<40} {rate:>6.1f} Hz")
        
        print()
    
    def _check_common_issues(self):
        """Check for common TF tree issues."""
        print("4. Common Issues Check")
        print("-" * 70)
        
        issues_found = False
        
        # Check for similar frame names (typos)
        frame_list = sorted(self.all_frames)
        for i, frame1 in enumerate(frame_list):
            for frame2 in frame_list[i+1:]:
                # Simple similarity check
                if self._similar_strings(frame1, frame2):
                    print(f"⚠ Similar frame names (possible typo?):")
                    print(f"  - {frame1}")
                    print(f"  - {frame2}")
                    self.warnings.append(f"Similar frame names: {frame1} vs {frame2}")
                    issues_found = True
        
        # Check for frames with underscores vs no underscores
        frame_bases = set()
        for frame in self.all_frames:
            base = frame.replace('_', '')
            if base in frame_bases:
                print(f"⚠ Frame naming inconsistency: {frame}")
                self.warnings.append(f"Inconsistent naming: {frame}")
                issues_found = True
            frame_bases.add(base)
        
        # Check for common namespace issues
        if any('/' in frame for frame in self.all_frames):
            print("⚠ Frames with namespaces found (can cause issues):")
            for frame in self.all_frames:
                if '/' in frame:
                    print(f"  - {frame}")
            self.warnings.append("Frames with namespaces detected")
            issues_found = True
        
        # Check for optical frames
        optical_frames = [f for f in self.all_frames if 'optical' in f]
        camera_frames = [f for f in self.all_frames if 'camera' in f and 'optical' not in f]
        
        if camera_frames and not optical_frames:
            print("⚠ Camera frames found but no optical frames")
            print("  Camera data uses optical frame convention (Z forward, Y down)")
            self.warnings.append("Missing camera optical frames")
            issues_found = True
        
        if not issues_found:
            print("✓ No common issues detected")
        
        print()
    
    def _check_circular_dependencies(self):
        """Check for circular dependencies in TF tree."""
        print("5. Circular Dependency Check")
        print("-" * 70)
        
        # Build parent-child relationships
        parent_child = defaultdict(list)
        
        for frame in self.all_frames:
            try:
                # Get parent frame
                transform = self.tf_buffer.lookup_transform(
                    frame, frame, rospy.Time(0), rospy.Duration(1.0)
                )
                parent = transform.header.frame_id
                if parent != frame:
                    parent_child[parent].append(frame)
            except:
                pass
        
        # Simple cycle detection
        def has_cycle(frame, visited, rec_stack):
            visited.add(frame)
            rec_stack.add(frame)
            
            for child in parent_child.get(frame, []):
                if child not in visited:
                    if has_cycle(child, visited, rec_stack):
                        return True
                elif child in rec_stack:
                    return True
            
            rec_stack.remove(frame)
            return False
        
        visited = set()
        has_cycles = False
        
        for frame in self.all_frames:
            if frame not in visited:
                if has_cycle(frame, visited, set()):
                    print(f"✗ Circular dependency detected involving: {frame}")
                    self.errors.append(f"Circular dependency in TF tree: {frame}")
                    has_cycles = True
        
        if not has_cycles:
            print("✓ No circular dependencies detected")
        
        print()
    
    def _similar_strings(self, s1, s2, threshold=0.8):
        """Check if two strings are similar (simple Levenshtein-like)."""
        if len(s1) < 3 or len(s2) < 3:
            return False
        
        # Simple similarity: shared characters / max length
        shared = sum(1 for c in s1 if c in s2)
        similarity = shared / max(len(s1), len(s2))
        
        return similarity > threshold and s1 != s2
    
    def _generate_visualization(self):
        """Generate TF tree visualization."""
        print("6. Generating Visualization")
        print("-" * 70)
        
        try:
            # Run view_frames
            print("Running view_frames (may take a few seconds)...")
            result = subprocess.run(
                ['rosrun', 'tf', 'view_frames'],
                capture_output=True,
                timeout=10
            )
            
            if result.returncode == 0:
                # Check if PDF was generated
                if os.path.exists('frames.pdf'):
                    print("✓ TF tree visualization saved to: frames.pdf")
                    print("  View with: evince frames.pdf")
                else:
                    print("⚠ view_frames ran but frames.pdf not found")
            else:
                print(f"✗ view_frames failed: {result.stderr.decode()}")
                
        except subprocess.TimeoutExpired:
            print("⚠ view_frames timed out")
        except FileNotFoundError:
            print("⚠ view_frames not found (install ros-$ROS_DISTRO-tf)")
        except Exception as e:
            print(f"⚠ Error generating visualization: {e}")
        
        print()
    
    def _print_summary(self):
        """Print validation summary."""
        print(f"{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")
        
        print(f"Total Frames: {len(self.all_frames)}")
        print(f"Required Frames: {len(self.required_frames)}")
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
            print("✅ TF tree looks good!")
        elif not self.errors:
            print("⚠️  TF tree has warnings but no critical errors")
        else:
            print("❌ TF tree has errors that need to be fixed")
            print("\nCommon fixes:")
            print("  1. Check URDF file is loaded: roslaunch your_package robot_description.launch")
            print("  2. Verify static_transform_publisher nodes are running")
            print("  3. Check frame names match between URDF and SLAM config")
            print("  4. Ensure robot_state_publisher is running for URDF")
        
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Validate ROS TF tree for SLAM integration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic check (auto-detect required frames):
  %(prog)s

  # Specify required frames:
  %(prog)s --frames map odom base_link os_sensor

  # Check specific transform:
  %(prog)s --source map --target base_link

  # Detailed output with visualization:
  %(prog)s --verbose --visualize
        """
    )
    
    parser.add_argument(
        '--frames', '-f',
        nargs='+',
        help='Required frame names to check'
    )
    
    parser.add_argument(
        '--source',
        help='Source frame for specific transform check'
    )
    
    parser.add_argument(
        '--target',
        help='Target frame for specific transform check'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed information'
    )
    
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Generate TF tree visualization (frames.pdf)'
    )
    
    args = parser.parse_args()
    
    try:
        validator = TFTreeValidator(
            required_frames=args.frames,
            verbose=args.verbose,
            visualize=args.visualize
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

