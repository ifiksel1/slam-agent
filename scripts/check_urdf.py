#!/usr/bin/env python3
"""
URDF Validator
==============

Validates URDF files for SLAM integration systems.
Checks syntax, physical plausibility, frame names, and common mistakes.

Usage:
    # Check URDF file:
    ./check_urdf.py my_robot.urdf

    # Check URDF from parameter server:
    ./check_urdf.py --from-param /robot_description

    # Verbose output with visualization:
    ./check_urdf.py my_robot.urdf --verbose --visualize

    # Check specific frames exist:
    ./check_urdf.py my_robot.urdf --required-frames base_link os_sensor camera_link

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import sys
import argparse
import xml.etree.ElementTree as ET
import math
import rospy

try:
    import urdf_parser_py.urdf as urdf_parser
except ImportError:
    urdf_parser = None
    print("⚠ urdf_parser_py not found. Installing: pip install urdf-parser-py")


class URDFValidator:
    """Validates URDF files for SLAM integration."""
    
    def __init__(self, urdf_file=None, urdf_param=None, required_frames=None, verbose=False, visualize=False):
        """
        Initialize validator.
        
        Args:
            urdf_file: Path to URDF file
            urdf_param: ROS parameter name containing URDF
            required_frames: List of frame names that must exist
            verbose: Print detailed information
            visualize: Generate visual representation
        """
        self.urdf_file = urdf_file
        self.urdf_param = urdf_param
        self.required_frames = required_frames or []
        self.verbose = verbose
        self.visualize = visualize
        
        self.urdf_string = None
        self.urdf_tree = None
        self.robot = None
        
        self.links = {}
        self.joints = {}
        self.frames = set()
        
        self.errors = []
        self.warnings = []
    
    def run(self):
        """Run URDF validation."""
        print(f"\n{'='*70}")
        print("URDF Validator")
        print(f"{'='*70}\n")
        
        # Load URDF
        if not self._load_urdf():
            return False
        
        # Parse URDF
        if not self._parse_urdf():
            return False
        
        print(f"✓ URDF loaded successfully")
        print(f"  Links: {len(self.links)}")
        print(f"  Joints: {len(self.joints)}")
        print()
        
        # Run checks
        print(f"{'='*70}")
        print("VALIDATION RESULTS")
        print(f"{'='*70}\n")
        
        self._check_required_frames()
        self._check_physical_plausibility()
        self._check_common_mistakes()
        self._check_transform_chains()
        
        if self.visualize:
            self._generate_visualization()
        
        # Summary
        self._print_summary()
        
        return len(self.errors) == 0
    
    def _load_urdf(self):
        """Load URDF from file or parameter server."""
        if self.urdf_file:
            print(f"Loading URDF from file: {self.urdf_file}")
            try:
                with open(self.urdf_file, 'r') as f:
                    self.urdf_string = f.read()
                return True
            except Exception as e:
                print(f"✗ Failed to load URDF file: {e}")
                return False
        
        elif self.urdf_param:
            print(f"Loading URDF from parameter: {self.urdf_param}")
            try:
                rospy.init_node('urdf_validator', anonymous=True)
                self.urdf_string = rospy.get_param(self.urdf_param)
                return True
            except Exception as e:
                print(f"✗ Failed to load URDF from parameter server: {e}")
                return False
        
        else:
            print("✗ No URDF source specified")
            return False
    
    def _parse_urdf(self):
        """Parse URDF string."""
        try:
            # Parse XML
            self.urdf_tree = ET.fromstring(self.urdf_string)
            
            # Extract links
            for link in self.urdf_tree.findall('link'):
                name = link.get('name')
                self.links[name] = link
                self.frames.add(name)
            
            # Extract joints
            for joint in self.urdf_tree.findall('joint'):
                name = joint.get('name')
                self.joints[name] = {
                    'xml': joint,
                    'parent': joint.find('parent').get('link') if joint.find('parent') is not None else None,
                    'child': joint.find('child').get('link') if joint.find('child') is not None else None,
                    'type': joint.get('type'),
                    'origin': self._parse_origin(joint.find('origin'))
                }
                if self.joints[name]['child']:
                    self.frames.add(self.joints[name]['child'])
            
            # Try advanced parsing if available
            if urdf_parser:
                self.robot = urdf_parser.Robot.from_xml_string(self.urdf_string)
            
            return True
            
        except Exception as e:
            print(f"✗ Failed to parse URDF: {e}")
            self.errors.append(f"URDF parsing error: {e}")
            return False
    
    def _parse_origin(self, origin_elem):
        """Parse origin element to xyz and rpy."""
        if origin_elem is None:
            return {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
        
        xyz_str = origin_elem.get('xyz', '0 0 0')
        rpy_str = origin_elem.get('rpy', '0 0 0')
        
        xyz = [float(x) for x in xyz_str.split()]
        rpy = [float(x) for x in rpy_str.split()]
        
        return {'xyz': xyz, 'rpy': rpy}
    
    def _check_required_frames(self):
        """Check that required frames exist."""
        print("1. Required Frames Check")
        print("-" * 70)
        
        if not self.required_frames:
            print("⚠ No required frames specified")
            print("  Common frames: base_link, base_footprint, <sensor>_link")
            print()
            return
        
        for frame in self.required_frames:
            if frame in self.frames:
                print(f"✓ {frame:<30} FOUND")
            else:
                print(f"✗ {frame:<30} MISSING")
                self.errors.append(f"Required frame not found: {frame}")
        
        # Check for base_link (should always exist)
        if 'base_link' not in self.frames:
            print(f"\n⚠ WARNING: 'base_link' not found (standard practice)")
            self.warnings.append("base_link not found")
        
        print()
    
    def _check_physical_plausibility(self):
        """Check physical plausibility of transforms."""
        print("2. Physical Plausibility Check")
        print("-" * 70)
        
        for joint_name, joint_data in self.joints.items():
            origin = joint_data['origin']
            xyz = origin['xyz']
            rpy = origin['rpy']
            
            # Check position magnitude (sensors should be within ~2m of base)
            distance = math.sqrt(sum(x**2 for x in xyz))
            
            if distance > 2.0:
                print(f"⚠ {joint_name}: Large offset ({distance:.2f}m)")
                print(f"  xyz: [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]")
                self.warnings.append(f"{joint_name} has large offset: {distance:.2f}m")
            
            # Check for suspicious values (likely units mistake)
            for i, val in enumerate(xyz):
                if abs(val) > 10:
                    print(f"⚠ {joint_name}: Suspicious {'xyz'[i]} value: {val:.3f}")
                    print(f"  Check units - should be meters, not mm!")
                    self.warnings.append(f"{joint_name} suspicious value (mm vs m?)")
            
            # Check rotations (should be in radians)
            for i, val in enumerate(rpy):
                if abs(val) > 10:
                    print(f"⚠ {joint_name}: Suspicious {'rpy'[i]} rotation: {val:.3f}")
                    print(f"  Rotations should be radians, not degrees!")
                    print(f"  (3.14159 rad = 180°, 1.57 rad = 90°)")
                    self.warnings.append(f"{joint_name} rotation in degrees?")
                    
            if self.verbose and distance <= 2.0 and all(abs(v) <= 10 for v in xyz + rpy):
                print(f"✓ {joint_name}: Plausible")
                print(f"  xyz: [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]")
                print(f"  rpy: [{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]")
        
        print()
    
    def _check_common_mistakes(self):
        """Check for common URDF mistakes."""
        print("3. Common Mistakes Check")
        print("-" * 70)
        
        issues_found = False
        
        # Check for degree vs radian confusion
        for joint_name, joint_data in self.joints.items():
            rpy = joint_data['origin']['rpy']
            
            for angle in rpy:
                # Common degree values that look like radians
                if abs(angle - 90) < 1 or abs(angle - 180) < 1 or abs(angle - 270) < 1:
                    print(f"⚠ {joint_name}: Rotation value {angle:.1f} looks like degrees!")
                    print(f"  URDF uses radians: 90° = 1.57, 180° = 3.14")
                    self.warnings.append(f"{joint_name} likely using degrees instead of radians")
                    issues_found = True
        
        # Check for meter vs mm confusion
        for joint_name, joint_data in self.joints.items():
            xyz = joint_data['origin']['xyz']
            
            for val in xyz:
                # If value is between 10-1000, likely mm instead of m
                if 10 < abs(val) < 1000:
                    print(f"⚠ {joint_name}: Position value {val:.1f} looks like mm!")
                    print(f"  URDF uses meters: 60mm = 0.06m")
                    self.warnings.append(f"{joint_name} likely using mm instead of m")
                    issues_found = True
        
        # Check for similar frame names (typos)
        frame_list = sorted(self.frames)
        for i, frame1 in enumerate(frame_list):
            for frame2 in frame_list[i+1:]:
                if self._similar_strings(frame1, frame2):
                    print(f"⚠ Similar frame names (possible typo?):")
                    print(f"  - {frame1}")
                    print(f"  - {frame2}")
                    self.warnings.append(f"Similar names: {frame1} vs {frame2}")
                    issues_found = True
        
        # Check for optical frames (cameras)
        camera_frames = [f for f in self.frames if 'camera' in f.lower()]
        optical_frames = [f for f in self.frames if 'optical' in f.lower()]
        
        if camera_frames and not optical_frames:
            print(f"⚠ Camera frames found but no optical frames")
            print(f"  Camera data uses optical convention (Z forward, Y down)")
            print(f"  Add optical frames: {camera_frames[0]}_optical_frame")
            self.warnings.append("Missing camera optical frames")
            issues_found = True
        
        if not issues_found:
            print("✓ No common mistakes detected")
        
        print()
    
    def _check_transform_chains(self):
        """Check transform chains from base_link to sensors."""
        print("4. Transform Chain Check")
        print("-" * 70)
        
        if 'base_link' not in self.frames:
            print("⚠ Cannot check chains - base_link not found")
            print()
            return
        
        # Find sensor frames
        sensor_keywords = ['sensor', 'lidar', 'camera', 'imu', 'gps']
        sensor_frames = [f for f in self.frames 
                        if any(kw in f.lower() for kw in sensor_keywords)]
        
        if not sensor_frames:
            print("⚠ No sensor frames detected")
            print()
            return
        
        # Check connectivity to base_link
        for sensor_frame in sensor_frames:
            chain = self._find_transform_chain('base_link', sensor_frame)
            
            if chain:
                print(f"✓ base_link → {sensor_frame}")
                if self.verbose:
                    print(f"  Chain: {' → '.join(chain)}")
            else:
                print(f"✗ base_link → {sensor_frame} (NOT CONNECTED)")
                self.errors.append(f"No transform chain: base_link → {sensor_frame}")
        
        print()
    
    def _find_transform_chain(self, start, end, visited=None):
        """Find transform chain between two frames."""
        if visited is None:
            visited = set()
        
        if start == end:
            return [start]
        
        if start in visited:
            return None
        
        visited.add(start)
        
        # Find joints where start is the parent
        for joint_name, joint_data in self.joints.items():
            if joint_data['parent'] == start:
                child = joint_data['child']
                chain = self._find_transform_chain(child, end, visited.copy())
                if chain:
                    return [start] + chain
        
        # Also try reverse (child to parent)
        for joint_name, joint_data in self.joints.items():
            if joint_data['child'] == start:
                parent = joint_data['parent']
                chain = self._find_transform_chain(parent, end, visited.copy())
                if chain:
                    return [start] + chain
        
        return None
    
    def _similar_strings(self, s1, s2, threshold=0.8):
        """Check if two strings are similar."""
        if len(s1) < 3 or len(s2) < 3:
            return False
        
        shared = sum(1 for c in s1 if c in s2)
        similarity = shared / max(len(s1), len(s2))
        
        return similarity > threshold and s1 != s2
    
    def _generate_visualization(self):
        """Generate URDF visualization."""
        print("5. Generating Visualization")
        print("-" * 70)
        
        try:
            # Simple text-based tree
            print("Frame Tree:")
            print()
            self._print_tree('base_link' if 'base_link' in self.frames else list(self.frames)[0], 0)
            
        except Exception as e:
            print(f"⚠ Visualization error: {e}")
        
        print()
    
    def _print_tree(self, frame, indent, visited=None):
        """Print frame tree recursively."""
        if visited is None:
            visited = set()
        
        if frame in visited:
            return
        
        visited.add(frame)
        
        print("  " * indent + f"├─ {frame}")
        
        # Find children
        for joint_name, joint_data in self.joints.items():
            if joint_data['parent'] == frame:
                child = joint_data['child']
                if child and child not in visited:
                    self._print_tree(child, indent + 1, visited)
    
    def _print_summary(self):
        """Print validation summary."""
        print(f"{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")
        
        print(f"URDF Structure:")
        print(f"  Links: {len(self.links)}")
        print(f"  Joints: {len(self.joints)}")
        print(f"  Frames: {len(self.frames)}")
        print()
        print(f"Validation:")
        print(f"  Errors: {len(self.errors)}")
        print(f"  Warnings: {len(self.warnings)}")
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
            print("✅ URDF looks good!")
        elif not self.errors:
            print("⚠️  URDF has warnings but no critical errors")
        else:
            print("❌ URDF has errors that need to be fixed")
            print("\nCommon fixes:")
            print("  1. Use radians for rotations (3.14159 = 180°, not 180)")
            print("  2. Use meters for positions (0.06 = 60mm, not 60)")
            print("  3. Ensure all sensors connect to base_link")
            print("  4. Add optical frames for cameras")
        
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Validate URDF files for SLAM integration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Check URDF file:
  %(prog)s my_robot.urdf

  # Check from ROS parameter:
  %(prog)s --from-param /robot_description

  # Verbose with visualization:
  %(prog)s my_robot.urdf --verbose --visualize

  # Check specific frames:
  %(prog)s my_robot.urdf --required-frames base_link os_sensor camera_link
        """
    )
    
    parser.add_argument(
        'urdf_file',
        nargs='?',
        help='Path to URDF file'
    )
    
    parser.add_argument(
        '--from-param',
        metavar='PARAM',
        help='Load URDF from ROS parameter (e.g., /robot_description)'
    )
    
    parser.add_argument(
        '--required-frames', '-f',
        nargs='+',
        help='Required frame names to check'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Print detailed information'
    )
    
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Generate frame tree visualization'
    )
    
    args = parser.parse_args()
    
    if not args.urdf_file and not args.from_param:
        parser.error("Either URDF file or --from-param must be specified")
    
    try:
        validator = URDFValidator(
            urdf_file=args.urdf_file,
            urdf_param=args.from_param,
            required_frames=args.required_frames,
            verbose=args.verbose,
            visualize=args.visualize
        )
        
        success = validator.run()
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

