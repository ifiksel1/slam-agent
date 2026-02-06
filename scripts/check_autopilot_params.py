#!/usr/bin/env python3
"""
ArduPilot/PX4 Parameter Validator
==================================

Validates autopilot parameters for SLAM/vision-based navigation.
Checks EKF configuration, safety settings, and common misconfigurations.

Usage:
    # Check ArduPilot parameters:
    ./check_autopilot_params.py --autopilot ardupilot

    # Check PX4 parameters:
    ./check_autopilot_params.py --autopilot px4

    # Auto-detect autopilot type:
    ./check_autopilot_params.py

    # Generate recommended parameter file:
    ./check_autopilot_params.py --generate recommended_params.parm

    # Compare against baseline:
    ./check_autopilot_params.py --baseline my_baseline.parm

Author: SLAM Integration Package
License: BSD-3-Clause
"""

import rospy
import sys
import argparse
from collections import defaultdict

try:
    from mavros_msgs.msg import ParamValue
    from mavros_msgs.srv import ParamGet, ParamSet, ParamPull
except ImportError:
    print("ERROR: MAVROS not found. Make sure MAVROS is installed and sourced.")
    sys.exit(1)


class AutopilotParamValidator:
    """Validates autopilot parameters for vision-based navigation."""
    
    # ArduPilot parameters for vision-based navigation
    ARDUPILOT_REQUIRED = {
        'EK3_SRC1_POSXY': {
            'value': 6,
            'description': 'Use vision for XY position',
            'critical': True
        },
        'EK3_SRC1_VELXY': {
            'value': 6,
            'description': 'Use vision for XY velocity',
            'critical': True
        },
        'EK3_SRC1_POSZ': {
            'value': 1,
            'description': 'Use barometer for Z position (or 6 for vision)',
            'critical': False,
            'alternatives': [1, 6]
        },
        'VISO_TYPE': {
            'value': 2,
            'description': 'MAVLink vision pose estimates',
            'critical': True
        },
    }
    
    ARDUPILOT_SAFETY = {
        'ARMING_CHECK': {
            'default': 1,
            'gps_denied': 388598,
            'description': 'Arming check configuration',
            'warning': 'GPS checks disabled - MUST enable geofence!'
        },
        'FENCE_ENABLE': {
            'recommended': 1,
            'description': 'Geofence enable',
            'critical_if': 'ARMING_CHECK',
            'warning': 'Geofence MUST be enabled when GPS checks are disabled'
        },
        'FENCE_TYPE': {
            'recommended': 7,
            'description': 'Geofence type (7 = cylinder)',
            'critical_if': 'FENCE_ENABLE'
        },
        'FENCE_RADIUS': {
            'recommended': 50,
            'description': 'Geofence radius (meters)',
            'warning': 'Set appropriate for your environment'
        },
        'FENCE_ALT_MAX': {
            'recommended': 20,
            'description': 'Geofence max altitude (meters)',
            'warning': 'Set appropriate for your environment'
        },
    }
    
    ARDUPILOT_GPS = {
        'GPS_TYPE': {
            'recommended': 0,
            'description': 'GPS type (0 = disabled for pure vision)',
            'warning': 'Consider disabling GPS for indoor/GPS-denied'
        },
    }
    
    # PX4 parameters for vision-based navigation
    PX4_REQUIRED = {
        'EKF2_AID_MASK': {
            'vision_bit': 24,  # Bit for vision position fusion
            'description': 'EKF2 aid sources mask',
            'critical': True
        },
        'EKF2_HGT_MODE': {
            'value': 3,  # Vision
            'description': 'Height sensor mode (3 = vision)',
            'critical': False,
            'alternatives': [0, 3]  # Baro or vision
        },
    }
    
    PX4_SAFETY = {
        'CBRK_GPSFAIL': {
            'value': 240024,
            'description': 'GPS failure circuit breaker',
            'warning': 'Disables GPS failure detection - use with caution'
        },
    }
    
    def __init__(self, autopilot=None, baseline_file=None):
        """
        Initialize validator.
        
        Args:
            autopilot: 'ardupilot' or 'px4' (auto-detect if None)
            baseline_file: Path to baseline parameter file for comparison
        """
        self.autopilot = autopilot
        self.baseline_file = baseline_file
        self.params = {}
        self.errors = []
        self.warnings = []
        self.baseline_params = {}
        
    def run(self, generate_file=None):
        """Run parameter validation."""
        print(f"\n{'='*70}")
        print("Autopilot Parameter Validator")
        print(f"{'='*70}\n")
        
        rospy.init_node('param_validator', anonymous=True)
        
        # Wait for MAVROS
        print("Waiting for MAVROS connection...")
        try:
            rospy.wait_for_service('/mavros/param/get', timeout=5.0)
            print("✓ MAVROS connected\n")
        except rospy.ROSException:
            print("✗ MAVROS connection timeout!")
            print("  Make sure MAVROS is running: roslaunch mavros apm.launch")
            return False
        
        # Auto-detect autopilot if not specified
        if not self.autopilot:
            self.autopilot = self._detect_autopilot()
            print(f"Detected autopilot: {self.autopilot.upper()}\n")
        
        # Load baseline if provided
        if self.baseline_file:
            self._load_baseline(self.baseline_file)
        
        # Pull parameters from autopilot
        print("Pulling parameters from autopilot...")
        if not self._pull_params():
            print("✗ Failed to pull parameters")
            return False
        print(f"✓ Retrieved {len(self.params)} parameters\n")
        
        # Run validation
        print(f"{'='*70}")
        print("VALIDATION RESULTS")
        print(f"{'='*70}\n")
        
        if self.autopilot == 'ardupilot':
            self._validate_ardupilot()
        elif self.autopilot == 'px4':
            self._validate_px4()
        
        # Compare with baseline if provided
        if self.baseline_params:
            self._compare_with_baseline()
        
        # Generate recommended file if requested
        if generate_file:
            self._generate_param_file(generate_file)
        
        # Summary
        self._print_summary()
        
        return len(self.errors) == 0
    
    def _detect_autopilot(self):
        """Auto-detect autopilot type from parameters."""
        # Try to get a parameter that's unique to each
        try:
            param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            
            # Try ArduPilot-specific parameter
            resp = param_get('SYSID_THISMAV')
            if resp.success:
                return 'ardupilot'
            
            # Try PX4-specific parameter
            resp = param_get('SYS_AUTOSTART')
            if resp.success:
                return 'px4'
                
        except:
            pass
        
        # Default to ArduPilot
        return 'ardupilot'
    
    def _pull_params(self):
        """Pull all parameters from autopilot."""
        try:
            # Request parameter list
            param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
            result = param_pull()
            
            if not result.success:
                return False
            
            # Wait a bit for parameters to be available
            rospy.sleep(2.0)
            
            # Get individual parameters
            param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
            
            # Get relevant parameters
            if self.autopilot == 'ardupilot':
                param_names = list(self.ARDUPILOT_REQUIRED.keys()) + \
                             list(self.ARDUPILOT_SAFETY.keys()) + \
                             list(self.ARDUPILOT_GPS.keys())
            else:
                param_names = list(self.PX4_REQUIRED.keys()) + \
                             list(self.PX4_SAFETY.keys())
            
            for param_name in param_names:
                try:
                    resp = param_get(param_name)
                    if resp.success:
                        self.params[param_name] = resp.value.integer or resp.value.real
                except:
                    pass
            
            return True
            
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
    
    def _validate_ardupilot(self):
        """Validate ArduPilot parameters."""
        print("1. Required EKF Parameters")
        print("-" * 70)
        
        for param, config in self.ARDUPILOT_REQUIRED.items():
            if param not in self.params:
                print(f"✗ {param:<20} NOT SET")
                if config['critical']:
                    self.errors.append(f"Missing critical parameter: {param}")
                else:
                    self.warnings.append(f"Missing parameter: {param}")
                continue
            
            value = self.params[param]
            expected = config['value']
            alternatives = config.get('alternatives', [])
            
            if value == expected or value in alternatives:
                print(f"✓ {param:<20} = {value:<10} ({config['description']})")
            else:
                print(f"✗ {param:<20} = {value:<10} (Expected: {expected})")
                if config['critical']:
                    self.errors.append(f"{param} = {value}, expected {expected}")
                else:
                    self.warnings.append(f"{param} = {value}, expected {expected}")
        
        print()
        
        # Safety parameters
        print("2. Safety Parameters")
        print("-" * 70)
        
        arming_check = self.params.get('ARMING_CHECK')
        fence_enabled = self.params.get('FENCE_ENABLE')
        
        # Check ARMING_CHECK
        if arming_check is not None:
            if arming_check == 1:
                print(f"✓ ARMING_CHECK        = {arming_check:<10} (All checks enabled - SAFE)")
            elif arming_check == 388598:
                print(f"⚠ ARMING_CHECK        = {arming_check:<10} (GPS checks disabled)")
                
                # MUST have geofence enabled
                if fence_enabled != 1:
                    print(f"✗ FENCE_ENABLE        = {fence_enabled or 'NOT SET':<10} (DANGEROUS!)")
                    self.errors.append("ARMING_CHECK=388598 requires FENCE_ENABLE=1")
                else:
                    print(f"✓ FENCE_ENABLE        = {fence_enabled:<10} (Required for GPS-denied)")
                    
                self.warnings.append("GPS arming checks are disabled")
            else:
                print(f"⚠ ARMING_CHECK        = {arming_check:<10} (Custom value)")
                self.warnings.append(f"Custom ARMING_CHECK value: {arming_check}")
        
        # Check geofence parameters
        for param, config in self.ARDUPILOT_SAFETY.items():
            if param == 'ARMING_CHECK' or param == 'FENCE_ENABLE':
                continue  # Already checked
            
            if param not in self.params:
                continue
            
            value = self.params[param]
            recommended = config.get('recommended')
            
            if recommended and value != recommended:
                print(f"⚠ {param:<20} = {value:<10} (Recommended: {recommended})")
                self.warnings.append(f"{param} = {value}, recommended {recommended}")
            else:
                print(f"✓ {param:<20} = {value:<10}")
        
        print()
        
        # GPS parameters
        print("3. GPS Configuration")
        print("-" * 70)
        
        gps_type = self.params.get('GPS_TYPE')
        if gps_type is not None:
            if gps_type == 0:
                print(f"✓ GPS_TYPE            = {gps_type:<10} (Disabled - pure vision)")
            else:
                print(f"⚠ GPS_TYPE            = {gps_type:<10} (GPS enabled)")
                print(f"  Consider disabling GPS (GPS_TYPE=0) for indoor/GPS-denied")
                self.warnings.append("GPS is enabled - may interfere indoors")
        
        print()
    
    def _validate_px4(self):
        """Validate PX4 parameters."""
        print("1. Required EKF2 Parameters")
        print("-" * 70)
        
        for param, config in self.PX4_REQUIRED.items():
            if param not in self.params:
                print(f"✗ {param:<20} NOT SET")
                if config['critical']:
                    self.errors.append(f"Missing critical parameter: {param}")
                else:
                    self.warnings.append(f"Missing parameter: {param}")
                continue
            
            value = self.params[param]
            
            if param == 'EKF2_AID_MASK':
                # Check if vision bit is set
                vision_enabled = bool(value & (1 << config['vision_bit']))
                if vision_enabled:
                    print(f"✓ {param:<20} = {value:<10} (Vision enabled)")
                else:
                    print(f"✗ {param:<20} = {value:<10} (Vision NOT enabled)")
                    self.errors.append(f"{param} vision bit not set")
            else:
                expected = config['value']
                alternatives = config.get('alternatives', [])
                
                if value == expected or value in alternatives:
                    print(f"✓ {param:<20} = {value:<10} ({config['description']})")
                else:
                    print(f"✗ {param:<20} = {value:<10} (Expected: {expected})")
                    if config['critical']:
                        self.errors.append(f"{param} = {value}, expected {expected}")
                    else:
                        self.warnings.append(f"{param} = {value}, expected {expected}")
        
        print()
        
        # Safety parameters
        print("2. Safety Parameters")
        print("-" * 70)
        
        for param, config in self.PX4_SAFETY.items():
            if param not in self.params:
                continue
            
            value = self.params[param]
            expected = config.get('value')
            
            if expected and value == expected:
                print(f"⚠ {param:<20} = {value:<10}")
                print(f"  {config['warning']}")
                self.warnings.append(config['warning'])
            else:
                print(f"✓ {param:<20} = {value:<10}")
        
        print()
    
    def _compare_with_baseline(self):
        """Compare current parameters with baseline."""
        print("4. Baseline Comparison")
        print("-" * 70)
        
        differences = []
        
        for param, baseline_value in self.baseline_params.items():
            current_value = self.params.get(param)
            
            if current_value is None:
                differences.append(f"{param}: NOT SET (baseline: {baseline_value})")
            elif abs(float(current_value) - float(baseline_value)) > 0.001:
                differences.append(f"{param}: {current_value} (baseline: {baseline_value})")
        
        if not differences:
            print("✓ All parameters match baseline")
        else:
            print(f"⚠ {len(differences)} parameters differ from baseline:")
            for diff in differences[:10]:  # Show first 10
                print(f"  • {diff}")
            if len(differences) > 10:
                print(f"  ... and {len(differences) - 10} more")
        
        print()
    
    def _load_baseline(self, filename):
        """Load baseline parameters from file."""
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) >= 2:
                            param_name = parts[0].strip()
                            param_value = float(parts[1].strip())
                            self.baseline_params[param_name] = param_value
            
            print(f"✓ Loaded {len(self.baseline_params)} baseline parameters from {filename}\n")
        except Exception as e:
            print(f"⚠ Failed to load baseline file: {e}\n")
    
    def _generate_param_file(self, filename):
        """Generate recommended parameter file."""
        print(f"5. Generating Recommended Parameters")
        print("-" * 70)
        
        try:
            with open(filename, 'w') as f:
                f.write("# Recommended parameters for vision-based navigation\n")
                f.write(f"# Generated by check_autopilot_params.py\n")
                f.write(f"# Autopilot: {self.autopilot.upper()}\n\n")
                
                if self.autopilot == 'ardupilot':
                    f.write("# EKF3 Vision Configuration\n")
                    for param, config in self.ARDUPILOT_REQUIRED.items():
                        f.write(f"{param},{config['value']}  # {config['description']}\n")
                    
                    f.write("\n# Safety Parameters\n")
                    f.write("ARMING_CHECK,388598  # GPS checks disabled (REQUIRES geofence!)\n")
                    f.write("FENCE_ENABLE,1  # CRITICAL: Enable geofence\n")
                    f.write("FENCE_TYPE,7  # Cylinder geofence\n")
                    f.write("FENCE_RADIUS,50  # Adjust for your environment\n")
                    f.write("FENCE_ALT_MAX,20  # Adjust for your environment\n")
                    
                    f.write("\n# GPS Configuration\n")
                    f.write("GPS_TYPE,0  # Disable GPS for indoor\n")
                
                elif self.autopilot == 'px4':
                    f.write("# EKF2 Vision Configuration\n")
                    f.write("EKF2_AID_MASK,24  # Enable vision position fusion\n")
                    f.write("EKF2_HGT_MODE,3  # Use vision for height\n")
                    
                    f.write("\n# Safety Parameters\n")
                    f.write("CBRK_GPSFAIL,240024  # Disable GPS failure detection\n")
            
            print(f"✓ Recommended parameters saved to: {filename}")
            print(f"  Load with: rosrun mavros mavparam load {filename}")
        
        except Exception as e:
            print(f"✗ Failed to generate file: {e}")
        
        print()
    
    def _print_summary(self):
        """Print validation summary."""
        print(f"{'='*70}")
        print("SUMMARY")
        print(f"{'='*70}\n")
        
        print(f"Autopilot: {self.autopilot.upper()}")
        print(f"Parameters Checked: {len(self.params)}")
        print(f"Errors: {len(self.errors)}")
        print(f"Warnings: {len(self.warnings)}")
        print()
        
        if self.errors:
            print("❌ CRITICAL ERRORS:")
            for error in self.errors:
                print(f"  • {error}")
            print()
            print("Fix with:")
            if self.autopilot == 'ardupilot':
                print("  rosrun mavros mavparam set EK3_SRC1_POSXY 6")
                print("  rosrun mavros mavparam set EK3_SRC1_VELXY 6")
                print("  rosrun mavros mavparam set VISO_TYPE 2")
            print()
        
        if self.warnings:
            print("⚠️  WARNINGS:")
            for warning in self.warnings:
                print(f"  • {warning}")
            print()
        
        if not self.errors and not self.warnings:
            print("✅ All parameters configured correctly!")
        elif not self.errors:
            print("⚠️  Parameters have warnings but no critical errors")
        else:
            print("❌ Critical parameter errors must be fixed before flight")
        
        print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Validate autopilot parameters for vision-based navigation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect and validate:
  %(prog)s

  # Specify autopilot type:
  %(prog)s --autopilot ardupilot
  %(prog)s --autopilot px4

  # Generate recommended parameter file:
  %(prog)s --generate recommended.parm

  # Compare with baseline:
  %(prog)s --baseline my_params.parm
        """
    )
    
    parser.add_argument(
        '--autopilot', '-a',
        choices=['ardupilot', 'px4'],
        help='Autopilot type (auto-detect if not specified)'
    )
    
    parser.add_argument(
        '--generate', '-g',
        metavar='FILE',
        help='Generate recommended parameter file'
    )
    
    parser.add_argument(
        '--baseline', '-b',
        metavar='FILE',
        help='Baseline parameter file for comparison'
    )
    
    args = parser.parse_args()
    
    try:
        validator = AutopilotParamValidator(
            autopilot=args.autopilot,
            baseline_file=args.baseline
        )
        
        success = validator.run(generate_file=args.generate)
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

