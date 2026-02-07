#!/usr/bin/env python3
"""
SLAM Flight Startup with Post-Launch Verification
Launches SLAM, waits for initialization, then runs pre-flight checks
"""

import subprocess
import sys
import time
import os
import signal

class FlightStartup:
    def __init__(self):
        self.slam_workspace = "/home/dev/slam_ws"
        self.preflight_script = "/home/dev/slam-agent/preflight_check.sh"
        self.slam_process = None
        self.slam_ready = False

    def launch_slam(self):
        """Launch SLAM in background"""
        print("\n" + "="*70)
        print("STEP 1: LAUNCHING SLAM")
        print("="*70 + "\n")

        print("Starting SLAM process...\n")

        launch_cmd = (
            f"cd {self.slam_workspace} && "
            f"source devel/setup.bash && "
            f"roslaunch fast_lio mapping_ouster64.launch rviz:=false"
        )

        try:
            self.slam_process = subprocess.Popen(
                ["bash", "-c", launch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create process group
            )
            print(f"  ✓ SLAM process started (PID: {self.slam_process.pid})")
            self.slam_ready = True
            return True

        except Exception as e:
            print(f"  ✗ Failed to start SLAM: {e}")
            return False

    def wait_for_initialization(self, seconds=20):
        """Wait for SLAM to initialize"""
        print(f"\nWaiting for SLAM to initialize ({seconds} seconds)...")
        print("  SLAM is loading configuration and building maps...\n")

        for i in range(seconds, 0, -1):
            # Check if SLAM crashed
            if self.slam_process.poll() is not None:
                print(f"\n  ❌ SLAM process crashed!")
                return False

            print(f"  [{seconds - i + 1:2d}s] Initializing... ", end='\r')
            time.sleep(1)

        print(f"  [{seconds}s] SLAM initialization complete         ")
        print(f"  ✓ SLAM process still running")
        return True

    def run_preflight_check(self):
        """Run pre-flight checks while SLAM is running"""
        print("\n" + "="*70)
        print("STEP 2: RUNNING FLIGHT VERIFICATION CHECKS")
        print("="*70 + "\n")

        try:
            result = subprocess.run(
                ["bash", self.preflight_script],
                check=False,
                capture_output=False
            )
            return result.returncode == 0
        except Exception as e:
            print(f"Error running preflight check: {e}")
            return False

    def check_slam_status(self):
        """Verify SLAM is still running"""
        if self.slam_process.poll() is not None:
            print("\n❌ SLAM process crashed!")
            return False
        print("\n  ✓ SLAM process still running")
        return True

    def shutdown_slam(self, signal_received=None, frame=None):
        """Gracefully shutdown SLAM"""
        if self.slam_process and self.slam_ready:
            print("\n\nShutting down SLAM...")
            try:
                # Terminate the process group
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
                self.slam_process.wait(timeout=5)
                print("SLAM shutdown complete.")
            except:
                print("Force killing SLAM...")
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
            finally:
                self.slam_ready = False

    def run(self):
        """Main flight startup sequence"""
        print("\n")
        print("╔" + "="*68 + "╗")
        print("║" + " "*10 + "SLAM AUTONOMOUS FLIGHT STARTUP (POST-LAUNCH CHECK)" + " "*8 + "║")
        print("╚" + "="*68 + "╝")

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown_slam)

        # Step 1: Launch SLAM
        if not self.launch_slam():
            print("\n❌ Failed to launch SLAM")
            return False

        # Step 2: Wait for initialization
        if not self.wait_for_initialization(seconds=20):
            self.shutdown_slam()
            print("\n❌ SLAM failed during initialization")
            return False

        # Step 3: Run preflight checks
        if not self.run_preflight_check():
            print("\n" + "="*70)
            print("❌ FLIGHT VERIFICATION FAILED")
            print("="*70)
            print("\n⚠️  System is not ready for flight")
            print("\nSLAM is still running. Fix the issues above, then:")
            print("  1. Resolve the issues")
            print("  2. Restart SLAM: rosnode kill /laserMapping")
            print("  3. Run this script again")
            print("\nKeeping SLAM running for debugging...")
            print("Press Ctrl+C to shutdown SLAM.\n")
            try:
                self.slam_process.wait()
            except KeyboardInterrupt:
                self.shutdown_slam()
            return False

        # Success!
        if not self.check_slam_status():
            return False

        print("\n" + "="*70)
        print("✅ FLIGHT VERIFICATION PASSED - READY FOR FLIGHT")
        print("="*70)
        print("\nSystem Status:")
        print("  ✓ SLAM initialized and running")
        print("  ✓ LiDAR data flowing")
        print("  ✓ Odometry generating")
        print("  ✓ All systems verified")
        print("\nSLAM is now running. Safe to proceed with flight operations.")
        print("\nPress Ctrl+C to shutdown SLAM.\n")

        # Keep SLAM running
        try:
            self.slam_process.wait()
        except KeyboardInterrupt:
            self.shutdown_slam()

        return True


if __name__ == "__main__":
    startup = FlightStartup()
    success = startup.run()
    sys.exit(0 if success else 1)
