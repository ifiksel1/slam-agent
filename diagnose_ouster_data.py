#!/usr/bin/env python3
"""
Ouster Data Flow Diagnostic Tool
Checks if LiDAR data is flowing from Ouster to SLAM
"""

import rospy
import sys
import time
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

def diagnose():
    print("\n" + "="*60)
    print("OUSTER DATA FLOW DIAGNOSTIC")
    print("="*60 + "\n")

    # Check ROS is initialized
    try:
        rospy.init_node('ouster_diagnostic', anonymous=True, disable_signals=True)
    except:
        print("✗ Failed to initialize ROS")
        return False

    # Check LiDAR topic
    print("[1/4] Checking /os_cloud_node/points topic...")
    lidar_received = False
    try:
        print("  Waiting for LiDAR data (10 second timeout)...")
        msg = rospy.wait_for_message('/os_cloud_node/points', PointCloud2, timeout=10.0)
        lidar_received = True
        print(f"  ✓ Received LiDAR message!")
        print(f"    - Frame ID: {msg.header.frame_id}")
        print(f"    - Timestamp: {msg.header.stamp}")
        print(f"    - Points: {msg.width} x {msg.height} = {msg.width * msg.height} points")
        print(f"    - Fields: {[f.name for f in msg.fields]}")
    except rospy.ROSException:
        print(f"  ✗ No data on /os_cloud_node/points (timeout after 10s)")
    except Exception as e:
        print(f"  ✗ Error: {e}")

    # Check IMU topic
    print("\n[2/4] Checking /os_cloud_node/imu topic...")
    try:
        from sensor_msgs.msg import Imu
        print("  Waiting for IMU data (5 second timeout)...")
        msg = rospy.wait_for_message('/os_cloud_node/imu', Imu, timeout=5.0)
        print(f"  ✓ Received IMU message!")
        print(f"    - Timestamp: {msg.header.stamp}")
        print(f"    - Accel: {msg.linear_acceleration}")
        print(f"    - Gyro: {msg.angular_velocity}")
    except rospy.ROSException:
        print(f"  ✗ No data on /os_cloud_node/imu (timeout)")
    except Exception as e:
        print(f"  ✗ Error: {e}")

    # Check SLAM Odometry
    print("\n[3/4] Checking /Odometry topic (SLAM output)...")
    try:
        print("  Waiting for SLAM odometry (5 second timeout)...")
        msg = rospy.wait_for_message('/Odometry', Odometry, timeout=5.0)
        print(f"  ✓ Received SLAM odometry!")
        pos = msg.pose.pose.position
        print(f"    - Position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")
        vel = msg.twist.twist.linear
        print(f"    - Velocity: x={vel.x:.4f}, y={vel.y:.4f}, z={vel.z:.4f}")
    except rospy.ROSException:
        if lidar_received:
            print(f"  ⚠ SLAM not generating odometry yet (but LiDAR data received)")
            print(f"    - SLAM may still be initializing")
        else:
            print(f"  ✗ No SLAM odometry and no LiDAR data")
    except Exception as e:
        print(f"  ✗ Error: {e}")

    # Check ROS topics
    print("\n[4/4] Checking ROS topic publications...")
    try:
        from subprocess import check_output
        output = check_output(['rostopic', 'list']).decode().strip().split('\n')
        ouster_topics = [t for t in output if 'os_' in t or 'ouster' in t]
        print(f"  Found {len(ouster_topics)} Ouster-related topics:")
        for topic in ouster_topics[:10]:  # Show first 10
            print(f"    - {topic}")
        if len(ouster_topics) > 10:
            print(f"    ... and {len(ouster_topics) - 10} more")
    except Exception as e:
        print(f"  Warning: Could not list topics: {e}")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    if lidar_received:
        print("✓ LiDAR data IS FLOWING from Ouster sensor")
        print("  Next: Check if SLAM is processing the data")
    else:
        print("✗ LiDAR data is NOT flowing")
        print("  Possible causes:")
        print("  1. Ouster driver not connected to sensor")
        print("  2. Ouster sensor not powered on")
        print("  3. Ethernet cable disconnected")
        print("  4. Network configuration issue")
        print("\n  Actions:")
        print("  - Verify eth0 has CARRIER: ip link show eth0")
        print("  - Check sensor reachable: ping 169.254.56.220")
        print("  - Check driver logs: tail ~/.ros/log/latest/*ouster*")

    print("="*60 + "\n")
    return lidar_received

if __name__ == '__main__':
    try:
        success = diagnose()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nFatal error: {e}")
        sys.exit(1)
