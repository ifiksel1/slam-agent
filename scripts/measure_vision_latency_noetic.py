#!/usr/bin/env python3
"""
VISO_DELAY_MS calibration for the ROS 1 Noetic Hesai JT128 / FAST-LIO stack.

ROS 1 counterpart to measure_vision_latency.py (which is rclpy/ROS 2 only and
cannot run on this stack). Run it INSIDE the slam-hesai-fastlio container.

READ-ONLY: subscribes to topics only. Never writes a parameter, never sends a
command to the flight controller.

--------------------------------------------------------------------------
Platform this was written for
--------------------------------------------------------------------------
    platform   : Intel NUC, Docker host networking, container slam-hesai-fastlio
    lidar      : Hesai JT128, 128 lines @ 20 Hz, INTERNAL IMU on /lidar_imu,
                 driver use_timestamp_type: 1 (host receive clock)
    slam       : FAST-LIO (RevoluteRobotics FAST_LIO_SLAM fork), ROS 1 Noetic
    bridge     : fastlio_mavros_bridge.py, restamp_system_time: false
    autopilot  : ArduPilot
    fcu        : mRo Pixracer Pro, USB CDC-ACM /dev/ttyACM0:921600
    ekf        : ExternalNav -- EK3_SRC1_{POSXY,VELXY,YAW}=6, VISO_TYPE=1

The measured latency budget is SPECIFIC to this hardware and to the current
FAST-LIO tuning. A different LiDAR, frame rate, companion computer, or FC link
gives a different answer -- do not port the resulting VISO_DELAY_MS between
rigs. (This repo already made that mistake once: VISO_DELAY_MS=60 was carried
over from a Jetson Orin NX / Ouster OS1-64 / ROS 2 setup, where the pipeline was
genuinely faster. See solutions_log.yaml entries 1 and 7.)

The FAST-LIO term also moves with point_filter_num, max_iteration and the voxel
filter sizes, and idle bench CPU is less contended than flight. Re-measure after
any tuning change.

--------------------------------------------------------------------------
What is being measured, and why not the ROS 2 script's method
--------------------------------------------------------------------------
ArduPilot defines VISO_DELAY_MS as the delay between the vision sensor
MEASURING the position and the autopilot RECEIVING the message.

The measurement that matters is taken at the LAST hop only:

    age = rospy.Time.now() - msg.header.stamp        # at /mavros/vision_pose/pose

That is the delay up to the moment MAVROS serialises the MAVLink frame. Serial
transit is added afterwards as a separate estimated term. It is valid because
the vision-pose stamp IS the instant the pose refers to: FAST-LIO propagates to
pcl_end_time and deskews into the end-of-scan frame (IMU_Processing.hpp:291-292,
319-324) and stamps /Odometry with that same lidar_end_time (laserMapping.cpp:613),
and the bridge forwards it untouched under restamp_system_time:false
(fastlio_mavros_bridge.py:106).

DO NOT decompose the per-hop ages. header.stamp is NOT carried unchanged along
the chain: the driver stamps /lidar_points with frame.points[0].timestamp
(source_driver_ros1.hpp:247) = sweep START, while /Odometry is restamped to
lidar_end_time = sweep END, ~50 ms later at 20 Hz. The two ages are measured
against different reference instants, so their difference is meaningless. An
earlier version of this docstring claimed otherwise and derived a bogus "+17 ms
FAST-LIO compute" term; the true transport-to-transport latency is ~70 ms. The
VISO_DELAY_MS result was never affected -- it only ever used the last hop.

The ROS 2 script instead uses p95 of /Odometry -> /mavros/local_position/pose.
That is wrong on this stack for two reasons:

  1. It is a ROUND TRIP. /mavros/local_position/pose is the FC's OUTPUT coming
     back over telemetry, so it includes FC->host latency that is not part of
     the sensor delay -- while missing the scan -> /lidar_points segment
     entirely.
  2. p95 + margin is the wrong statistic. VISO_DELAY_MS is a fixed constant the
     EKF uses to index its state history buffer, so it wants the CENTRAL value.
     Padding it high makes the EKF fuse each measurement against a too-old
     state. Over- and under-estimating are both harmful.

--------------------------------------------------------------------------
Known limitation: the result is a LOWER BOUND
--------------------------------------------------------------------------
header.stamp is the host's PACKET RECEIVE time, not the true scan instant, so
LiDAR-internal acquisition and ethernet transit sit outside this measurement.
There is also a convention question worth up to ~+/-25 ms at 20 Hz: whether
FAST-LIO deskews to sweep start or sweep end shifts the true measurement epoch
within the frame. Treat the output as a well-grounded starting point and
validate in motion (see below).

--------------------------------------------------------------------------
Validating the value
--------------------------------------------------------------------------
HANDHELD AND DISARMED ONLY -- never as a flight test. Move the vehicle gently
and watch whether FC position innovation correlates with velocity. If innovation
grows with speed, step VISO_DELAY_MS and re-check.

Usage:
    python3 measure_vision_latency_noetic.py [--duration 30] [--json]
"""

import argparse
import json
import sys
from collections import defaultdict

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

# USB CDC-ACM @921600: a MAVLink2 VISION_POSITION_ESTIMATE frame is ~57 bytes
# (~0.6 ms on the wire); the rest is USB polling/buffering overhead.
SERIAL_TRANSIT_MS = 1.5

STAGES = (
    ("lidar", "/lidar_points", PointCloud2, "driver assembly"),
    ("odom", "/Odometry", Odometry, "+ FAST-LIO compute"),
    ("vision", "/mavros/vision_pose/pose", PoseStamped, "+ bridge hop"),
)


def stats(values):
    a = np.asarray(values, dtype=float)
    return {
        "count": int(a.size),
        "mean_ms": float(a.mean()),
        "median_ms": float(np.median(a)),
        "sd_ms": float(a.std()),
        "p05_ms": float(np.percentile(a, 5)),
        "p95_ms": float(np.percentile(a, 95)),
        "min_ms": float(a.min()),
        "max_ms": float(a.max()),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0,
                    help="measurement duration in seconds (default: 30)")
    ap.add_argument("--json", action="store_true", help="emit JSON (MCP-compatible)")
    args = ap.parse_args()

    ages = defaultdict(list)     # stage -> [age_ms]
    first_age = defaultdict(dict)  # stage -> {stamp_sec: age_ms} for pairing hops

    def record(stage, header):
        now = rospy.Time.now()
        if header.stamp.to_sec() == 0.0:
            return  # unstamped message; nothing to measure against
        age_ms = (now - header.stamp).to_sec() * 1000.0
        ages[stage].append(age_ms)
        first_age[stage].setdefault(round(header.stamp.to_sec(), 6), age_ms)

    rospy.init_node("viso_delay_calibration", anonymous=True, disable_signals=True)
    for key, topic, msg_type, _ in STAGES:
        rospy.Subscriber(topic, msg_type,
                         (lambda k: lambda m: record(k, m.header))(key),
                         queue_size=200)

    if not args.json:
        print("collecting %.0fs ..." % args.duration)
    rospy.sleep(args.duration)

    errors = []
    result = {"measurement_duration_s": args.duration, "stages": {}, "increments": {}}

    for key, topic, _, label in STAGES:
        if ages[key]:
            result["stages"][key] = dict(stats(ages[key]), topic=topic, label=label)
        else:
            errors.append("%s: no data (is the pipeline running?)" % topic)

    # Per-stage increments, paired on identical header.stamp so each delta is the
    # cost of exactly one hop rather than a difference of two distributions.
    for a, b, name in (("lidar", "odom", "fast_lio_compute"),
                       ("odom", "vision", "bridge_hop")):
        common = set(first_age[a]) & set(first_age[b])
        deltas = [first_age[b][s] - first_age[a][s] for s in common]
        if deltas:
            result["increments"][name] = stats(deltas)

    if "vision" in result["stages"]:
        v = result["stages"]["vision"]
        suggested = int(round(v["median_ms"] + SERIAL_TRANSIT_MS))
        result["recommendation"] = {
            "suggested_value_ms": suggested,
            "basis": ("median age at /mavros/vision_pose/pose (%.1f ms) + %.1f ms "
                      "estimated serial transit" % (v["median_ms"], SERIAL_TRANSIT_MS)),
            "caveat": "LOWER BOUND -- excludes LiDAR-internal acquisition; "
                      "deskew convention adds up to +/-25 ms uncertainty at 20 Hz",
        }
    else:
        errors.append("cannot recommend a value without /mavros/vision_pose/pose")

    result["errors"] = errors

    if args.json:
        print(json.dumps(result, indent=2))
        return 1 if errors else 0

    print("\n%-8s %6s %8s %8s %8s %8s %8s %8s"
          % ("stage", "n", "mean", "median", "sd", "p05", "p95", "max"))
    print("-" * 68)
    for key, _, _, _ in STAGES:
        s = result["stages"].get(key)
        if not s:
            print("%-8s  NO DATA" % key)
            continue
        print("%-8s %6d %7.1f %8.1f %8.1f %8.1f %8.1f %8.1f"
              % (key, s["count"], s["mean_ms"], s["median_ms"], s["sd_ms"],
                 s["p05_ms"], s["p95_ms"], s["max_ms"]))

    if result["increments"]:
        print("\nper-stage increments (matched on identical header.stamp):")
        for name, s in result["increments"].items():
            print("  %-18s n=%-5d mean=%6.1f ms  median=%6.1f ms  p95=%6.1f ms"
                  % (name, s["count"], s["mean_ms"], s["median_ms"], s["p95_ms"]))

    if "recommendation" in result:
        r = result["recommendation"]
        print("\n" + "=" * 68)
        print("VISO_DELAY_MS recommendation: %d ms" % r["suggested_value_ms"])
        print("  basis:  %s" % r["basis"])
        print("  caveat: %s" % r["caveat"])
        print("  validate HANDHELD + DISARMED; never as a flight test.")
        print("=" * 68)

    for e in errors:
        print("  ! %s" % e, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
