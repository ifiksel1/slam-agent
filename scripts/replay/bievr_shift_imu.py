#!/usr/bin/env python3
"""Write the /lidar_points and /lidar_imu topics of one or more bags (in order) to a new bag,
shifting every /lidar_imu header stamp by a fixed offset (0 = plain merge).
Used for split flight bags (BIEVR initialises assuming the vehicle is still, so a later split
part cannot be replayed on its own) and to test the lidar/IMU time offset, which BIEVR-LIO has
no parameter for. Usage: bievr_shift_imu.py <in.bag>[,<in2.bag>...] <out.bag> <offset_s>"""
import sys
import rosbag
import rospy

srcs, dst, off = sys.argv[1].split(','), sys.argv[2], rospy.Duration.from_sec(float(sys.argv[3]))
with rosbag.Bag(dst, 'w') as bo:
    for src in srcs:
        with rosbag.Bag(src) as bi:
            for topic, msg, t in bi.read_messages(topics=['/lidar_points', '/lidar_imu']):
                if topic == '/lidar_imu':
                    msg.header.stamp += off
                bo.write(topic, msg, t)
