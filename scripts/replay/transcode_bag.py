#!/usr/bin/env python3
"""
Transcode a Hesai-format bag into the Velodyne point layout FAST-LIO (lidar_type=2) expects.

INPUT  /lidar_points:  x,y,z,intensity (f32), ring (u16), timestamp (f64 ABSOLUTE sec)  [point_step 26]
OUTPUT /lidar_points:  x,y,z,intensity (f32), ring (u16), time (f32 RELATIVE sec)        [point_step 22]
  time = float32(timestamp - min(timestamp in that scan))  -> 0 .. ~scan_period, matching the live driver.
/lidar_imu and all other topics + headers + bag-record times are passed through UNCHANGED.

Usage: transcode_bag.py <in.bag> <out.bag> [lidar_topic=/lidar_points]
"""
import sys
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2, PointField

IN  = sys.argv[1]
OUT = sys.argv[2]
LTOPIC = sys.argv[3] if len(sys.argv) > 3 else "/lidar_points"

IN_DT = np.dtype({'names':   ['x','y','z','intensity','ring','timestamp'],
                  'formats': ['<f4','<f4','<f4','<f4','<u2','<f8'],
                  'offsets': [0,4,8,12,16,18], 'itemsize': 26})
OUT_DT = np.dtype({'names':   ['x','y','z','intensity','ring','time'],
                   'formats': ['<f4','<f4','<f4','<f4','<u2','<f4'],
                   'offsets': [0,4,8,12,16,18], 'itemsize': 22})
OUT_FIELDS = [
    PointField('x',         0,  PointField.FLOAT32, 1),
    PointField('y',         4,  PointField.FLOAT32, 1),
    PointField('z',         8,  PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
    PointField('ring',      16, PointField.UINT16,  1),
    PointField('time',      18, PointField.FLOAT32, 1),
]

n_cloud = n_other = 0
with rosbag.Bag(IN) as bin_, rosbag.Bag(OUT, 'w') as bout:
    for topic, msg, t in bin_.read_messages():
        if topic == LTOPIC and msg._type == 'sensor_msgs/PointCloud2':
            npts = msg.width * msg.height
            a = np.frombuffer(msg.data, dtype=IN_DT, count=npts)
            ts = a['timestamp'].astype(np.float64)
            rel = (ts - ts.min()).astype(np.float32)   # per-point relative seconds
            o = np.zeros(npts, dtype=OUT_DT)
            for f in ('x', 'y', 'z', 'intensity', 'ring'):
                o[f] = a[f]
            o['time'] = rel
            m = PointCloud2()
            m.header       = msg.header           # unchanged stamp/frame
            m.height       = msg.height
            m.width        = msg.width
            m.fields       = OUT_FIELDS
            m.is_bigendian = msg.is_bigendian
            m.point_step   = 22
            m.row_step     = 22 * msg.width
            m.is_dense     = msg.is_dense
            m.data         = o.tobytes()
            bout.write(LTOPIC, m, t)             # same bag-record time
            if n_cloud < 3:
                print("[scan %d] npts=%d time %.4f..%.4fs" % (n_cloud, npts, rel.min(), rel.max()))
            n_cloud += 1
        else:
            bout.write(topic, msg, t)
            n_other += 1
print("DONE: %d clouds transcoded, %d other msgs passed -> %s" % (n_cloud, n_other, OUT))
