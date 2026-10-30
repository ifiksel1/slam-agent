#!/usr/bin/env python3
"""add_tf.py BAG [BAG...] -- add /tf (tf2_msgs/TFMessage) <odom frame_id> -> drone from /planner/odom.

For closed-loop bags, which have no /tf, so a Foxglove 3D panel can follow the robot ("drone", like the
planner-only bags). One /tf message per /planner/odom message, at its receive time and header stamp.
Writes <dir>/.<name>.tf_tmp.bag (LZ4), verifies it (same counts per topic, /tf count == odom count,
same time range, every message readable), then replaces the original. Skips bags that already have /tf.
"""
import os
import sys

from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ODOM, CHILD = "/planner/odom", "drone"
TF_MD5 = "94810edda583a504dfda3829e70d7eec"


def typestore_for(r):
    ts = get_typestore(Stores.ROS1_NOETIC)
    types = {}
    for c in r.connections:   # register every type from the bag's own definitions
        try:
            types.update(get_types_from_msg(c.msgdef.data, c.msgtype))
        except Exception as e:  # noqa: BLE001
            print("  (could not parse msgdef of %s: %s)" % (c.topic, e))
    types.update(get_types_from_msg("geometry_msgs/TransformStamped[] transforms", "tf2_msgs/msg/TFMessage"))
    ts.register({k: v for k, v in types.items() if k not in ts.types})
    return ts


def counts(path):
    with Reader(path) as r:
        n = {}
        for c in r.connections:
            n[c.topic] = n.get(c.topic, 0) + c.msgcount
        return n, r.start_time, r.end_time


def rewrite(src):
    d, base = os.path.split(os.path.abspath(src))
    tmp = os.path.join(d, "." + base[:-4] + ".tf_tmp.bag")
    if os.path.exists(tmp):
        os.remove(tmp)
    with Reader(src) as r:
        if any(c.topic == "/tf" for c in r.connections):
            print("skip %s: already has /tf" % src)
            return True
        ts = typestore_for(r)
        TF = ts.types["tf2_msgs/msg/TFMessage"]
        TS = ts.types["geometry_msgs/msg/TransformStamped"]
        Tr = ts.types["geometry_msgs/msg/Transform"]
        V3 = ts.types["geometry_msgs/msg/Vector3"]
        w = Writer(tmp)
        w.set_compression(Writer.CompressionFormat.LZ4)
        with w:
            out = {}
            for c in r.connections:
                out[c.id] = w.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest,
                                             callerid=getattr(c.ext, "callerid", None),
                                             latching=getattr(c.ext, "latching", None))
            tfc = w.add_connection("/tf", "tf2_msgs/msg/TFMessage", typestore=ts)
            if tfc.digest != TF_MD5:
                raise SystemExit("TFMessage md5 %s != %s" % (tfc.digest, TF_MD5))
            frames = set()
            for c, t, data in r.messages():
                w.write(out[c.id], t, data)
                if c.topic == ODOM:
                    o = ts.deserialize_ros1(data, c.msgtype)
                    p, q = o.pose.pose.position, o.pose.pose.orientation
                    frames.add(o.header.frame_id)
                    m = TF(transforms=[TS(header=o.header, child_frame_id=CHILD, transform=Tr(
                        translation=V3(x=p.x, y=p.y, z=p.z), rotation=q))])
                    w.write(tfc, t, ts.serialize_ros1(m, "tf2_msgs/msg/TFMessage"))
    # verify
    a, s0, e0 = counts(src)
    b, s1, e1 = counts(tmp)
    want = dict(a, **{"/tf": a.get(ODOM, 0)})
    ok = b == want and s0 == s1 and e0 == e1 and a.get(ODOM, 0) > 0
    if ok:
        n = 0
        with Reader(tmp) as r:
            for _ in r.messages():
                n += 1
        ok = n == sum(want.values())
    if not ok:
        print("VERIFY FAILED %s: kept original, temp at %s" % (src, tmp))
        print("  src", a, s0, e0)
        print("  tmp", b, s1, e1)
        return False
    os.replace(tmp, os.path.abspath(src))
    print("ok %s: +%d /tf (%s -> %s), %d topics, %.0f MB" % (src, want["/tf"], ",".join(sorted(frames)), CHILD,
                                                          len(want), os.path.getsize(src) / 1e6))
    return True


if __name__ == "__main__":
    bad = [b for b in sys.argv[1:] if not rewrite(b)]
    sys.exit(1 if bad else 0)
