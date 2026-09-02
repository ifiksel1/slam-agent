#!/usr/bin/env python3
"""
Trim a ROS1 bag to the active-flight window: from 5s before the drone leaves its
starting rest pose to 5s after it settles at the end. Cuts dead hover time at
both ends -> smaller bags + faster playback, without touching the flight itself.

Motion is detected from /mavros/local_position/pose (onboard position estimate):
  start_rest = median of the first `rest` seconds of position
  end_rest   = median of the last  `rest` seconds
  motion_start = first time ||pos - start_rest|| > move_thresh
  motion_end   = last  time ||pos - end_rest||   > move_thresh
Everything is in bag receipt-time so the write filter is consistent.

  Usage: trim_bag.py <in.bag> <out.bag> [--pad 5] [--dry-run]
"""
import argparse, sys
import numpy as np
import rosbag


def load_positions(bag_path, topic):
    ts, xyz = [], []
    with rosbag.Bag(bag_path, 'r') as b:
        info = b.get_type_and_topic_info()
        t_start = b.get_start_time(); t_end = b.get_end_time()
        for _, msg, t in b.read_messages(topics=[topic]):
            p = msg.pose.position
            ts.append(t.to_sec()); xyz.append([p.x, p.y, p.z])
    return np.array(ts), np.array(xyz), t_start, t_end


def motion_window(ts, xyz, rest, move_thresh):
    if len(ts) < 10:
        return None
    t0 = ts[0]; tN = ts[-1]
    start_mask = ts <= t0 + rest
    end_mask = ts >= tN - rest
    start_rest = np.median(xyz[start_mask], axis=0)
    end_rest = np.median(xyz[end_mask], axis=0)
    d_start = np.linalg.norm(xyz - start_rest, axis=1)
    d_end = np.linalg.norm(xyz - end_rest, axis=1)
    moved_start = np.where(d_start > move_thresh)[0]
    moved_end = np.where(d_end > move_thresh)[0]
    if len(moved_start) == 0 or len(moved_end) == 0:
        return None  # never really moved
    t_motion_start = ts[moved_start[0]]
    t_motion_end = ts[moved_end[-1]]
    return t_motion_start, t_motion_end, start_rest, end_rest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inbag'); ap.add_argument('outbag')
    ap.add_argument('--topic', default='/mavros/local_position/pose')
    ap.add_argument('--pad', type=float, default=5.0)
    ap.add_argument('--rest', type=float, default=2.0, help='rest-window sec at each end')
    ap.add_argument('--move-thresh', type=float, default=0.15, help='m from rest = moving')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    ts, xyz, bag_t0, bag_t1 = load_positions(args.inbag, args.topic)
    if len(ts) == 0:
        print(f"ERROR: no {args.topic} in {args.inbag}"); sys.exit(2)
    win = motion_window(ts, xyz, args.rest, args.move_thresh)
    if win is None:
        print(f"WARN: no clear motion in {args.inbag} (drone never left rest?) -> NOT trimming")
        sys.exit(3)
    m_start, m_end, sr, er = win
    t0 = max(bag_t0, m_start - args.pad)
    t1 = min(bag_t1, m_end + args.pad)
    orig_dur = bag_t1 - bag_t0
    new_dur = t1 - t0

    print(f"{args.inbag.split('/')[-1]}")
    print(f"  bag duration      : {orig_dur:.1f}s")
    print(f"  motion start/end  : +{m_start-bag_t0:.1f}s .. +{m_end-bag_t0:.1f}s")
    print(f"  trim window (pad{args.pad:g}): +{t0-bag_t0:.1f}s .. +{t1-bag_t0:.1f}s")
    print(f"  new duration      : {new_dur:.1f}s  ({100*new_dur/orig_dur:.0f}% of original, "
          f"cuts {orig_dur-new_dur:.1f}s)")
    if args.dry_run:
        return

    import rospy
    n_in = n_out = 0
    with rosbag.Bag(args.inbag, 'r') as bin_, rosbag.Bag(args.outbag, 'w') as bout:
        for topic, msg, t in bin_.read_messages():
            n_in += 1
            if t0 <= t.to_sec() <= t1:
                bout.write(topic, msg, t)
                n_out += 1
    print(f"  wrote {args.outbag} ({n_out}/{n_in} msgs kept)")


if __name__ == '__main__':
    main()
