#!/usr/bin/env python3
# Scan a rosbag2 bag for SENSOR DROPOUTS — the thing that silently ruins a SLAM benchmark take.
# A simultaneous gap in BOTH /ouster/points and /ouster/imu means the single Ouster ethernet
# link went down (e.g. the tether stretched as you walked, or a loose/under-powered connector).
# During the blackout the estimator dead-reckons on nothing, so ANY LiDAR-inertial odometry
# diverges when data resumes — the take is unusable past the gap.
#
# Pure Python stdlib (sqlite3) — no ROS needed, runs anywhere the .db3 is readable.
# Usage: check_bag_gaps.py <bag_dir_or_db3> [more_bags...]
#   exit 0 = all clean, 1 = at least one dropout, 2 = couldn't read.
import sqlite3, glob, os, sys

# (topic, gap-threshold seconds, nominal period for the "clean" message)
TOPICS = [('/ouster/points', 0.15, 0.05), ('/ouster/imu', 0.05, 0.011)]
DROPOUT_S = 0.30  # a gap this large is a real blackout, not jitter

def find_db3(p):
    if p.endswith('.db3'):
        return p if os.path.exists(p) else None
    hits = glob.glob(os.path.join(p, '*.db3'))
    return hits[0] if hits else None

def scan_topic(cur, topic, thr):
    r = cur.execute("select id from topics where name=?", (topic,)).fetchone()
    if not r:
        return None
    ts = [x[0] for x in cur.execute(
        "select timestamp from messages where topic_id=? order by timestamp", (r[0],))]
    if len(ts) < 2:
        return (0, 0.0, 0.0, 0)
    t0 = ts[0]
    rel = [(t - t0) / 1e9 for t in ts]
    worst = worst_t = 0.0
    nbig = 0
    for i in range(len(rel) - 1):
        dt = rel[i + 1] - rel[i]
        if dt > thr:
            nbig += 1
        if dt > worst:
            worst, worst_t = dt, rel[i]
    return (len(ts), worst, worst_t, nbig)

def check(path):
    db = find_db3(path)
    if not db:
        print(f"  ✗ {path}: no .db3 found"); return 2
    cur = sqlite3.connect(db).cursor()
    span = None
    bad = False
    lines = []
    for topic, thr, _ in TOPICS:
        s = scan_topic(cur, topic, thr)
        if s is None:
            lines.append(f"    {topic}: (not in bag)"); continue
        n, worst, wt, nbig = s
        if worst > DROPOUT_S:
            bad = True
            lines.append(f"    ⚠ {topic}: {n} msgs — {worst*1000:.0f}ms DROPOUT at t={wt:.1f}s ({nbig} gaps >{thr*1000:.0f}ms)")
        else:
            lines.append(f"    ✓ {topic}: {n} msgs — max gap {worst*1000:.0f}ms (clean)")
    print(f"{os.path.basename(os.path.normpath(path))}:")
    print("\n".join(lines))
    print("  VERDICT: ⚠⚠ RE-RECORD — sensor blacked out (check the LiDAR cable/tether/power)" if bad
          else "  VERDICT: ✓ CLEAN — good for SLAM benchmarking")
    return 1 if bad else 0

def main():
    if len(sys.argv) < 2:
        print("usage: check_bag_gaps.py <bag_dir_or_db3> [more...]"); sys.exit(2)
    rc = 0
    for p in sys.argv[1:]:
        rc = max(rc, check(p))
    sys.exit(rc)

if __name__ == '__main__':
    main()
