#!/usr/bin/env python3
"""
latency_replay.py — replay recorded flight bags through drift_monitor's latency detector.

Why this can work at all: LatencyTracker in drift_monitor.py never reads a clock itself —
every method takes now_sec — so the exact logic that flies can be driven with bag timestamps.
The bags all predate the feature, so latency is derived from /Odometry (t_recv - header.stamp);
laserMapping stamps /Odometry with lidar_end_time on the host clock, making that a true
age-of-measurement. This is the same quantity data[7] will publish, one hop earlier.

Usage:
  latency_replay.py BAG [BAG ...]           # per-bag detail
  latency_replay.py --summary BAG [BAG ...] # one line per bag (fleet sweep)
  latency_replay.py --calibrate BAG [...]   # pooled percentiles + threshold recommendation

Acceptance criteria this is meant to check:
  08_25_2026_20_38_04  (the crash)      WARN before the 250 ms crossing, CRITICAL ~1-1.5 s after;
                                        prints the margin to the 3.7 g impact from /mavros/imu/data
  08_25_2026_20_44_47  (13.6 s cold start)  interlock LOCKED throughout, ZERO restarts
  the other 22 bags                     zero WARN, zero CRIT — if a flat 67-78 ms flight trips
                                        WARN at 150 ms the threshold is wrong and nothing ships
"""
import argparse
import os
import sys

import rosbag

# Import the real tracker so the tested logic is the flown logic.
_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "..", "docker_src", "FAST_LIO_SLAM", "FAST_LIO_SLAM", "FAST-LIO", "src")
sys.path.insert(0, os.path.abspath(_SRC))
from drift_monitor import LatencyTracker  # noqa: E402

# Mirrors _evaluate_latency() and the latch/counter block in health_cb(). Only the arithmetic
# is duplicated (the subtle part - median, starvation floor, skew rejection - is imported).
# Keep these in step with the rosparam defaults in drift_monitor.py.
D = dict(lat_warn_ms=150.0, lat_crit_ms=250.0, lat_arm_ms=150.0, lat_drain_sec=5.0,
         lat_median_n=5, lat_k_warn=15, lat_k_crit=20, lat_k_release=40,
         lat_k_arm_lock=10, lat_k_arm_release=60,
         lat_startup_grace_sec=5.0, lat_restart_blank_sec=20.0)


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))]


def replay(path, cfg, verbose=True, warm=False):
    """Feed one bag through the detector. Returns a stats dict."""
    bag = rosbag.Bag(path)
    topics = bag.get_type_and_topic_info().topics

    if "/Odometry" not in topics:
        bag.close()
        return dict(name=os.path.basename(path), skipped="no /Odometry")

    trk = LatencyTracker(cfg["lat_median_n"], cfg["lat_startup_grace_sec"])
    n_warn = n_crit = n_ok = 0
    n_arm = n_armok = 0
    latched = "OK"
    lat_lock = False
    # Armed at the first sample, mirroring drift_monitor.__init__. Without this the harness
    # models the pre-fix node and reports a restart on the cold-start bag that the real node
    # would never perform - the "test the thing that flies" trap, hit for real.
    blank_until = None
    failover_at = None
    escalated = False
    events = []
    samples = []      # raw age at receipt
    eff = []          # effective age the detector thresholds on (median + starvation floor)
    t0 = None

    # Impact detection for the crash bag: biggest |a| - g spike on the FC IMU.
    impact_t = None
    if "/mavros/imu/data" in topics:
        best = 0.0
        for _, m, t in bag.read_messages(topics=["/mavros/imu/data"]):
            a = m.linear_acceleration
            mag = abs((a.x ** 2 + a.y ** 2 + a.z ** 2) ** 0.5 - 9.80665)
            if mag > best:
                best, impact_t = mag, t.to_sec()
        if best < 15.0:          # nothing that looks like a hard arrival
            impact_t = None

    for _, msg, t in bag.read_messages(topics=["/Odometry"]):
        now = t.to_sec()
        if t0 is None:
            t0 = now
            # warm=True models a node that booted long before the recording did - the normal
            # case in flight, where the stack is up minutes before takeoff. Without it the
            # harness applies a startup blank the real node would have finished long ago and
            # makes the detector look slower than it is.
            blank_until = None if warm else (t0 + cfg["lat_restart_blank_sec"])
        age_ms = (now - msg.header.stamp.to_sec()) * 1000.0
        trk.add_sample(now, age_ms)
        samples.append(age_ms)

        lat_ms, note = trk.evaluate(now)
        if lat_ms is not None:
            eff.append(lat_ms)

        # _evaluate_latency
        crit = warn = False
        if lat_ms is None:
            warn = (note == "no_odom")
        elif lat_ms >= cfg["lat_crit_ms"]:
            crit = True
        elif lat_ms >= cfg["lat_warn_ms"]:
            warn = True

        blanked = blank_until is not None and now < blank_until
        if crit and blanked:
            crit, warn = False, True

        n_crit = n_crit + 1 if crit else 0
        n_warn = n_warn + 1 if (crit or warn) else 0
        n_ok = 0 if (crit or warn) else n_ok + 1

        arm_bad = (lat_ms is None and note != "init") or (lat_ms is not None and lat_ms >= cfg["lat_arm_ms"])
        n_arm = n_arm + 1 if arm_bad else 0
        n_armok = 0 if arm_bad else n_armok + 1
        if not lat_lock and n_arm >= cfg["lat_k_arm_lock"]:
            lat_lock = True
            events.append((now - t0, "INTERLOCK LOCKED", "lat=%.0fms" % (lat_ms or -1)))
        elif lat_lock and n_armok >= cfg["lat_k_arm_release"]:
            lat_lock = False
            events.append((now - t0, "interlock released", ""))

        new = latched
        if n_crit >= cfg["lat_k_crit"]:
            new = "CRITICAL"
        elif n_warn >= cfg["lat_k_warn"] and latched != "CRITICAL":
            new = "WARN"
        if n_ok >= cfg["lat_k_release"]:
            new = "OK"

        if new != latched:
            events.append((now - t0, "%s -> %s" % (latched, new), "lat=%.0fms%s" % (lat_ms or -1, " " + note if note else "")))
            if new == "CRITICAL" and failover_at is None:
                events.append((now - t0, "ACTION: switch to OF (SRC2)", "no restart"))
                failover_at = now
                escalated = False
            latched = new

        if (failover_at is not None and not escalated
                and (now - failover_at) >= cfg["lat_drain_sec"]
                and lat_ms is not None and lat_ms >= cfg["lat_crit_ms"]):
            escalated = True
            events.append((now - t0, "ACTION: ESCALATE to node restart", "did not drain in %.0fs" % cfg["lat_drain_sec"]))
            blank_until = now + cfg["lat_restart_blank_sec"]

    bag.close()

    st = dict(name=os.path.basename(path), n=len(samples), dur=(now - t0) if samples else 0.0,
              p50=pct(samples, 50), p90=pct(samples, 90), p99=pct(samples, 99),
              mx=max(samples) if samples else 0.0,
              warns=sum(1 for e in events if "-> WARN" in e[1]),
              crits=sum(1 for e in events if "-> CRITICAL" in e[1]),
              failovers=sum(1 for e in events if "switch to OF" in e[1]),
              restarts=sum(1 for e in events if "ESCALATE" in e[1]),
              locks=sum(1 for e in events if "LOCKED" in e[1]),
              events=events, samples=samples, eff=eff, impact_t=impact_t, t0=t0)

    if verbose:
        print("=" * 78)
        print("%s   %.1f s, %d odom msgs" % (st["name"], st["dur"], st["n"]))
        print("  latency  p50 %.0f  p90 %.0f  p99 %.0f  max %.0f ms" % (st["p50"], st["p90"], st["p99"], st["mx"]))
        if not events:
            print("  no events — detector silent for the whole bag")
        for dt, what, detail in events:
            print("  %7.2fs  %-32s %s" % (dt, what, detail))
        if impact_t is not None:
            crit_ev = [e for e in events if "-> CRITICAL" in e[1]]
            print("  impact (>15 m/s^2 dev) at %.2fs" % (impact_t - t0))
            if crit_ev:
                print("  *** CRITICAL fired %.2f s BEFORE impact ***" % (impact_t - t0 - crit_ev[0][0]))
            else:
                print("  *** no CRITICAL before impact ***")
    return st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--summary", action="store_true", help="one line per bag")
    ap.add_argument("--calibrate", action="store_true", help="pooled percentiles + recommendation")
    ap.add_argument("--warn-ms", type=float, default=D["lat_warn_ms"])
    ap.add_argument("--crit-ms", type=float, default=D["lat_crit_ms"])
    ap.add_argument("--warm", action="store_true",
                    help="model a node already running before the recording started (normal in flight)")
    args = ap.parse_args()

    cfg = dict(D, lat_warn_ms=args.warn_ms, lat_crit_ms=args.crit_ms)
    stats, pooled = [], []

    if args.summary:
        print("%-34s %6s %5s %6s %6s %6s %6s %5s %5s %5s %5s" %
              ("bag", "dur_s", "n", "p50", "p90", "p99", "max", "WARN", "CRIT", "OF", "RST"))
        print("-" * 110)

    for b in args.bags:
        try:
            st = replay(b, cfg, verbose=not (args.summary or args.calibrate), warm=args.warm)
        except Exception as e:
            print("ERR %s: %s" % (os.path.basename(b), e))
            continue
        if st.get("skipped"):
            if args.summary:
                print("%-34s  %s" % (st["name"], st["skipped"]))
            continue
        stats.append(st)
        pooled.extend(st["eff"])
        if args.summary:
            print("%-34s %6.1f %5d %6.0f %6.0f %6.0f %6.0f %5d %5d %5d %5d" %
                  (st["name"], st["dur"], st["n"], st["p50"], st["p90"], st["p99"], st["mx"],
                   st["warns"], st["crits"], st["failovers"], st["restarts"]))

    if args.calibrate and pooled:
        print("\n=== pooled EFFECTIVE latency across %d bags, %d samples ===" % (len(stats), len(pooled)))
        print("    (what the detector thresholds on: median + starvation floor, not the raw age)")
        for p in (50, 90, 99, 99.9):
            print("  p%-5s %7.0f ms" % (p, pct(pooled, p)))
        print("  max    %7.0f ms" % max(pooled))
        for cand in (100, 150, 200, 250, 300):
            over = sum(1 for v in pooled if v >= cand)
            run = best = 0
            for v in pooled:
                run = run + 1 if v >= cand else 0
                best = max(best, run)
            print("  >=%4d ms: %6d samples (%.3f%%), longest consecutive run %d scans (%.1f s @20Hz)"
                  % (cand, over, 100.0 * over / len(pooled), best, best / 20.0))

    if stats and not args.calibrate:
        bad = [s for s in stats if s["warns"] or s["crits"]]
        print("\n%d/%d bags produced any event" % (len(bad), len(stats)))
        for s in bad:
            print("   %-34s WARN=%d CRIT=%d OF=%d RESTART=%d" %
                  (s["name"], s["warns"], s["crits"], s["failovers"], s["restarts"]))


if __name__ == "__main__":
    main()
