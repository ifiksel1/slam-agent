#!/usr/bin/env python3
"""
test_latency_logic.py - synthetic tests for drift_monitor's latency detector.

No ROS, no bags, no hardware: stubs rospy and drives the real LatencyTracker plus a mirror of
health_cb's latency arithmetic with generated profiles. Runs in about a second.

It exists because it caught a real bug. lat_restart_blank_sec was armed only on a
/fastlio_health gap, so at NODE STARTUP - when drift_monitor comes up alongside a cold-starting
laserMapping - nothing suppressed the cold-start backlog. The detector fired CRITICAL 2.4 s in
and escalated to a node restart 7.5 s in, restarting laserMapping during its own cold start,
forever. The blank is now armed in __init__ too, and the cold-start case below is the regression
test for it.

Run:      python3 scripts/replay/test_latency_logic.py
Expected: crash profile fires, cold start never fires, healthy flight silent.

The bag-based counterpart is scripts/replay/latency_replay.py, which needs the flight bags.
"""
import os
import random
import sys
import types

def _drift_monitor_dir():
    """Prefer the live source, fall back to the tracked override copy.

    docker_src/ is gitignored, so a fresh clone has none of it and importing from there fails.
    overrides/ is the tracked copy of the same file, which is exactly why it exists.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in (("..", "..", "docker_src", "FAST_LIO_SLAM", "FAST_LIO_SLAM", "FAST-LIO", "src"),
                ("..", "..", "overrides", "FAST-LIO", "src")):
        cand = os.path.abspath(os.path.join(here, *rel))
        if os.path.isfile(os.path.join(cand, "drift_monitor.py")):
            return cand
    raise SystemExit("drift_monitor.py not found in docker_src/ or overrides/")


sys.path.insert(0, _drift_monitor_dir())

# LatencyTracker never touches rospy, so stub the imports drift_monitor does at module scope.
sys.modules["rospy"] = types.ModuleType("rospy")
for _n in ("std_msgs", "std_msgs.msg", "nav_msgs", "nav_msgs.msg",
           "mavros_msgs", "mavros_msgs.msg", "mavros_msgs.srv"):
    sys.modules[_n] = types.ModuleType(_n)
sys.modules["std_msgs.msg"].Float32MultiArray = object
sys.modules["std_msgs.msg"].String = object
sys.modules["nav_msgs.msg"].Odometry = object

from drift_monitor import LatencyTracker  # noqa: E402

# Must match the rosparam defaults in drift_monitor.py.
CRIT, WARN = 250.0, 150.0
K_CRIT, K_WARN = 20, 15
BLANK, DRAIN, WARMUP = 20.0, 5.0, 30
LAND_ARMED = True          # the terminal stage is gated on being armed

FAILURES = []


def sim(name, ages, hz=20.0, blank_at_start=True, warmup=WARMUP, land=False, armed=LAND_ARMED):
    """Mirror of health_cb's latency arithmetic. -> (warn_s, crit_s, escalate_s[, land_s])."""
    trk = LatencyTracker(5, 5.0)
    t, dt = 0.0, 1.0 / hz
    n_crit = n_warn = 0
    blank_until = BLANK if blank_at_start else -1e9
    latched, fw, fc, esc, failover_at, lnd = "OK", None, None, None, None, None

    for i, age in enumerate(ages):
        trk.add_sample(t, age)
        lat, _note = trk.evaluate(t)
        crit = lat is not None and lat >= CRIT
        warn = (not crit) and lat is not None and lat >= WARN
        if crit and (t < blank_until or i < warmup):
            crit, warn = False, True          # demoted: measure and warn, never act
        n_crit = n_crit + 1 if crit else 0
        n_warn = n_warn + 1 if (crit or warn) else 0
        if n_warn >= K_WARN and latched == "OK" and fw is None:
            fw, latched = t, "WARN"
        if n_crit >= K_CRIT and latched != "CRITICAL":
            fc, latched, failover_at = t, "CRITICAL", t
        if (failover_at is not None and esc is None and (t - failover_at) >= DRAIN
                and lat is not None and lat >= CRIT):
            esc = t
            blank_until = t + BLANK        # the restart re-arms the blank
        # terminal stage: blank expired, still over the ceiling, and armed
        if (land and lnd is None and esc is not None and t >= blank_until
                and lat is not None and lat >= CRIT and armed):
            lnd = t
        t += dt

    def fmt(v):
        return ("%.1fs" % v) if v is not None else "never"

    tail = ("  land@%s" % fmt(lnd)) if land else ""
    print("  %-44s warn@%-8s crit@%-8s escalate@%s%s" % (name, fmt(fw), fmt(fc), fmt(esc), tail))
    return fw, fc, esc, lnd


def expect(label, got, want):
    ok = got == want
    print("      %-52s %s" % (label, "PASS" if ok else "FAIL (got %r, want %r)" % (got, want)))
    if not ok:
        FAILURES.append(label)


def main():
    # 82 ms flat for 6 s, ramp to 669 ms over 6 s, then hold - the 25 Aug shape.
    crash = [82] * 120 + [82 + (669 - 82) * i / 120.0 for i in range(120)] + [600] * 280
    # 13.6 s backlog draining linearly to 78 ms over 9.1 s, then flat - the observed cold start.
    cold = [13600 - (13600 - 78) * i / 182.0 for i in range(182)] + [78] * 400
    random.seed(1)
    healthy = [71 + random.gauss(0, 4.5) for _ in range(2400)]

    print("=== crash profile, node already warm (the realistic case) ===")
    fw, fc, esc, _ = sim("crash", crash, blank_at_start=False)
    print("      ramp crosses 250 ms at ~7.7s; +K_CRIT(20 scans)=1.0s -> expect CRITICAL ~8.7s")
    expect("WARN fires", fw is not None, True)
    expect("CRITICAL fires", fc is not None, True)
    expect("CRITICAL within 8.0-9.5s", fc is not None and 8.0 <= fc <= 9.5, True)
    expect("escalates ~DRAIN after CRITICAL", esc is not None and abs(esc - fc - DRAIN) < 0.3, True)

    print("\n=== cold start - the regression test for the startup-blank bug ===")
    fw, fc, esc, _ = sim("cold start, blank armed", cold, blank_at_start=True)
    expect("never reaches CRITICAL", fc, None)
    expect("never escalates to a restart", esc, None)
    expect("still WARNs (so the interlock locks)", fw is not None, True)
    print("   for contrast, the bug this caught:")
    _, _bug_fc, bug_esc, _ = sim("cold start, blank NOT armed <-- old behaviour", cold, blank_at_start=False)
    expect("unfixed code would have restarted", bug_esc is not None, True)

    print("\n=== terminal LAND stage (enable_latency_land) ===")
    # SLAM never comes back: the restart fires, its blank expires, latency is still over the
    # ceiling. Should land. ~25 s after CRITICAL by construction (DRAIN 5 + BLANK 20).
    _, fc_l, esc_l, lnd = sim("never recovers -> lands", crash + [600] * 1200,
                              blank_at_start=False, land=True)
    expect("LAND fires", lnd is not None, True)
    expect("only after DRAIN+BLANK past CRITICAL",
           lnd is not None and abs(lnd - fc_l - DRAIN - BLANK) < 0.3, True)

    # The restart WORKS. Timeline: escalation at ~13.7 s kills laserMapping; roslaunch respawns
    # it after respawn_delay=2 s; its cold-start backlog then drains over ~9.1 s, finishing well
    # inside the 20 s blank. Must NOT land.
    # (An earlier version of this profile started the drain 17 s after the restart and did land -
    # which is the real sensitivity here: the blank is sized on ONE observed 9.1 s drain. A slower
    # cold start than that would put a normal recovery past the blank and trip the terminal stage.)
    recover = (crash[:300]                                  # ramp + CRITICAL + escalation
               + [600] * 40                                 # 2 s respawn delay
               + [13600 - (13600 - 78) * i / 182.0 for i in range(182)]   # cold start drains
               + [78] * 900)
    _, _, _, lnd2 = sim("restart recovers -> no land", recover, blank_at_start=False, land=True)
    expect("no LAND when the restart recovers SLAM", lnd2, None)

    # Disarmed on the bench: never command a descent.
    _, _, _, lnd3 = sim("disarmed -> never lands", crash + [600] * 1200,
                        blank_at_start=False, land=True, armed=False)
    expect("no LAND while disarmed", lnd3, None)

    print("\n=== healthy flight, 71 +/- 4.5 ms for 120 s ===")
    fw, fc, esc, _ = sim("healthy", healthy)
    expect("completely silent", (fw, fc, esc), (None, None, None))

    print("\n=== vision dropout: the starvation floor ===")
    trk = LatencyTracker(5, 5.0)
    for i in range(100):
        trk.add_sample(i * 0.05, 71)
    for gap in (0.5, 1.0, 3.0):
        lat, note = trk.evaluate(100 * 0.05 + gap)
        print("      +%.1fs since last sample -> lat=%.0fms note=%r" % (gap, lat, note))
    lat, note = trk.evaluate(100 * 0.05 + 1.0)
    expect("a 1 s dropout reads >= 1000 ms", lat >= 1000.0, True)
    expect("and is marked stale", note, "stale")

    print("\n=== clock-step rejection: a ONE-OFF step is a short blind spot ===")
    trk = LatencyTracker(5, 5.0)
    for i in range(20):
        trk.add_sample(i * 0.05, 71)
    accepted = trk.add_sample(1.05, 45000)
    expect("45 s sample rejected", accepted, False)
    expect("blanked, not poisoned", trk.evaluate(1.10), (None, "init"))
    trk.add_sample(1.15, 72)
    lat, note = trk.evaluate(1.15)   # evaluate at the sample instant: starvation term is zero
    expect("recovers on the next good sample", (round(lat), note), (72, ""))
    # Off the sample instant the starvation floor legitimately adds the elapsed time, because the
    # freshest pose really is that much older. At 20 Hz the reported value therefore breathes
    # between age and age+50 ms - by design, not noise.
    expect("starvation floor adds elapsed time", round(trk.evaluate(1.20)[0]), 122)

    print("\n=== wrong clock domain: EVERY sample implausible must escalate, not go silent ===")
    # Regression test. The pre-2026-08 bench bags stamp /Odometry with a monotonic uptime clock
    # while the bag records epoch, so every computed age is ~1.78e12 ms. The first version reset
    # the grace window on each rejection, so evaluate() returned "init" forever and the detector
    # was permanently, silently blind on 16 real bags.
    trk = LatencyTracker(5, 5.0)
    t = 0.0
    for _ in range(40):                       # 2 s of garbage, still inside the grace window
        trk.add_sample(t, 1.78e12)
        t += 0.05
    expect("inside grace: still init", trk.evaluate(t)[1], "init")
    for _ in range(120):                      # past grace_sec
        trk.add_sample(t, 1.78e12)
        t += 0.05
    lat, note = trk.evaluate(t)
    expect("past grace: escalates to no_odom", note, "no_odom")
    expect("and reports no number rather than nonsense", lat, None)

    print()
    if FAILURES:
        print("%d FAILURE(S): %s" % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
