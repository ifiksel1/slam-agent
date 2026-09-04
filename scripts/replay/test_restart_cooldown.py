#!/usr/bin/env python3
"""Regression test for the restart loop observed on 2026-09-03 (FC log 52).

Replays the ACTUAL trigger timestamps from that flight's drift_monitor log:

  17:23:58.010  drift:matched<150(starv)                     eig=7669 matched=72
  17:25:17.938  drift:eig_min<5000,matched<150(starv)        eig=1760 matched=31
  17:25:23.348  ... same condition, 5.4 s later               eig=1979 matched=26
  17:25:28.573  ...                                           eig=1930 matched=29
  17:25:33.738  ...                                           eig=4651 matched=33
  17:25:39.066  ...                                           eig=4391 matched=32

PROVENANCE: these triggers were INDUCED. The operator held a hand in front of
the lidar, deliberately, with the vehicle disarmed on the bench. So this is not
evidence that the aircraft starves its own geometry in flight -- the confined
-space sweep puts the worst real sub-floor dwell at 3-4 scans, below the k_crit
of 5, meaning drift CRITICAL would not have fired at all in any of those 22
flights. It is a controlled, reproducible starvation source, which is what makes
it a good regression fixture.

What it exposed is real regardless of cause. Five restarts in 26 seconds: each
killed laserMapping, each re-warmed cleanly, and each re-fired within 2 s
because the occlusion was still there -- matched 26-33 against a floor of 150,
and a fresh process cannot invent features. Any sustained starvation source does
this: a hand, a featureless corridor, a tight corner. Armed, it would have been
optical flow with SLAM dying every 5.2 s.

Mirrors the cooldown gate in DriftMonitor._execute_restart. Keep the two in
step: this is a mirror, not the real object, because constructing one needs a
ROS master.
"""
import sys

# the observed triggers, seconds relative to the first
TRIGGERS = [0.000, 79.928, 85.338, 90.563, 95.728, 101.056]
COOLDOWN = 45.0

failures = []


def replay(triggers, cooldown, manual=False):
    """-> (restarts_performed, suppressed)"""
    last, restarts, suppressed = None, [], 0
    for t in triggers:
        cooling = (not manual and last is not None and (t - last) < cooldown)
        if cooling:
            suppressed += 1
            continue
        last = t
        restarts.append(t)
    return restarts, suppressed


def check(label, got, want):
    ok = got == want
    if not ok:
        failures.append(label)
    print("  %-52s %-12s want %-12s %s" % (label, got, want, "PASS" if ok else "**FAIL**"))


print("=== the bug, reproduced: no cooldown ===")
r, s = replay(TRIGGERS, 0.0)
check("every trigger restarts laserMapping", len(r), 6)
check("  none suppressed", s, 0)
print("      that is 5 restarts in %.0f s once the loop starts" % (TRIGGERS[-1] - TRIGGERS[1]))

print("=== with the 45 s cooldown ===")
r, s = replay(TRIGGERS, COOLDOWN)
check("restarts performed", len(r), 2)
check("  loop repeats suppressed", s, 4)
check("  the 17:23:58 event still restarts", r[0], 0.000)
check("  the 17:25:17 event still restarts", round(r[1], 3), 79.928)
print("      79.9 s apart, so the second is a genuine new fault, not a repeat")

print("=== a pilot CH9 restart is never suppressed ===")
r, s = replay([0.0, 1.0, 2.0], COOLDOWN, manual=True)
check("three CH9 holds -> three restarts", len(r), 3)
check("  none suppressed", s, 0)

print("=== a fault returning AFTER the cooldown restarts again ===")
r, s = replay([0.0, 50.0], COOLDOWN)
check("50 s later is a fresh restart", len(r), 2)

print("=== sustained starvation restarts at most once per cooldown ===")
r, s = replay([i * 5.2 for i in range(40)], COOLDOWN)   # 208 s of 5.2 s re-fires
check("208 s of continuous re-firing", len(r), 5)
print("      one restart per 45 s instead of 40 -- the failover carries the rest")

print()
print("ALL CHECKS PASSED" if not failures else "%d CHECK(S) FAILED" % len(failures))
sys.exit(1 if failures else 0)
