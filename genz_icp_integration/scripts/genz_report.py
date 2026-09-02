#!/usr/bin/env python3
"""Frame-invariant metrics + alpha (degeneracy) summary for one GenZ-ICP run.

Metrics match the ones used for EllipseLIO/SuperOdom/FAST-LIO so numbers are
directly comparable across estimators:
  path length, peak excursion from origin, final displacement (return-to-origin
  proxy when the flight started and ended at the same spot), max single step.
"""
import csv, math, sys

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
if not rows:
    print("empty trajectory"); sys.exit(1)

P = [(float(r["x"]), float(r["y"]), float(r["z"])) for r in rows]
d = lambda a, b: math.dist(a, b)

path_len = sum(d(P[i - 1], P[i]) for i in range(1, len(P)))
steps = [d(P[i - 1], P[i]) for i in range(1, len(P))] or [0.0]
peak = max(d(P[0], p) for p in P)
final = d(P[0], P[-1])

alphas = [float(r["alpha"]) for r in rows if r.get("alpha")]
t = [float(r["stamp"]) for r in rows]
span = t[-1] - t[0] if len(t) > 1 else 0.0

print(f"  poses            : {len(P)}  over {span:.1f}s  ({len(P)/span:.1f} Hz)" if span else f"  poses: {len(P)}")
print(f"  path length      : {path_len:.2f} m")
print(f"  peak excursion   : {peak:.2f} m")
print(f"  final displ.     : {final:.2f} m   <-- return-to-origin proxy")
print(f"  max single step  : {max(steps):.3f} m")
if alphas:
    a = sorted(alphas)
    q = lambda f: a[min(len(a) - 1, int(f * len(a)))]
    print(f"  alpha (planar frac): mean {sum(a)/len(a):.3f}  "
          f"p05 {q(.05):.3f}  median {q(.5):.3f}  p95 {q(.95):.3f}  n={len(a)}")
    print(f"  alpha < 0.5 (unstructured-dominant): {100.0*sum(1 for x in a if x<0.5)/len(a):.1f}% of scans")
else:
    print("  alpha            : NOT CAPTURED (debug clouds absent -- check visualize param)")
