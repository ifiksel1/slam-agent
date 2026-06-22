---
name: clean-slate-between-repeated-runs
description: "Repeated SLAM A/B or benchmark runs MUST restart the container (verify `ros2 node list` empty) BEFORE each run to remove a stale-node/DDS contamination confound — but that is NECESSARY, NOT SUFFICIENT: EllipseLIO stays inherently non-deterministic on this rig even with clean slates (0.02–9.69 m run-to-run), so ALWAYS report N-run distributions, never single shots."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7ab11289-52e2-4cda-b96b-f3410159290f
---

When running the SAME bag/algorithm multiple times for a benchmark, determinism check, or A/B (photo vs geo, build vs build), give EACH run a clean DDS slate. Reaping the previous node by PID and relaunching in the same container — even with a few seconds' gap — is NOT enough.

**Why:** leftover ROS2 nodes and un-cleared CycloneDDS discovery from the prior run keep answering param queries, hold publishers, and compete on topics (`/ellipselio_odom`, `/clock`, etc.) → they ADD a contamination confound on top of any real signal. So a valid repeated comparison REQUIRES clean slates; otherwise you can't tell contamination from the effect you're measuring.

**IMPORTANT (clean slates are necessary but NOT sufficient):** clean slates are required, but they did NOT remove EllipseLIO's run-to-run non-determinism on this rig. Tested 2026-06-13 on the 3rd_floor bag, boosted clock, 14 clean-slate runs (`docker restart` + verified-empty node list before EACH run, `--rate 1.0`): final_disp was **bimodal and a near coin-flip — 7/14 tight (~0.02 m), 6/14 BLEW UP >5 m (6.6, 9.7, 13.2, 14.9, 26.2, and 302.2 m), 1 marginal (0.40 m)**. Same bag, same config, same clean state → ~43% catastrophic divergence. So the estimator is INHERENTLY non-deterministic — most likely multi-threaded FP reduction-order differences (`component_container_mt` + parallel tensor-voting) tipping a knife-edge / barely-observable spot in the bag (the same t≈230 s region where FAST-LIO hard-diverged), NOT zombie contamination and NOT scan-drop starvation (sample counts did NOT track the drift: the 302 m run had MORE samples than a tight run). The earlier dirty batteries (no restart) also swung wildly (0.01 → 77.69 m); a user hypothesis that zombies caused it was reasonable but the clean-slate test refuted it. CONSEQUENCE: every single-run SLAM number on this bag is meaningless (incl. the 4-way table and the photometric neutrality test); base-estimator determinism must be fixed before any IRIS-LIO/photometric benefit can be measured. Next probes (not yet run): `--rate 0.5` (compute headroom) and a SingleThreadedExecutor (rules out thread-level FP non-determinism).

**How to apply:**
- Between EVERY repeated run: `docker restart <ctr>; sleep ~8;` then verify `ros2 node list` is EMPTY before launching. Orchestrate the restarts from the HOST — you cannot `docker restart` from a script running inside that same container (it kills the script).
- Reap nodes by PID (`kill -INT $PID; pkill -9 -P $PID`), never `pkill -f` (self-matches the caller's cmdline → exit 137) and never `ps -o comm` (15-char truncation). See [[ros-node-teardown-comm-truncation]].
- Treat ANY single-run SLAM number on this rig as one draw from a wide distribution — report N-run distributions with clean slates, never single shots. This invalidated an earlier single-run 4-way comparison.
- The harness batteries (benchmark_bag.sh, run_*_bag.sh, the neutrality/determinism batteries) should bake the per-run restart + empty-node-list check in. Relates to [[coinlio-ellipselio-photometric-fusion]] and [[jetson-cpu-clock-boost]].
