---
name: ros_node_teardown_comm_truncation
description: "Per-cycle ROS2 node teardown must reap by PID or pkill -f, never ps -o comm (15-char truncation silently fails → leaked duplicate estimators)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7ab11289-52e2-4cda-b96b-f3410159290f
---

In replay-loop scripts that relaunch a ROS2 node each cycle (Foxglove bag demos, benchmark loops), tearing the node down with `ps -eo pid,comm | awk '/component_container/{print $1}'` SILENTLY FAILS: `ps -o comm` truncates the command name to 15 chars, so `component_container_mt` shows as `component_conta` and the `/component_container/` regex never matches. The old node is never killed, and every cycle stacks another → 2+ estimators publishing the SAME topics on domain 0 (`/ellipselio_odom` Publisher count > 1), giving garbled/jittery pose in Foxglove that looks like a "zombie process."

Discovered 2026-06-13 while watching IRIS-LIO on the 3rd_floor bag: `foxglove_bag_demo.sh` had this bug (apartment demo had it too). Diagnosed via `ros2 topic info /ellipselio_odom` → Publisher count: 2, and two `component_container_mt` PIDs with different ELAPSED times.

**Why:** `ps -o comm` is truncated; `ps -o args` (full cmdline) is not. `pkill -f <pattern>` matches the full cmdline (works) but self-matches the calling shell if the pattern appears in that shell's own command line (caused an exit-137 self-kill during cleanup).

**How to apply:**
- Per-cycle teardown: capture `LAUNCH_PID=$!` after `ros2 launch ... &`, then reap by PID: `kill -INT $LAUNCH_PID; sleep 3; pkill -9 -P $LAUNCH_PID; kill -9 $LAUNCH_PID`. PID-based = no truncation, no self-match.
- One-off cleanup from an interactive/agent shell: kill by explicit PID (from `ps -eo pid,args`), or `pkill -f` with a pattern that is NOT present in your own kill command's text. Never `pkill -f` a string that also appears in the kill line.
- Verify after any replay loop: `ros2 topic info <odom_topic>` should show Publisher count: 1. `ps -o args` (not `comm`) when matching node processes.
- Fixed runner: `coinlio_fusion_integration/scripts/run_iris_lio_foxglove.sh` (parameterized, PID-based). Relates to [[coinlio_ellipselio_photometric_fusion]].
