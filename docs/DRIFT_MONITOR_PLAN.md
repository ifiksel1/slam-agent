# Plan: FAST-LIO Drift Early-Warning + Auto-Response (`drift_monitor`)

> Copy of the implementation plan for in-repo reference. Design summary also lives in `PROGRESS.yaml` (P13).

## Executive summary
- **Goal:** a read-only ROS1 node (`drift_monitor`) that watches `/fastlio_health` and flags drift *before* the trajectory diverges, using `eig_min` (primary) + `matched_pts` (secondary).
- **WARN** → push a message to the GCS banner (`/mavros/statustext/send`) + publish `/slam/drift_risk`. **No FC action** — fully read-only.
- **CRITICAL** → (a) command FC to **LAND**, (b) **restart only the `laserMapping` node** (not the container, not the OS, not mavros), (c) hold an **advisory arm-interlock** until SLAM is healthy again.
- **Restart mechanism:** add `respawn="true"` to the `laserMapping` node; the monitor restarts it with a single `rosnode kill /laserMapping` and roslaunch respawns it (same params, siblings untouched). No container/OS restart, no root.
- **Arm-interlock = advisory + natural EKF:** during the restart, `vision_pose` drops, so ArduPilot's EKF already refuses to arm (GPS-denied = no position) — a *natural* interlock; the node publishes `/slam/arm_interlock` + a "DO NOT ARM" banner and releases after K healthy scans. **No FC param writes.**
- **Phased + safety-gated:** Phase 1 (detection+WARN) and Phase 2 (restart) are read-only and offline-testable. Phase 3 (FC-commanding LAND) ships behind `enable_critical_fc` (**default OFF**) and requires explicit sign-off — it crosses the project's hard "never command the FC" rule.
- **Test offline first** on `massrobotics_3rd_floor.bag` via the existing replay harness; thresholds come from `calibrate_thresholds.py`.

## Context
The project's core goal is detecting *when/how* FAST-LIO fails (GPS-denied drone SLAM, no loop closure → drift is uncorrected). We added `/fastlio_health` (incl. HtDH observability eigenvalues) and calibrated drift thresholds (PROGRESS T26). Drift is *caused* by lost observability (`eig_min` collapsing), which is detectable **before** position visibly diverges. This plan turns that signal into a live early-warning + automated mitigation, while respecting the hard safety rule (`CLAUDE.md`): anything that commands the FC is gated and opt-in.

Design reference already in repo: `PROGRESS.yaml` **P13** (logic, thresholds, response, flowchart, safety caveats).

## Detection logic (recap — see P13)
Per `/fastlio_health` scan: `eig_min = data[4]`, `matched_pts = data[0]`. Maintain a rolling median (healthy window).
```
WARN  = eig_min < 0.2×median  OR  eig_min < 40k        # absolute floor
        OR matched < 0.25×median  OR  matched < 300
CRIT  = eig_min < 5k (hard floor)  OR  matched < 150 (starvation)
fire only after K=5 consecutive scans (hysteresis); severity = worst that fired
```
`eig_min` owns CRITICAL; `matched_pts` only escalates WARN. Thresholds are config-specific starting points (calibrated on the tuned config) — recalibrate live.

## Architecture
```
/fastlio_health ──▶ drift_monitor ──┬─▶ /slam/drift_risk   (OK|WARN|CRITICAL + reason + values)
 (~20Hz)                            ├─▶ /slam/arm_interlock (LOCKED|RELEASED)         [advisory]
                                    ├─▶ /mavros/statustext/send  (WARN + interlock banner)
                                    ├─▶ rosnode kill /laserMapping  → roslaunch respawns  [CRITICAL]
                                    └─▶ /mavros/set_mode "LAND"      [CRITICAL, gated OFF by default]
```

## Phase 1 — Detection + WARN  (read-only, offline-testable)
**New node** `docker_src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/src/drift_monitor.py` (mirror the `fastlio_mavros_bridge.py` pattern):
- Subscribe `/fastlio_health`; rolling-median deque; OR-logic + hysteresis (all thresholds + K as `~params`).
- Publish `/slam/drift_risk` (start with `std_msgs/String` `"LEVEL reason eig=.. matched=.."`; optional custom msg later).
- On WARN: publish `/mavros/statustext/send` (`mavros_msgs/StatusText`, severity=4 WARNING, text `<50` chars), **throttled** (e.g. 1/2 s) to avoid banner spam.
- Register in `FAST-LIO/CMakeLists.txt` `catkin_install_python(PROGRAMS … src/drift_monitor.py …)`.
- Add to `config/fastlio_to_fc.launch`: `<node pkg="fast_lio" type="drift_monitor.py" name="drift_monitor" launch-prefix="python3">` with threshold params.

## Phase 2 — Restart only `laserMapping`  (read-only toward FC)
- **`config/fastlio_to_fc.launch`:** add `respawn="true"` (and a small `respawn_delay`) to the existing `laserMapping` node. roslaunch then auto-restarts *only* that node on exit, re-reading the same params from the param server; mavros/hesai/bridge untouched.
- **Monitor trigger:** on CRITICAL, `drift_monitor` runs `rosnode kill /laserMapping` (ROS API or subprocess) → respawn handles re-launch. Monitor watches `/fastlio_health` resume + K healthy scans = "restart complete".
- **Optional** `scripts/laserMapping.sh` (start/stop/restart/status/logs) following the MCP `control_node` `<workspace>/scripts/<node>.sh` pattern, for manual/MCP-driven node restarts. (Not required for the auto path; respawn covers it.)
- Note: during the ~1–2 s gap `/Odometry`→`vision_pose` stops; that's acceptable (we're landing) and is exactly what makes the natural arm-interlock work.

## Phase 3 — CRITICAL FC actions  (GATED: `enable_critical_fc`, default OFF; needs sign-off)
- **LAND:** call `/mavros/set_mode` (`mavros_msgs/SetMode`, `custom_mode="LAND"`). Mirror the existing `set_ekf_origin` mavros-native pattern in the launch.
- **Advisory arm-interlock:** publish `/slam/arm_interlock = LOCKED` + STATUSTEXT "FAST-LIO RESTART — DO NOT ARM" from CRITICAL onset; **RELEASED** after K consecutive healthy scans post-restart. Enforcement is advisory + the *natural* EKF prearm (vision drop blocks arming in GPS-denied) — **no FC param writes**.
- Use a **larger hysteresis K for the LAND path** than for WARN (a false CRITICAL would force a mid-flight auto-land).
- Stays disabled until explicitly enabled and validated; until then CRITICAL behaves like a loud WARN (banner + risk topic only).

## Files
| Action | Path |
|---|---|
| **Create** | `docker_src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/src/drift_monitor.py` (gitignored; baked into image) |
| **Modify** | `…/FAST-LIO/CMakeLists.txt` (add drift_monitor.py to `catkin_install_python`) |
| **Modify** | `config/fastlio_to_fc.launch` (add `drift_monitor` node; `respawn="true"` on `laserMapping`) |
| **Optional** | `scripts/laserMapping.sh` (manual/MCP node control) |
| **Reuse** | `scripts/replay/calibrate_thresholds.py` (thresholds), `scripts/replay/*` (offline test), `fastlio_mavros_bridge.py` (node template), `set_ekf_origin` launch block (mavros-native pattern) |

**Persistence:** `drift_monitor.py` lives in gitignored `docker_src/` → must be baked into the image (and pushed to the RevoluteRobotics fork, P10) to survive container recreation, same as the velocity work. `config/` edits are bind-mounted (live on restart).

## Verification
1. **Offline (no FC):** feed `out_tuned.bag`/`out_current.bag` `/fastlio_health` into `drift_monitor` (or extend the replay harness) → confirm WARN/CRITICAL fire on the known **97-scan degenerate stretch** and stay OK elsewhere. Tune K / thresholds.
2. **Live read-only (Phase 1+2, benched, disarmed):** run on the live pipeline; watch `/slam/drift_risk` + GCS banner; manually `rosnode kill /laserMapping` → confirm **respawn restores it (same params, mavros stays connected)** and the interlock topic cycles LOCKED→RELEASED on health recovery.
3. **Phase 3 (only after sign-off):** validate LAND + interlock with **props off / SITL** before any real flight; confirm `enable_critical_fc=false` is a pure no-op (banner+topic only).

## Safety
- Phases 1–2 are read-only toward the FC. Phase 3 is opt-in (`enable_critical_fc`, default OFF) and requires explicit user sign-off per `CLAUDE.md` (no FC mode/arm commands without it).
- No FC parameter writes anywhere (interlock is advisory + natural EKF).
- Open risk to confirm in Phase 3: **LAND uses the just-degraded SLAM for horizontal hold during descent** — verify acceptable, or consider an alternative mode, before enabling.
