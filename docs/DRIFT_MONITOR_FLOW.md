
```
================================ DRIFT MONITOR — CONTROL FLOW ================================

  INPUTS
  ------
   /mavros/state ---(on connect)---> [request RC_CHANNELS stream]   (keepalive: keeps CH9 alive)

   /fastlio_health                              /mavros/rc/in (CH9)
   (data0=matched_pts, data4=eig_min)           (pilot switch)
        |                                              |
        v                                              v
  ===== AUTOMATIC DETECTION PATH =====        ===== MANUAL CH9 PATH =====
        |                                              |
        v                                              v
  [baseline warmed up? >= warmup_n] --no--> [append baseline, publish OK/warmup]
        | yes                                          |
        v                                              v
  [health gap > reset_gap?] --yes--> [SLAM restarted:      [CH9 in [low,high] band?]
        | no                          clear baseline +         |         \
        |                             counters, latch OK,      |          `--(left band)--> [re-arm, no fire]
        |                             mark gap seen]           | (entered band)
        v                                                      v
  [evaluate scan vs rolling average]                    [start dwell timer]
        |                                                      |
        v                                                      v
  [crit? / warn?]                                       [held >= ch9_dwell?]  (edge, fires once)
        |                                                      |
        v                                                      | yes
  [update hysteresis counters: n_crit / n_warn / n_ok]         |
        |                                                      |
        +--[n_crit >= k_crit?]--yes--> latch CRITICAL          |
        |                                   |                  |
        +--[n_warn >= k_warn?]--yes--> latch WARN              |
        |        (risk topic only, NO banner)                  |
        |                                                      |
        +--[n_ok >= k_release?]--yes--> latch OK               |
                                            |                  |
              (CRITICAL + enable_drift_land) |                 |
                                            v                  v
                                    +===================================+
                                    |     _execute_restart()            |   <-- SHARED RESPONSE
                                    +===================================+
                                            |
                                            v
                             [interlock LOCKED; reset n_ok; clear gap flag]
                             [MSG 1/4: "restart triggered"]
                                            |
                                            v
                                   [ restart_mode ? ]
                                    /              \
                                 (of)              (land)
                                  |                  |
                                  v                  v
                    [EKF source -> OF (SRC2)   [set_mode LAND]
                     DO_AUX_FUNCTION 90]       [MSG 2/4: "LANDING"]
                    [MSG 2/4: "Switched OF"]           |
                                  \                   /
                                   \                 /
                                    v               v
                          [enable_auto_restart? kill laserMapping + bridge]
                          [roslaunch respawns them]
                          [MSG 3/4: "restart in progress"]
                                          |
                                          v
              . . . later, when fresh SLAM is healthy . . .
              [health gap seen  AND  n_ok >= k_release]
                                          |
                                          v
                          [ restart_mode == of  AND  failover_active ? ]
                                    /                       \
                                 (yes)                      (no)
                                  |                          |
                                  v                          |
                    [EKF source -> SLAM (SRC1)]              |
                    [MSG 4/4: "back to SRC1"]                |
                                  \                         /
                                   v                       v
                          [interlock RELEASED; MSG 4/4: "ready to arm"]

==============================================================================================
```

## Context — concise notes

- **Two triggers, one response.** Automatic (drift CRITICAL) and manual (CH9 hold) both call `_execute_restart()`, so behavior is identical in auto and manual modes.
- **Detection metrics:** `matched_pts` (data[0]) and `eig_min` (data[4]) vs a rolling-average baseline. CRITICAL = `matched<150` **OR** `eig_min<5000`; WARN = softer relative/absolute floors.
- **matched_pts is the real detector** (bag analysis 2026-07-23): fires earliest and most consistently. `eig_min` at its 5000 floor is near dead-weight — matched carries every detection in the recorded flights.
- **eig_min still earns a place:** it catches *geometry degeneracy* (e.g. long featureless corridor — matched looks fine, one axis unobservable), a failure mode absent from current bags. Raising its floor to ~15–20k is safe (no false positives observed) and turns it into a live second detector.
- **Condition number (eig_min/eig_max)** lags matched and can miss events — use only as an offline environment classifier, NOT in the trigger path.
- **Hysteresis (K-scan):** requires `k_crit`/`k_warn` consecutive scans to latch, `k_release` OK scans to clear — prevents single-scan chatter.
- **Re-warm on restart:** a `/fastlio_health` gap > `reset_gap` means laserMapping restarted; baseline + counters reset and OK is latched so the fresh (transiently degenerate) SLAM doesn't instantly re-fire.
- **CH9 is edge-triggered with dwell:** must enter the ~1500 band and HOLD `ch9_dwell` seconds; transit through the band does not fire. Never fires if booted already in-band.
- **`restart_mode` selects the FC action:** `of` = EKF source failover SLAM(SRC1)->OF(SRC2), switch back on recovery — NO flight-mode change; `land` = `set_mode LAND`, no switch-back. Toggle via `switch_restart_mode.sh` or the web UI.
- **RC-stream keepalive:** on each `/mavros/state` connect the node re-requests RC_CHANNELS so `/mavros/rc/in` stays populated and CH9 survives reboots / mavros restarts (FC `SR0_RC_CHAN` alone is not enough).
- **Four GCS banners per event:** (1) restart triggered, (2) mode action (OF/LAND), (3) restart in progress, (4) recovery.
- **Order in `of` mode:** source switches to OF *before* SLAM is killed, so the EKF never loses its position feed.
- **Safety:** the node never arms and (in `of`) never changes flight mode. `arm_interlock` is advisory/telemetry only (no FC param writes). The FC's own EKF prearm enforces the actual arming block while vision drops.

---

## Timing — how long each failure mode takes to act

Health scans arrive at 20 Hz, so **1 scan = 50 ms**. Every `k_*` below is a count of
consecutive scans, which is why the times are exact rather than approximate.

### Latency path (`drift_monitor`, companion)

| Stage | Condition | Confirm | Fires after |
|---|---|---|---|
| WARN | age ≥ `lat_warn_ms` 150 | `lat_k_warn` 15 | **0.75 s** |
| CRITICAL → failover SRC1→SRC2 | age ≥ `lat_crit_ms` 250 | `lat_k_crit` 20 | **1.00 s** |
| Escalate → restart both nodes | still ≥ 250 while on OF | `lat_drain_sec` | **+5.0 s** |
| Terminal LAND | still ≥ 250 once the blank expires | `lat_restart_blank_sec` | **+20.0 s** |
| Recover → back to SRC1 | age < 150 | `lat_k_release` 40 | **2.0 s** |

Worst case from the 250 ms crossing to a commanded LAND: **1.0 + 5.0 + 20.0 = 26.0 s**.
Bench T3 measured CRITICAL at +1.5 s from load-on and the drain escalation at exactly 5.0 s
after failover, which is this table.

The 20 s blank is not a delay for its own sake: a fresh `laserMapping` cold start is itself a
multi-second backlog (9.1 s observed), so any shorter fixed timer would fire LAND during a
normal recovery. It is also the least-evidenced number here — sized on ONE observed drain.

### Drift path (geometry)

| Stage | Condition | Confirm | Fires after |
|---|---|---|---|
| WARN | `eig_min` < 40000 or < 0.2x median; `matched` < 300 or < 0.2x median | `k_warn` 5 | **0.25 s** |
| CRITICAL | `eig_min` < 5000 or `matched` < 150 | `k_crit` 5 | **0.25 s** |
| Recover | back above | `k_release` 10 | **0.5 s** |

Drift is 4x quicker to latch than latency because its thresholds are far below anything a
healthy flight produces (worst observed sub-floor dwell across the bag corpus is 3-4 scans),
whereas 150 ms sits only ~2x above the 67-78 ms fleet nominal.

### Loss of the feed itself

| Stage | Condition | Fires after |
|---|---|---|
| Health watchdog → OF (SRC2) | no `/fastlio_health` at all | `health_timeout_sec` **1.0 s** |
| roslaunch respawn | node exited | `respawn_delay` **2 s** |
| Starvation floor | samples stop; age = last + elapsed | caught by the 1.0 s watchdog |

### Arming gate (`slam_latency_gate.lua`, FC)

| Stage | Condition | Fires after |
|---|---|---|
| Block arming | `SLAMLAT` >= `SLG_MS` 150 | `lat_k_arm_lock` 10 scans = **0.5 s** |
| Release | below 150 | `lat_k_arm_release` 60 scans = **3.0 s** |
| Fail open | no `SLAMLAT` at all | `SLG_TOUT` **3.0 s** |

Deliberately asymmetric: lock fast, release slow.

### FC-native EKF failsafe — this is the ALT_HOLD stage

`FS_EKF_ACTION = 2` (AltHold), `FS_EKF_THRESH = 0.8`. `ekf_check` runs at 10 Hz and needs
`EKF_CHECK_ITERATIONS_MAX` = 10 consecutive bad iterations, so:

| Stage | Condition | Fires after |
|---|---|---|
| EKF failsafe → **ALT_HOLD** | variance > 0.8 | **1.0 s** |
| Clear | variance back under | ~**1.0 s** (fail_count decrements) |

This is not something `drift_monitor` does or knows about. It runs in parallel and, at 1.0 s,
is usually the FIRST thing to act.

## What happens when the OF fallback is ALSO bad

The companion has no optical-flow health signal at all. Nothing in `drift_monitor` subscribes
to flow quality, and `EK3_SRC2_POSXY = 0` (None) means SRC2 has no position source: it flies on
flow VELOCITY (`SRC2_VELXY=5`), rangefinder HEIGHT (`SRC2_POSZ=2`) and compass yaw
(`SRC2_YAW=1`), with position dead-reckoned. So the companion cannot detect this case and will
happily failover into it.

What actually catches it is the flight controller, on its own:

```
  SLAM latency/drift bad ──(companion, 1.0 s)──> SRC1 -> SRC2, still LOITER
                                                    |
                                    OF also bad ----+
                                                    |
                                          EKF variance > 0.8
                                                    |
                                       (FC, 1.0 s, FS_EKF_ACTION=2)
                                                    v
                                                 ALT_HOLD
                                        (attitude + height only;
                                         horizontal is the pilot's)
                                                    |
                            still degraded, blank expired, armed
                                                    |
                              (companion, +20 s, enable_latency_land)
                                                    v
                                                   LAND
```

LOITER is held through the source switch — the failover is a lane change, not a mode change, so
the position controller keeps running on whatever the EKF now believes. ALT_HOLD is the first
point at which the vehicle stops trying to hold a position it can no longer estimate, and it is
reached by EKF variance rather than by any latency or geometry number.

**The remaining hole.** The MTF-01 supplies BOTH the optical flow and the rangefinder. If that
one device dies, SRC2 loses velocity and height together: ALT_HOLD then holds altitude on the
barometer, which swings ~6 m in flight on this airframe, and the LAND stage would descend on it
too. Nothing detects this. Note also that `ekf_check` needs two of three (compass, velocity,
position) to be bad, and on SRC1 the yaw and position innovations are both SLAM-derived -- one
sensor wearing two hats -- so the 1.0 s figure above assumes the variance actually crosses,
which is not guaranteed for a SLAM-only failure.
