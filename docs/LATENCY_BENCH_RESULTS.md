# SLAM latency detector — bench results

Hardware bench, 2026-09-02. Vehicle disarmed, **props removed**, mRo PixracerPro on Copter 4.6.3.

These are the two tests that had never been run outside offline replay. Everything before them was
either a bag replay, a Lua interpreter on a laptop, or a transient caused by restarting the SLAM
container — none of which proves the chain works on this airframe.

## T3 — sustained overrun (the 25 August reproduction)

**Method.** `laserMapping` duty-cycled with SIGSTOP/SIGCONT, 400 ms stopped / 100 ms running
(~20% speed), for 32 s. This reproduces the actual failure mechanism — per-scan cost exceeding the
50 ms budget at 20 Hz, backlog accumulating — rather than starving the whole machine, which would
also have degraded mavros and the observability the test depends on. Stop intervals are kept under
the 1.0 s health watchdog so the *latency* path is exercised, not the health-loss path.

| t | Event | Reported by |
|---|---|---|
| 6.0 s | load on | — |
| 7.02 s | **WARN** latches, `lat=432ms` | detector |
| 7.07 s | `SLG: SLAM latency 329ms - arming blocked` | FC |
| 7.37 s | `PreArm: SLAM latency 432ms` | FC |
| 7.37 s | `PreArm: VisOdom: not healthy` | FC (independent, `ARMING_CHECK` bit 18) |
| 7.50 s | **CRITICAL** `lat=656ms` → `LATENCY failover -> OF only, no restart` | detector |
| 7.51 s | **`Using EKF Source Set 2`** | **FC confirms** |
| 12.5 s | `latency still 1703 ms after 5.0s on OF -> escalating to restart` | detector |
| 12.5 s | `RESTART (of) triggered by lat:no-drain`, kills laserMapping + bridge | detector |
| 38 s | load off; peak latency **5326 ms** | — |
| 41.26 s | **`Using EKF Source Set 1`** — handed back to SLAM | **FC confirms** |
| 45.37 s | `SLG: SLAM latency 69ms - OK to arm` | FC |

**Passes.** Every stage fired in order, including the two never previously executed against a live
reading: a source switch commanded from real measured latency and confirmed by the FC itself, and
the drain escalation at exactly `lat_drain_sec = 5.0 s`.

### Three findings beyond pass/fail

**The 150 ms bar has adequate margin.** Six seconds of baseline at nominal load held 67–78 ms with
zero WARNs. This was the stated failure condition — if WARN fires on a load the vehicle would have
survived, the threshold is too low — and it did not.

**A restart under sustained load chains into a second failover.** At t≈38 s a second
`LATENCY failover (lat=4798ms(crit))` fired. This is correct, not a defect:
`lat_restart_blank_sec = 20` had expired and the output really was 4.8 s stale, because the load
was still applied. It self-resolved ~3 s after load removal. Worth knowing, because it is not
obvious from reading the code that one restart can lead directly to another failover.

**`ARMING_CHECK` bit 18 reached the same conclusion independently.** `PreArm: VisOdom: not healthy`
appeared alongside the gate's own message — two separate barriers, neither depending on the other.

## Fail-open — the companion dies while the gate is blocking

**Method.** Force a block by lowering `SLG_MS` to 50 against ~70 ms actual latency, then SIGSTOP
`drift_monitor` so `SLAMLAT` stops. SIGSTOP rather than kill, so roslaunch's respawn cannot race
the measurement, and the publisher is guaranteed recoverable.

| t | Event |
|---|---|
| 3.05 s | `SLG_MS=50` → `SLAM latency 69ms - arming blocked` |
| 4.89 s | `PreArm: SLAM latency 69ms` — genuinely blocking |
| 12.28 s | publisher stopped |
| **15.28 s** | **`SLG: no SLAMLAT - latency gate open`** — 3.00 s, exactly `SLG_TOUT` |
| ~19 s | prearm run with the feed dead → **no SLAM-latency failure at all** |
| 24.89 s | feed restored → re-blocks within 0.4 s |
| 34.12 s | `SLG_MS=150` → `OK to arm` |

**Passes.** The decisive line is the prearm at ~19 s: with the companion dead and the gate
previously blocking, prearm reported no SLAM-latency failure. The gate released rather than
latching. A stuck gate strands the aircraft; this one returns the vehicle to its pre-gate
behaviour and says so on the GCS.

The block/release asymmetry also held in both directions — re-blocking 0.4 s after the feed
returned, releasing only after 3 s of good samples.

## What these tests do NOT establish

- That no real flight produces a false CRITICAL. That is what the staged rollout is for:
  `enable_latency_critical=false` for 3–5 sorties, then flip it back.
- That 150 ms is right for flight rather than merely right for the bench. The calibration behind it
  came from 22 recorded bags, and those bags may contain a fake 45 Hz `/Odometry` stream (removed
  upstream in FAST_LIO_SLAM `345806d`) stamped `ros::Time::now()` instead of `lidar_end_time`.
  Such samples read as ~0 ms old and would bias the distribution. **Unverified — check before
  trusting the p99.9 = 174 ms figure.**

## Reproducing

Scripts used are inline in the session rather than committed, because both drive the live vehicle
and neither should be runnable by accident. The mechanisms are: SIGSTOP/SIGCONT duty cycling on
`laserMapping` (T3) and on `drift_monitor` (fail-open), with `MAV_CMD_RUN_PREARM_CHECKS` (401) for
arming state. **Never** use `MAV_CMD_COMPONENT_ARM_DISARM` (400) to check arming on this airframe:
`MOT_SPIN_ARM = 0.10` with `DISARM_DELAY = 0` and `BRD_SAFETY_DEFLT = 0` means a successful arm
spins all four motors indefinitely with no hardware interlock behind it.
