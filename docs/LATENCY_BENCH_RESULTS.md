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
  came from 22 recorded bags, replayed offline.

## Calibration integrity — the fake 45 Hz stream, checked and ruled out

FAST_LIO_SLAM `345806d` removed a timer that published to the **same** `/Odometry` topic at 45 Hz
stamped `ros::Time::now()` rather than `lidar_end_time`. Samples like that read as ~0 ms old, so
had they been present in the recorded bags they would have dragged the latency distribution down
and made every threshold derived from it too optimistic.

Checked across all 22 flight bags, using two independent tells — publish rate, and the
distribution of `bag_receive_time - header.stamp`:

| | Result |
|---|---|
| Pooled samples | 29,895 |
| **Fraction below 20 ms** | **0.00%** |
| Rate, 20 of 22 bags | 19.9–20.3 Hz — one message per scan |
| Median age, healthy bags | 67.7–77.2 ms, matching the 67–78 ms fleet baseline |

Had the timer been running, roughly 69% of samples (45 of every 65) would sit near zero. None do.

Both outliers are already-understood flights, not artefacts:

- `08_25_2026_20_44_47` — 43.4 Hz, p50 **6044 ms**. The cold-start bag. During a cold start
  FAST-LIO burns through a queued backlog faster than 20 Hz while every pose it emits is seconds
  stale. High rate and high age together are the signature of a backlog, and the exact opposite of
  the fake stream, which would show high rate with *zero* age.
- `08_25_2026_20_38_04` — p50 **286 ms**. The crash flight.

**Conclusion: the calibration is sound.** `lat_warn_ms = 150`, `lat_k_warn = 15` and the
p99.9 = 174 ms figure stand as measured.

## Reproducing

Scripts used are inline in the session rather than committed, because both drive the live vehicle
and neither should be runnable by accident. The mechanisms are: SIGSTOP/SIGCONT duty cycling on
`laserMapping` (T3) and on `drift_monitor` (fail-open), with `MAV_CMD_RUN_PREARM_CHECKS` (401) for
arming state. **Never** use `MAV_CMD_COMPONENT_ARM_DISARM` (400) to check arming on this airframe:
`MOT_SPIN_ARM = 0.10` with `DISARM_DELAY = 0` and `BRD_SAFETY_DEFLT = 0` means a successful arm
spins all four motors indefinitely with no hardware interlock behind it.

---

# Restart cooldown — bench result

Hardware bench, 2026-09-07. Vehicle disarmed, props off. Starvation induced the same
way as the 2026-09-03 event that motivated the fix: an operator's hand held in front of
the lidar, this time for ~90 s continuously.

## The sequence

| t | Event |
|---|---|
| 21:15:34.054 | `WARN -> CRITICAL (matched<150(starv))` eig=5861 **matched=45** |
| 21:15:34.057 | `RESTART (of) triggered by drift:matched<150(starv)` — the one restart |
| 21:15:34.060 | `commanding FC EKF source -> OF (SRC2) (aux 90 pos 1)` |
| 21:15:37.468 | `/fastlio_health resumed after 3.0s gap -> re-warming` |
| 21:15:37.470 | `latency interlock released` |
| 21:15:39.177 | `OK -> CRITICAL (matched<150(starv))` eig=14113 **matched=139** |
| 21:15:39.181 | **`came back 5.1s after the last restart (cooldown 45s) — holding on OF, not restarting again`** |
| 21:15:39 → 21:17:03 | **84.7 s of continuous occlusion. Zero further restarts. One banner.** |
| 21:17:03.928 | `CRITICAL -> OK (ok)` eig=450311 matched=2143 — hand removed |
| 21:17:03.930 | `commanding FC EKF source -> SLAM (SRC1) (aux 90 pos 0)` |
| after | `/slam/arm_interlock` **RELEASED**, `/slam/drift_risk` `OK ... lat=71ms` |

The re-fire came at **5.1 s**, against the 5.2 s loop period measured on 2026-09-03. This
is the same trigger that produced kill #2 that day. It produced none here.

## Three findings

**The cooldown breaks the loop more thoroughly than by declining one restart.** After the
suppression the latch stays CRITICAL, so there is no further OK→CRITICAL *transition* — and
`_execute_restart` only runs on a transition. On 2026-09-03 it was the restart itself that
reset the latch: kill → health gap → re-warm → OK → CRITICAL → kill. Removing the kill
removes the gap that re-armed the trigger. That is why the 45 s cooldown expiring at
21:16:19 produced nothing: by then there was no edge left to fire on. The fix is
self-reinforcing rather than a timer racing a fault.

**`matched=139` on the second trigger, against 26–33 on 2026-09-03.** The restarted process
was doing measurably better and still sat under the 150 floor. That is precisely the case
where another restart buys nothing, and the clearest evidence that restarting is the wrong
response to starvation even when the process is healthy.

**The interlock released.** This was the specific risk in the suppressed path: skipping the
restart means no health gap ever arrives, and the release logic waits on
`_saw_gap_since_lock`. Setting `_lat_no_restart` covers it, and `latency interlock
released` at 21:15:37.470 with `RELEASED` on the topic afterwards confirms it. Had this been
wrong the aircraft would have been left unable to arm with no indication why.

## What this does NOT establish

- Anything armed. `failsafe_ekf_event()` returns immediately when disarmed, so the FC's own
  EKF failsafe was inert here as in every other bench test.
- That 45 s is the right number. It is longer than the observed 5.2 s loop and shorter than a
  sortie; nothing else sets it.
- That a genuine stuck-SLAM case never needs two restarts inside 45 s. If one does, the
  cooldown blocks the second and the OF failover carries it.
