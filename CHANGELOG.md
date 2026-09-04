# Changelog — slam-agent

All notable changes made during SLAM/FC diagnostic + tooling work. Newest first.
Persistence key: **[img]** baked into `slam-system:latest` (survives container recreate) ·
**[cfg]** in bind-mounted `./config` (live on pipeline restart) · **[fc]** FC EEPROM param ·
**[host]** host OS (fstab, etc.).

---

## 2026-07-08

### Manhole-entrance altitude excursions — FIXED & VALIDATED [fc]
Applied the P20 patch on the FC: **`SURFTRAK_MODE = 0`**. Re-flew the manhole — **the fix worked**: the altitude target no longer drops/climbs at the opening, the vision-only vertical loop holds in Loiter, and the drone can hold at / descend into the shaft. Confirms the surface-tracking-off-glitchy-rangefinder root cause from 2026-07-02. `SURFTRAK_MODE=0` alone was sufficient; `EK3_SRC1_POSZ=6` (vision) unchanged. `WPNAV_RFND_USE=0` remains recommended belt-and-suspenders for nav modes. P20 → RESOLVED.

---

## 2026-07-02 (later)

### Manhole-entrance altitude excursions — ROOT-CAUSED: surface tracking off glitchy rangefinder [fc — applied 2026-07-08]
Investigation of `manhole_flight` (log 97) altitude drop/climb at the opening. **Root cause: ArduPilot surface tracking driving the altitude target off the downward rangefinder, which reads garbage over the open shaft** — NOT SLAM, NOT PID, NOT the EKF height source.
- **Ruled out** (with data): SLAM/vision (accurate, tiny EKF innovations, smooth — no jumps); compute/rate (held 20 Hz); vibration (VibeZ 15.6, low); EKF innovation rejection (0%); IMU lane switches (0); attitude-thrust coupling (tilt 2–9°); thrust saturation; battery sag; throttle-stick trim (calibrated, and centered stick coincided with commanded descent); inner-loop PID (tracked target to ±4 cm/s — healthy).
- **Confirmed cause:** `SURFTRAK_MODE=1` + `WPNAV_RFND_USE=1` were enabled (a *separate* consumer of the rangefinder from the EKF; `EK3_SRC1_POSZ=6`/`EK3_RNG_USE_HGT=-1` correctly kept the EKF on vision). Over the manhole the rangefinder glitched (status flickered NoData/OutOfRange/Good, 0–5 m while hovering at 4.4 m). Surface tracking kept dis/re-engaging and latched garbage terrain targets. Proof: `CTUN.DSAlt` (ST target) drove `CTUN.DAlt` (altitude target) — DAlt converged to DSAlt at both drops (4.4→3.0, then 3.5→2.0), ST engaged 79% of the first drop. Loiter-specific → explains why Stabilize flew into the manhole fine (no surface tracking in Stabilize).
- **PATCH (apply on FC, then re-fly Loiter to validate):**
  - `SURFTRAK_MODE = 0`  (stop rangefinder from moving the altitude target)
  - `WPNAV_RFND_USE = 0` (don't use rangefinder for terrain in nav modes)
  - keep `EK3_SRC1_POSZ = 6` (vision) — already correct.
- Status: **APPLIED + VALIDATED 2026-07-08** — `SURFTRAK_MODE=0` set on FC, manhole re-flown, altitude holds (see 2026-07-08 entry above).

## 2026-07-02

### FAST-LIO tuning — reduce compute load for the big/dense facility [cfg]
`config/fastlio_to_fc.launch`. Facility flight ran `/Odometry` at 17 Hz (ekf_update_ms mean 40 / **peak 83 ms** vs 50 ms budget, ~13.4k downsampled pts — ~2× the office). Relaxed the confined-space tuning to cut per-scan cost:
- `point_filter_num` **2 → 3** (fewer input points)
- `max_iteration` **4 → 3** (fewer IEKF iterations)
- `filter_size_surf` **0.25 → 0.35** (coarser scan voxel)
- `filter_size_map` **0.25 → 0.35** (coarser map voxel)
- **Status:** applied + pipeline restarted; `/Odometry` 20.6 Hz on bench. **Needs facility re-test to validate** (bench is too sparse to reproduce the load).
- Earlier same-day step: `point_filter_num` **1 → 2** (first attempt; insufficient in the facility).

### `flight_recorder` — NEW arm-triggered diagnostic recorder [img][cfg]
- `docker_src/.../FAST-LIO/src/flight_recorder.py` (new node) — records the 29-topic Tier1+2 diagnostic set **only while ARMED** (reads `/mavros/state.armed`; start on arm, stop on disarm). lz4, 2-min splits, to `/mnt/usb/MM_DD_YYYY_HH_mm_ss_flight/`. Read-only toward FC (never arms/commands). Thumbdrive-marker guard.
- `docker_src/.../FAST-LIO/CMakeLists.txt` — registered in `catkin_install_python`.
- `config/fastlio_to_fc.launch` — added as node #10 (`trigger=arm`).
- Validated via fake-`/mavros/state` test (no arming) + a real 105 s flight capture.
- **Known open item:** 4 topics not streaming from FC on ground/flight — `/mavros/altitude`, `/mavros/estimator_status`, `/mavros/setpoint_raw/target_local`, `/mavros/extended_state` (FC/plugin, not stream-rate).

### Image rebuild [img]
- Rebuilt `slam-system:latest` (= `5e076c1628b9`) to bake `flight_recorder.py` + the ch9_logger banners. Verified present + rosrun-able in the image.

---

## 2026-07-01

### `ch9_logger` — free-space GCS banners [img][cfg]
- `docker_src/.../FAST-LIO/src/ch9_logger.py` — start/stop GCS messages now include disk free space: `"recording started/finished, X/Y GB (Z% free)"` (added `_freestr()` via `shutil.disk_usage`). Validated live (real CH9 toggle) + fake trigger.

### Persistent thumbdrive mount for auto-recording [host]
- Standardized the recording drive to the connected Extreme Pro (**exfat, UUID `4667-2F38`**).
- `/etc/fstab`: removed old ext4 stick entry (`bd2f03a0…`); added
  `UUID=4667-2F38 /mnt/thumbdrive exfat defaults,nofail,uid=1000,gid=1000 0 0` → auto-mounts at boot before the container.
- Created `.slam_thumbdrive` marker on the drive (both loggers' guard).
- Note: container bind `/mnt/thumbdrive → /mnt/usb` is `rprivate`, so the drive must be mounted *before* container start (reboot handles ordering; live change needs container recreate).

### FC EKF source params → ExternalNav [fc]
- Set `EK3_SRC1_POSXY / VELXY / POSZ / VELZ / YAW` all to **6** (ExternalNav / MAVLink), written to FC EEPROM + verified. (Note: `YAW=6` overrides the compass-fallback setup — flagged.)

### PROGRESS.yaml
- P14 marked resolved (reboot brings pipeline up cleanly from systemd + baked image).
- Added P15 (ch9_logger), P16 (flight_recorder), P17 (thumbdrive persistence), P18 (Loiter altitude investigation).

---

## Diagnostics performed (no code change — reference)
- **Loiter altitude instability (facility flight):** root cause = degraded/jittery SLAM update rate to FC (`/Odometry`→`vision_pose`→`VISP` at 17.5 Hz, 2× jitter, from FAST-LIO frame-starvation in the dense facility) + elevated vibration peaks (VibeZ 36.6, clipping) → EKF rejected 26% of velocity innovations + 4 IMU-lane switches → climb-rate estimate decoupled from vision → unstable altitude. Config identical to the good flight; vibration alone did not explain it (weak time correlation).
- **Good flight (2026-07-01, office) vs bad flight (facility):** confirmed corresponding bag for FC log `94…` = `07_01_2026_20_08_26_flight` via altitude-trace cross-correlation (1.000). Office: 20 Hz vision, 0% innovation rejection, altitude tracked to 7 cm. Facility: 17 Hz, 26% rejection, 5.7 m divergence.
- Offline replay of `06_25_2026_20_23_40.bag` through tuned config: odometry clean, no drift; observability collapse only in the last ~5 s (feature-starvation precursor).
