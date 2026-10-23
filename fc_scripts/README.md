# fc_scripts/

Lua that runs **on the flight controller**, not on the companion. These are copied to the
FC's SD card under `APM/scripts/` and loaded by ArduPilot's scripting engine at boot.

| Script | Purpose |
|---|---|
| `slam_latency_gate.lua` | Arming gate. Reads the `SLAMLAT` NAMED_VALUE_FLOAT the companion publishes and blocks arming at or above `SLG_MS` (150 ms). Fails **open** after `SLG_TOUT` (3 s) of silence, so a dead companion returns the vehicle to its pre-gate behaviour rather than stranding it. |
| `land_speed_schedule.lua` | Two-stage LAND descent: `LSS_HI_SPD` (50 cm/s) until the downward rangefinder reads below `LSS_LO_ALT` (50 cm), then `LSS_LO_SPD` (25 cm/s). Stock 4.6.3 cannot do this: it floors `LAND_ALT_LOW` at 100 cm and ramps rather than steps. Writes `LAND_SPEED`/`LAND_SPEED_HIGH` to RAM only while armed in LAND, and restores the saved values the moment LAND ends. `LSS_ENABLE=0` is a live kill switch. Test: `scripts/replay/test_land_speed_schedule.py`. |
| `land_speed_schedule_qr.lua` | **QuadRover** version of the above (Rover-4.7.1 QuadRover firmware): the same `LSS_` params and 50 / 25 cm/s at 50 cm, plus a final stage (owner 2026-10-23, the touchdown bounce): `LSS_FI_SPD` 10 cm/s below `LSS_FI_ALT` 25 cm, acting in QRLand (mode 21) by holding `Q_LAND_ALT` at 1000 m and stepping `Q_LAND_SPD` (m/s) in RAM; restores both when QRLand ends (QuadRover switches to Manual on touchdown). Uses 4.7's `distance_orient` (metres). A Lua error restores the saved params and stops the script. **Install one of the two LSS scripts, never both** (same `LSS_` table). SITL-tested 2026-10-08 on QuadRover: 0.50 m/s above 0.5 m, latched at 49 cm, touchdown at 0.23 m/s, params restored. Since 2026-10-23 (owner, the touchdown bounce): every QuadRover landing (QRLand, QRRTL, Auto, Guided), and two more stages: `LSS_FI_SPD` 15 cm/s below `LSS_FI_ALT` 25 cm, then `LSS_TD_SPD` 50 cm/s below `LSS_TD_ALT` 13 cm (just above the robot's 10-12 cm resting reading; 0 = off). The touchdown stage makes the controller drop the thrust at contact, as a pilot's held-down stick does, instead of rocking on the gear with near-hover thrust (FC logs 82-84). The table is now 8 params: reboot after installing. |
| `landing_gain_reduction_qr.lua` | **QuadRover**: softer roll/pitch control in the last centimetres of every landing. Descending through `LGR_ALT` (20 cm) on the downward rangefinder, it ramps the gains Copter's landed gain reduction scales (roll/pitch rate P and D, angle P) to `LGR_MULT` (50%) in RAM, over Copter's 2 × `Q_A_INPUT_TC`, and back once the landing is declared, on climbing above 35 cm, a mode change or disarm. Never on takeoff. For the touchdown bounce (FC logs 76–79): Copter's `Q_A_LAND_*_MULT` acts only after the landing is declared, 1–2 s too late. `LGR_ENABLE=0` or a Lua error restores the gains. |
| `loiter_speed_switch_qr.lua` | **QuadRover**: QRLoiter speed from a 3-position switch. Reads the channel with `RCx_OPTION` 300 (Scripting1, `QLS_RC_OPT`) and sets `Q_LT_SPEED_MS` and the pilot lean `Q_LT_ANG_MAX` in RAM: low 0.5 m/s / 5°, mid 1.5 m/s / 10°, high 4.0 m/s / 20° (`QLS_SPD_*`, `QLS_ANG_*`; the lean sets how fast loiter reaches its speed limit, τ = speed / (g·tan lean), kept at 0.6–1.1 s); takes effect at once, also in the air. Keeps the speed while there is no RC input; `QLS_ENABLE=0` or a Lua error restores the saved values. |
| `tank_mode_v1.13_params.lua` | Split-arcade tank mode plus the flight speed select. Drives the tracks in tank mode, and in drone mode scales the pilot's roll and pitch sticks so `TANK_FLY_SLOW_V` (cm/s) becomes the Loiter speed at full stick. Also owns the EKF source set for tank vs drone. |

## PARAM_TABLE_KEY collisions

Every script that adds parameters claims a `PARAM_TABLE_KEY`, and it must be unique across
every script on the vehicle:

| Key | Prefix | Script |
|---|---|---|
| 73 | `TANK_` | `tank_mode_v1.13_params.lua` |
| 74 | `SLG_` | `slam_latency_gate.lua` |
| first free of 93, 117, 151, 177, 197 | `LSS_` | `land_speed_schedule.lua` or `land_speed_schedule_qr.lua`, never both (announced in the boot banner) |
| first free of 101, 127, 143, 171, 203 | `QLS_` | `loiter_speed_switch_qr.lua` (announced in the boot banner) |
| first free of 109, 131, 157, 181, 209 | `LGR_` | `landing_gain_reduction_qr.lua` (announced in the boot banner) |

**Two copies of the tank script on the SD card at once will collide on key 73 and kill
both.** The filename carries the version, so installing a new one means deleting the old
one in the same operation. There is no warning if you forget: both scripts simply stop.

## Installing

Copy to `APM/scripts/` on the FC's SD card, remove any older version of the same script,
and reboot. Verify afterwards by checking the parameters appeared (`TANK_FLY_SLOW_V`,
`SLG_MS`, `LSS_LO_ALT`) and that the GCS shows the boot banner. A GCS caches the parameter list, so
refresh it before concluding a parameter is missing.

Over MAVFTP the CRC ArduPilot reports is **CRC-32 with init 0 and no final xor**, not
zlib's — a byte-identical file mismatches `zlib.crc32` and reads as corruption when nothing
is wrong.

## Aux-auth on Copter 4.6.3

`slam_latency_gate.lua` claims an aux-auth slot, and 4.6.3 never reclaims them: every
`MAV_CMD_SCRIPTING` restart leaks one permanently, and `aux_auth_count_max` is 3. Test
these scripts by **rebooting**, never by restarting scripting. The gate checks `SLG_ENABLE`
before requesting a slot, because asking when the pool is full sets `aux_auth_error` and
fails prearm for the whole vehicle.
