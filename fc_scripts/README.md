# fc_scripts/

Lua that runs **on the flight controller**, not on the companion. These are copied to the
FC's SD card under `APM/scripts/` and loaded by ArduPilot's scripting engine at boot.

| Script | Purpose |
|---|---|
| `slam_latency_gate.lua` | Arming gate. Reads the `SLAMLAT` NAMED_VALUE_FLOAT the companion publishes and blocks arming at or above `SLG_MS` (150 ms). Fails **open** after `SLG_TOUT` (3 s) of silence, so a dead companion returns the vehicle to its pre-gate behaviour rather than stranding it. |
| `tank_mode_v1.13_params.lua` | Split-arcade tank mode plus the flight speed select. Drives the tracks in tank mode, and in drone mode scales the pilot's roll and pitch sticks so `TANK_FLY_SLOW_V` (cm/s) becomes the Loiter speed at full stick. Also owns the EKF source set for tank vs drone. |

## PARAM_TABLE_KEY collisions

Every script that adds parameters claims a `PARAM_TABLE_KEY`, and it must be unique across
every script on the vehicle:

| Key | Prefix | Script |
|---|---|---|
| 73 | `TANK_` | `tank_mode_v1.13_params.lua` |
| 74 | `SLG_` | `slam_latency_gate.lua` |

**Two copies of the tank script on the SD card at once will collide on key 73 and kill
both.** The filename carries the version, so installing a new one means deleting the old
one in the same operation. There is no warning if you forget: both scripts simply stop.

## Installing

Copy to `APM/scripts/` on the FC's SD card, remove any older version of the same script,
and reboot. Verify afterwards by checking the parameters appeared (`TANK_FLY_SLOW_V`,
`SLG_MS`) and that the GCS shows the boot banner. A GCS caches the parameter list, so
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
