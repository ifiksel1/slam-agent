# Map limits: what the JT128 and ROG-Map can and cannot see

From recorded robot scans only (2026-10-27, L1 "Check the map"): the July 15 CAN USA flight bags and the
Hensel Phelps 2026-10-15 flight, 655 scans sampled; maps `hp_1015_carved.pcd` and `canusa_2034.pcd`.
Sensor frame = JT128 (+z is the spin axis = the robot's nose; +y up, +x left, as raw FAST-LIO).
The lidar-to-IMU extrinsic is near identity (T = [-0.008, 0.014, 0.013] m), so the sensor frame is
the body frame to within 1.5 cm. The lidar sits 0.178 m ahead of the FC, level.

## 1. The robot's own points

- Returns that stay put in the sensor frame (the same 2 cm cell in at least 30% of scans): 76 cells,
  **all within 0.23 m of the sensor**, so all inside FAST-LIO's `blind: 0.4`. Main clusters: two either
  side of the sensor (x = +/-0.14..0.22 m, around the nose plane), small ones just above (y +0.13 m)
  and just ahead (z +0.11 m): the mount and covers next to the lidar.
- Beyond 0.4 m nothing is persistent: no cell is hit in 20% of scans or more. At a 10% threshold, 4
  cells 0.76 m straight below the sensor; at 5%, a sheet below it at 0.4-1.0 m. That is the floor
  while the robot sits on the ground (takeoff, landing, parked), not the robot. The props don't show.
- **Proposal: no extra crop box is needed; keep `blind: 0.4`.** If the blind range is ever reduced,
  crop the sensor-frame box x in [-0.25, 0.25], y in [-0.15, 0.20], z in [-0.45, 0.15] m first
  (the robot's body around and behind the lidar; everything behind the sensor is outside the dome anyway).

## 2. What the robot cannot see (body frame)

The dome covers **1.30-94.75 deg from the nose** (128 rings, 900 azimuth samples per ring; ring
spacing median 0.69 deg, 0.14-1.83 deg). So:

- **Behind:** everything more than 94.75 deg from the nose: the half-space behind a plane through the
  lidar, tilted 4.75 deg back. Straight up, down and to the sides are just visible; behind the lidar,
  nothing. The rear ~0.43 m of the robot and everything behind it are unseen.
- **Straight ahead:** a 1.3 deg cone with no ring: 11 cm across at 5 m, 23 cm at 10 m. Narrow, and the
  robot's own motion sweeps it, but a thin pole exactly on the nose line can sit in it.
- **Near field:** nothing within 0.4 m of the lidar (FAST-LIO blind). The lidar is 0.076 m behind the
  front edge (frame 0.508 m long, lidar 0.178 m ahead of centre), so the first **0.32 m ahead of the
  nose** and **0.17 m beyond each side** (half-width 0.23 m) are unseen, as are 0.4 m above and below.
- For the planner: move nose-first (yaw_mode 1); never plan backwards or sideways into space not seen
  earlier from the front; an obstacle that enters the 0.32 m zone ahead of the nose after it was mapped
  stays in the map (it is not cleared by rays that cannot reach it).

## 3. Thin obstacles

Point spacing on a surface at distance d (azimuth 0.40 deg, rings 0.69 deg median) and the expected hits
per scan on 1 m of a thin object crossing the rings:

| distance | azimuth spacing | ring spacing | 2 cm pole: hits/m/scan | 6 mm cable: hits/m/scan |
| --- | --- | --- | --- | --- |
| 2 m | 1.4 cm | 2.4 cm (worst 6 cm) | 59 | 17.8 |
| 5 m | 3.5 cm | 6.0 cm (worst 16 cm) | 10 | 2.9 |
| 10 m | 7.0 cm | 12.0 cm (worst 32 cm) | 2 | 0.7 |

- ROG-Map (`p_hit` 0.9, `p_occ` 0.85, `p_miss` 0.49): **one hit marks a 10 cm cell occupied** (log-odds
  2.20 > 1.73), and a miss only removes 0.04, so a cell needs ~50 misses to clear one hit. A 2 cm pole is
  mapped from well beyond 10 m; a 6 mm cable reliably within ~5 m (several hits per scan at 10 Hz),
  marginally at 10 m (under one hit per metre per scan), and only if it returns at all: the beam
  footprint and a weak return from a thin wire are not modelled here, so cables need a real test.
- The maps already hold thin uprights: 575 (Hensel Phelps) and 216 (CAN USA) columns taller than 1.2 m
  with a footprint of at most 15 cm, down to a single 5 cm cell. The maps are made with deskew
  effectively off (the time-unit bug), which widens thin objects by a few cm and if anything helps.
- **Glass:** cannot be judged from these bags: no glass surface could be identified in them. A
  recording past a glass door or window is needed (JT128 returns from glass depend on angle).

## To test on the robot (L1/L3)

A 2 cm pole and a 6 mm cable at 2, 5 and 10 m, and a glass door, each mapped by ROG-Map from a short
bench recording (motors off): confirms the hit counts and the glass behaviour above.
