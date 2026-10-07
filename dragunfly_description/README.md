# dragunfly_description

One xacro, two targets. `sim:=false` emits the bare TF tree that runs on the
aircraft; `sim:=true` emits a Gazebo Harmonic SITL model. Keeping them in one
file is deliberate: a separate sim URDF drifts from the flight URDF, and the
drift shows up as a coordinate-frame bug nobody can reproduce on the bench.

```bash
# Real aircraft
xacro urdf/dragunfly.urdf.xacro sim:=false > /tmp/dragunfly.urdf

# Gazebo Harmonic SITL
ros2 launch dragunfly_description sim_launch.py
sim_vehicle.py -v ArduCopter -f JSON --console --map   # separate terminal
```

## Why Gazebo and not PyBullet/MuJoCo

Only Gazebo has all three of: a real 3D LiDAR sensor model (`gpu_lidar` →
`PointCloud2`), an official ArduPilot SITL bridge (`ardupilot_gazebo`), and a
ROS 2 bridge. PyBullet and MuJoCo have none of them — they are rigid-body/RL
tools. ArduPilot's documented pairing is **ROS 2 Humble + Gazebo Harmonic**,
which matches this repo's stack.

## Sim/real parity

`config/gz_bridge.yaml` maps the simulated sensors onto the **real driver's**
topic names:

| Gazebo topic | ROS topic | Frame |
|---|---|---|
| `/dragunfly/points` | `/ouster/points` | `os_lidar` |
| `/dragunfly/imu` | `/ouster/imu` | `os_imu` |

So FAST-LIO2 config, launch files and ArduPilot params are **identical** in sim
and in flight. If you ever need a sim-only topic name, remap it in the bridge
rather than forking SLAM config.

## Frame ownership

This URDF owns every static frame, `os_lidar` and `os_imu` included. The Ouster
driver must therefore run with **`pub_static_tf: false`** (as the known-good
ROS 2 config already does), or you get conflicting static TFs on `/tf_static`.

`os_lidar` / `os_imu` / `fcu` carry a 1 mg placeholder inertia in sim mode only.
This is not cosmetic: sdformat's URDF→SDF converter merges a *massless*
fixed-joint child into its parent, which would delete the exact frame names the
cloud and IMU are published in.

`base_footprint` exists only when `sim:=false` — a massless root link is an
invalid canonical link in Gazebo.

## Ouster internal extrinsics

Taken from sensor metadata, SN 122224003549, fw 2.4.0 — not guessed:

| Transform | xyz (m) | rpy |
|---|---|---|
| `os_sensor → os_lidar` | `0, 0, 0.03618` | `0, 0, π` |
| `os_sensor → os_imu` | `0.006253, -0.011775, 0.007645` | `0, 0, 0` |

The previous `robot.urdf` had `os_lidar` at `z=0.03675` with **no yaw**, and
`os_imu` at `z=0.03618` with no lateral offset. Both were wrong. A different
OS1 unit has different values: read them from that sensor's own metadata JSON.

## Mounting orientation (resolved)

Per Ouster's firmware manual the two frames point opposite ways along X:

- **`os_lidar` +X points AT the external connector** (encoder angle 0 deg)
- **`os_sensor` +X points AWAY from it** (connector is in `os_sensor` -X)

With `lidar_rpy = "0 ${pi} 0"` (pitch pi) the model resolves to:

| axis | direction in body frame | meaning |
|---|---|---|
| `os_lidar` +X (connector) | `[+1, 0, 0]` | connector **forward** |
| `os_lidar` +Z (sensor top) | `[0, 0, -1]` | **upside down** |

which matches the airframe: inverted, connector forward. `base_link -> os_lidar`
is `[0, 0, -0.09618]` with rotation `Rx(pi)`.

## Propulsion and airframe are derived from prop size

18.5 in props (r = 0.235 m) set the scale of this airframe. Three things follow
from that number and are computed, not typed in:

```
prop_radius     = prop_diameter_in * 0.0254 / 2
motor_constant  = (T/W * all_up_mass * g / 4) / max_rot_velocity^2
moment_constant = moment_arm_ratio * 2 * prop_radius
```

| quantity | value | where it comes from |
|---|---|---|
| `prop_radius` | 0.23495 m | 18.5 in / 2 |
| `arm_length` | 0.400 m | **measured** (800 mm diagonal) |
| `max_rot_velocity` | 520 rad/s (4966 RPM) | 122 m/s tip speed |
| `motor_constant` | 1.2698e-04 | T/W = 2.0 at 7 kg |
| `moment_constant` | 0.0296 m | 0.063 x prop diameter |
| thrust/rotor | 34.34 N (3.50 kgf) | plausible for 18.5 in |
| **T/W** | **2.00** | verified |

**Prop clearance constraint (quad-X).** Adjacent rotor spacing is
`arm_length * sqrt(2)`; props overlap when that drops below `2 * prop_radius`.
The measured 800 mm diagonal gives `arm_length = 0.400 m`, a 0.5657 m spacing
against a 0.4699 m prop diameter, and a **96 mm tip gap (20.4%)** - comfortable.
18.5 in props would only begin to touch at `arm_length = 0.3323 m`.
Verified: closest pair spacing 0.5657 m, tip gap +0.0958 m.

**Why this is not optional.** The upstream ArduPilot Iris constants previously in
this file gave **T/W = 1.12 at 7 kg** - the model would not have lifted off - and
`arm_length = 0.260 m` (the Dragunfly baseline) would have overlapped
18.5 in props by 102 mm. A fixed
`moment_constant` of 0.016 m would also understate yaw authority by ~2x at this
prop size.

`ArduPilotPlugin`'s `multiplier` must equal `max_rot_velocity` or the plugin and
the motor model disagree about full throttle. Both come from the same property,
enforced by the `ap_control` macro. Verified matching at 520.0.

## Open items

Everything below is a placeholder that survived from the Dragunfly baseline or
was derived rather than measured. The model is self-consistent and flies in SITL
as-is; these affect fidelity, not function.

1. **`body_x` / `body_y` = 0.30 m are placeholders.** An 800 mm-diagonal
   airframe almost certainly has a larger centre section. Affects the inertia
   box approximation and the collision hull. This is the last geometry input
   still guessed.
2. **`landing_gear_h = 0.25 m` is plausible, not measured.** Hard minimum is
   ~0.10 m: the inverted Ouster housing reaches z = -0.0956 m, so the previous
   0.10 m left 4.4 mm of ground clearance.
3. **Body inertia is still a box approximation and reads low.** Composite about
   `base_link` is `Ixx = Iyy = 0.0869`, `Izz = 0.1438 kg m^2`. A real 7 kg
   airframe measures nearer `Ixx 0.10-0.20`. Matters for the **attitude loop
   only**: do not carry SITL-tuned `ATC_RAT_*` gains onto the aircraft without
   measuring real inertia (bifilar pendulum). For exercising
   SLAM -> `vision_position_estimate` -> EKF3 and catching frame errors, these
   defaults are fine. Fixing item 1 will raise these numbers toward reality.
4. **Verify the Ouster metadata matches YOUR unit.** Extrinsics here are from
   SN 122224003549, fw 2.4.0. Read them from the sensor's own metadata JSON.
5. **Re-check the vision bridge's "180 deg Y-axis similarity transform."** The
   previous `robot.urdf` was missing the `Rz(pi)` in `os_sensor -> os_lidar` that
   the sensor metadata specifies, which renders the cloud yawed 180 deg. With
   that corrected here, downstream rotation workarounds (including the recent
   "upright Foxglove view" commits) may now be double-correcting.
