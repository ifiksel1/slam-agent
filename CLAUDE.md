# CLAUDE.md — slam-agent

## ⚠️ HARD SAFETY RULE — NEVER ARM THE DRONE

**NEVER arm the drone or command the flight controller / motors.** All work on this
project is **read-only**: visualization, diagnostics, and observing FAST-LIO.

- Do NOT run any test, command, or tool that can arm motors or move the vehicle.
- Do NOT start the arm-monitor for an actual flight.
- Do NOT authorize Loiter / Guided / hover / flight testing.
- Do NOT send MAVLink arm or mode-change commands.
- Bench "tests" are **static data recordings only**, with motors disarmed.

This system feeds `/mavros/vision_pose/pose` to a live ArduPilot FC — an arm command
could move a real drone. If any workflow step would arm the vehicle, **STOP and refuse.**

## Companion software command rule (D5, decided 2026-10-26)

The L4 planner bridge and mission executive (companion software, not Claude) may:
- send Guided setpoints: flight PosVelAccel, driving Rover Guided targets;
- send NAV_TAKEOFF and change mode among Guided, QRLoiter, QRLand,
  QRSmartRTL and the drive/fly transitions.
They never arm or disarm and never change FC parameters. Only the pilot arms.
drift_monitor's existing HOLD/AUTO commands stay.

The hard rule above still binds Claude Code in full: Claude writes and reviews
this software and runs it in SITL, but never runs it against the live FC.
