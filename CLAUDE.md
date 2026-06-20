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
