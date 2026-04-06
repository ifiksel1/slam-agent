# SLAM Integration Agent Architecture

## Overview

Selective phase loading with MCP tool integration. The agent loads one phase file at a time, uses MCP tools for script execution and knowledge persistence, and routes to sub-files when phases are large.

## Architecture

```
User
  |
  v
SKILL.md (Router, ~250 tokens)
  - Routes to correct phase file
  - Maintains progress YAML between phases
  - Directs MCP tool usage
  |
  |                         MCP Server (slam-tools)
  |                           |-- run_install_script()
  |                           |-- run_diagnostic()
  |                           |-- run_deploy_script()
  |                           |-- control_node()
  |                           |-- inspect_topic()
  |                           |-- search_profiles() / get_profile()
  |                           |-- save_hardware_profile()
  |                           |-- update_profile_status()
  |                           |-- save_known_good_config()
  |                           |-- save_solution() / search_solutions()
  |                           |-- commit_learning() / pull_latest_learning()
  |
  |-- Phase 0: Docker Deployment (optional)
  |     Reads: phase0_docker_deployment.md (~330 lines)
  |     Tools: run_deploy_script, docker_diagnostics
  |     Output: Docker infrastructure
  |
  |-- Phase 1: Hardware Assessment
  |     Reads: phase1_assessment.md (~230 lines)
  |     Tools: search_profiles, save_hardware_profile
  |     Output: hardware_config.yaml
  |
  |-- Phase 2: Compatibility Validation
  |     Reads: phase2_validation.md (~120 lines)
  |     Tools: update_profile_status(validated=true)
  |     Output: install_config.yaml
  |
  |-- Phase 3: Config Generation
  |     Reads: phase3_generation.md (~570 lines)
  |     Ref: SLAM_ARDUPILOT_INTEGRATION_GUIDE.md, SLAM_INTEGRATION_TEMPLATE.md
  |     Output: SLAM config, URDF, launch files, ArduPilot params
  |
  |-- Phase 4: Installation
  |     Reads: phase4_installation.md (~150 lines)
  |     Tools: run_install_script (lidar, camera, SLAM, MAVROS, bridge, planner)
  |     Output: Installed system
  |
  |-- Phase 5: Testing
  |     Reads: phase5_testing.md (~140 lines)
  |     Tools: flight_recorder, flight_analysis, transform_calibrator, arm_monitor
  |     Output: Validated system → save_known_good_config, commit_learning
  |
  |-- Phase 6: Troubleshooting
  |     Reads: phase6_troubleshooting.md (~100 lines) → specific troubleshooting file
  |     Tools: search_solutions (check FIRST), save_solution, commit_learning
  |     Output: Fix applied → return to Phase 5
  |
  |-- Phase 7: Optimization
  |     Reads: phase7_optimization.md (~70 lines)
  |     Output: Tuned SLAM/ArduPilot params
  |
  |-- Phase 8: Path Planning (split into sub-files)
  |     Reads: phase8_path_planner.md (~200 lines, router + shared config)
  |       THEN one of:
  |       |-- phase8a_waypoint_nav.md (~230 lines) — simple MAVROS waypoint following
  |       |-- phase8b_super.md (~360 lines) — SUPER + ROG-Map + OMMPC
  |       |-- phase8c_ego_planner.md (~180 lines) — EGO-Planner-v2 / FUEL + bridge
  |       |-- phase8d_nav2.md (~60 lines) — Nav2 for drones (ROS 2)
  |     Tools: install_path_planner, check_path_planner
  |     Output: Planner + bridge + config
  |
  |-- Phase 9: VOXL/ModalAI (on-demand)
  |     Reads: phase9_voxl.md (~80 lines)
  |     Output: VOXL-specific config
  |
  |-- Troubleshooter (on-demand, per symptom)
        Reads: ONE file from docs/troubleshooting/ (~100-200 lines each)
          coordinate_frames, ros_environment, performance,
          sensor_calibration, vio_specific, visualization_debugging,
          hardware_data_quality, dependencies_flowchart
```

## Learning System

```
Session Start                              After Phase 5 Success
     |                                            |
pull_latest_learning()                   update_profile_status(complete)
     |                                            |
search_profiles(hardware)                save_known_good_config(fingerprint, configs)
     |                                            |
  Match found?                           commit_learning("validated: ...")
  YES → skip to Phase 4/5                        |
  NO  → start Phase 1                    git push → available to all future sessions
```

Data stored in `docs/learned/`:
- `hardware_profiles.yaml` — cached hardware assessments
- `solutions_log.yaml` — troubleshooting solutions (symptom → root cause → fix)
- `known_good_configs/<fingerprint>/` — complete validated config sets

## Context Budget per Phase

| Phase | Lines loaded | Notes |
|-------|-------------|-------|
| 0 | ~330 | Docker setup |
| 1 | ~230 | Hardware questions |
| 2 | ~120 | Compatibility check |
| 3 | ~570 | Largest core phase (config generation) |
| 4 | ~150 | Installation via MCP |
| 5 | ~140 | Testing checklists |
| 6 | ~100 + ~150 | Router + one troubleshooting file |
| 7 | ~70 | Optimization tuning |
| 8 | ~200 + ~60-360 | Router + one planner sub-file |
| 9 | ~80 | VOXL-specific |

Typical session loads 2-3 phases = **400-900 lines** of reference docs, not the full 2500+.

## Data Flow

```
Phase 1 → hardware_config.yaml → Phase 2
Phase 2 → install_config.yaml → Phase 3
Phase 3 → config file paths → Phase 4
Phase 4 → installed system → Phase 5
Phase 5 → test results → Phase 6 (issues) or Phase 7 (working) or Phase 8 (autonomy)
Phase 6 → fix applied → Phase 5 (re-test)
Phase 7 → optimized config → Phase 5 (re-test)
Phase 8 → planner validated → save to learning system → Done
```

## File Map

```
slam-agent/
├── .claude/
│   ├── README.md                          # Quick start
│   ├── slam_integration_agent.md          # Thin agent dispatcher
│   ├── settings.local.json                # MCP permissions whitelist
│   └── skills/slam-integration/
│       └── SKILL.md                       # Main skill (router + MCP tool table)
├── mcp/
│   ├── slam_mcp_server.py                 # MCP server (all tools)
│   ├── run_mcp_server.sh                  # Server launcher
│   └── requirements.txt
├── docs/
│   ├── AGENT_TEAM.md                      # ← This file
│   ├── phases/
│   │   ├── phase0_docker_deployment.md
│   │   ├── phase1_assessment.md
│   │   ├── phase2_validation.md
│   │   ├── phase3_generation.md
│   │   ├── phase4_installation.md
│   │   ├── phase5_testing.md
│   │   ├── phase6_troubleshooting.md
│   │   ├── phase7_optimization.md
│   │   ├── phase8_path_planner.md         # Router + shared config
│   │   ├── phase8a_waypoint_nav.md        # Simple waypoint nav
│   │   ├── phase8b_super.md               # SUPER + ROG-Map + OMMPC
│   │   ├── phase8c_ego_planner.md         # EGO-Planner / FUEL + bridge
│   │   ├── phase8d_nav2.md                # Nav2 (ROS 2)
│   │   └── phase9_voxl.md
│   ├── troubleshooting/                   # Per-symptom files
│   ├── learned/                           # Git-backed knowledge
│   │   ├── hardware_profiles.yaml
│   │   ├── solutions_log.yaml
│   │   └── known_good_configs/
│   └── *.md                               # Reference docs (loaded on demand)
├── scripts/
│   ├── install_*.sh                       # Installation scripts (MCP whitelisted)
│   ├── check_*.py                         # Diagnostic scripts (MCP whitelisted)
│   ├── flight_recorder.sh                 # Flight data recording
│   ├── flight_analysis.py                 # Post-flight HTML dashboard
│   ├── transform_calibrator.py            # LiDAR extrinsic calibration
│   ├── arm_monitor.py                     # Auto-record on arm/disarm
│   └── deploy_docker_slam.sh              # Docker deployment
├── config/                                # Config templates
├── launch/                                # ROS launch files
├── flights/                               # Recorded flight data
└── Dockerfile / docker-compose.yml        # Docker definitions
```
