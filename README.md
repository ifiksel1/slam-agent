# SLAM Integration Package

**Complete GPS-Denied Navigation System for UAVs**

This directory contains everything needed to integrate SLAM algorithms with ArduPilot/PX4 for autonomous UAV navigation in GPS-denied environments.

---

## Directory Structure

```
slam_integration/
├── README.md                              # This file
├── QUICK_START.md                         # One-page quick reference
├── .mcp.json                              # MCP server config (Claude Code + Cursor)
├── .cursorrules                           # Cursor workspace rules
├── .cursor/agents/slam_integration_agent.md   # Cursor agent
├── .claude/
│   ├── slam_integration_agent.md          # Claude agent dispatcher
│   └── skills/slam-integration/           # Claude Code Skill
│       ├── SKILL.md                       #   Skill entrypoint (brain)
│       └── references/                    #   Symlinks to phase files
│
├── mcp/                                   # MCP Server (hands)
│   ├── slam_mcp_server.py                 #   FastMCP server with 13 tools
│   └── requirements.txt                   #   Python dependencies
│
├── docs/
│   ├── COORDINATOR.md                     # Entry point for AI - phase routing & rules
│   ├── AGENT_TEAM.md                      # Agent team architecture documentation
│   │
│   ├── phases/                            # Phase-specific files (load one at a time)
│   │   ├── phase1_assessment.md           #   Hardware assessment (3 batched Q groups)
│   │   ├── phase2_validation.md           #   Compatibility validation
│   │   ├── phase3_generation.md           #   Config/launch/URDF file generation
│   │   ├── phase4_installation.md         #   Workspace setup & package installation
│   │   ├── phase5_testing.md              #   Progressive testing protocol
│   │   ├── phase6_troubleshooting.md      #   Operational troubleshooting (post-integration)
│   │   ├── phase7_optimization.md         #   SLAM/ArduPilot tuning & optimization
│   │   └── phase9_voxl.md                 #   VOXL/ModalAI systems (on-demand)
│   │
│   ├── troubleshooting/                   # On-demand troubleshooting (load per issue)
│   │   ├── coordinate_frames.md           #   Drone moves wrong direction
│   │   ├── ros_environment.md             #   Package/build errors
│   │   ├── performance.md                 #   SLAM too slow, high CPU
│   │   ├── sensor_calibration.md          #   Camera/IMU/LiDAR calibration
│   │   ├── vio_specific.md               #   VIO tracking/drift issues
│   │   ├── visualization_debugging.md    #   RViz, topics, TF, bag files
│   │   ├── hardware_data_quality.md      #   Vibration, mounting, environment
│   │   └── dependencies_flowchart.md     #   Build failures, diagnostic tree
│   │
│   ├── learned/                           # Learning system (auto-updated, git-backed)
│   │   ├── hardware_profiles.yaml        #   Cached Phase 1 configs by hardware
│   │   ├── solutions_log.yaml            #   Resolved issues for future reference
│   │   └── known_good_configs/           #   Validated config sets from successful integrations
│   │
│   ├── profiles/                          # Curated starter hardware profiles
│   │   └── *.yaml                        #   One file per hardware combination
│   │
│   ├── AI_SYSTEM_BUILDER_GUIDE.md         # Original monolithic guide (archive/reference)
│   ├── SLAM_ARDUPILOT_INTEGRATION_GUIDE.md  # Technical integration reference
│   ├── SLAM_INTEGRATION_TEMPLATE.md       # Config templates
│   ├── SLAM_INTEGRATION_DIAGNOSTICS.md    # Systematic diagnostics
│   └── archive/                           # Archived meta-docs (not for AI runtime)
│
├── scripts/                               # Automation & diagnostics (called via MCP)
│   ├── install_slam_integration.sh        #   Main installation orchestrator
│   ├── install_core_ros_packages.sh       #   Core ROS packages
│   ├── install_mavros.sh                  #   MAVROS installer
│   ├── install_dds_bridge.sh              #   DDS/micro-ROS installer
│   ├── install_lidar_driver.sh            #   LiDAR driver installer
│   ├── install_camera_driver.sh           #   Camera driver installer
│   ├── install_slam_algorithm.sh          #   SLAM algorithm installer
│   ├── install_vision_to_mavros.sh        #   Vision bridge installer
│   ├── verify_installation.sh             #   Post-install verification
│   ├── slam_diagnostics.sh               #   Automated health checks
│   ├── check_sensor_time_sync.py          #   Sensor timestamp validator
│   ├── check_tf_tree.py                   #   TF tree validator
│   ├── check_autopilot_params.py          #   Autopilot parameter validator
│   ├── check_topic_pipeline.py            #   ROS topic pipeline validator
│   ├── check_urdf.py                      #   URDF plausibility validator
│   └── analyze_slam_bag.py               #   Post-flight bag analysis
│
└── examples/                              # Example configurations
```

---

## Setup

### Prerequisites
```bash
pip3 install -r mcp/requirements.txt
```

This installs the MCP server dependencies so Claude Code / Cursor can run scripts, manage profiles, and commit learned data automatically.

### Getting Started

**With Claude Code** (recommended):
```
"Help me integrate SLAM with my drone."
```
The `slam-integration` skill activates automatically. MCP tools handle script execution and learning.

**With Cursor IDE**:
Select the `slam_integration_agent` from the agent dropdown. MCP tools are available via `.mcp.json`.

**With Any AI Assistant** (no MCP):
Point the AI to `docs/COORDINATOR.md` as its entry point. Scripts must be run manually.

**Resuming a Previous Session**:
```
"Resume my SLAM integration from this progress file: [paste YAML]"
```

---

## Architecture: Agent Team (Token Optimized)

The system uses a **coordinator + specialized agents** pattern:

| Component | Context Size | Purpose |
|-----------|-------------|---------|
| Coordinator | ~500 tokens | Routes between phases, maintains state |
| Phase 1 Agent | ~2k tokens | Collects hardware info (3 batched groups) |
| Phase 2 Agent | ~1k tokens | Validates compatibility |
| Phase 3 Agent | ~5k tokens | Generates all config files |
| Phase 4 Agent | ~2k tokens | Runs installation |
| Phase 5 Agent | ~1k tokens | Progressive testing |
| Phase 6 Agent | ~1k tokens | Operational troubleshooting |
| Phase 7 Agent | ~1k tokens | Optimization & tuning |
| Troubleshooter | ~1k tokens | Loads per-issue file on demand |

**Token savings**: ~85% reduction vs original monolithic approach (120k vs 910k per session).

### MCP Tools (Script Execution + Learning)

The `slam-tools` MCP server (`mcp/slam_mcp_server.py`) provides 13 tools:

| Category | Tools | Purpose |
|----------|-------|---------|
| Installation | `run_install_script` | Execute install scripts without loading into context |
| Diagnostics | `run_diagnostic` | Run check/validation scripts |
| Profiles | `search_profiles`, `get_profile`, `get_known_good_config` | Find matching hardware configs |
| Learning | `save_hardware_profile`, `update_profile_status`, `save_solution`, `save_known_good_config` | Persist knowledge |
| Git | `commit_learning`, `pull_latest_learning` | Sync learned data across sessions |

### Self-Updating Learning Loop

The agent improves over time by committing knowledge back to git:
- After Phase 5 success: saves validated hardware profiles and configs, pushes to git
- After Phase 6 fix: saves troubleshooting solutions, pushes to git
- At session start: pulls latest profiles and solutions from git

See `docs/AGENT_TEAM.md` for full architecture documentation.

---

## Supported Configurations

**SLAM Algorithms**: FAST-LIO2, LIO-SAM, COIN-LIO, Cartographer, LeGO-LOAM, Point-LIO, ORB-SLAM3, RTAB-Map, OpenVINS, VINS-Fusion, LVI-SAM, R3LIVE, FAST-LIVO2, GLIO, GVINS, and 15+ more

**Autopilots**: ArduPilot 4.0+, PX4 v1.10+

**ROS**: Noetic, Humble, Foxy, Iron

**Platforms**: Jetson (Orin/Xavier/Nano/TX2), Intel NUC, x86, Raspberry Pi 4/5, VOXL (ModalAI)

**Sensors**: Ouster, Velodyne, Livox, Hesai, RoboSense, RealSense, ZED, and more

---

## Key Files for Developers

| File | Purpose |
|------|---------|
| `docs/COORDINATOR.md` | AI entry point - phase routing and rules |
| `docs/AGENT_TEAM.md` | Agent team architecture and token analysis |
| `docs/phases/*.md` | Individual phase instructions |
| `docs/troubleshooting/*.md` | Per-issue troubleshooting guides |
| `scripts/install_slam_integration.sh` | Automated installation orchestrator |
| `docs/SLAM_ARDUPILOT_INTEGRATION_GUIDE.md` | Deep technical reference |
