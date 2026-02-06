# Known Good Configs

This directory stores complete, validated configuration sets from successful integrations.

Each subdirectory is named by hardware fingerprint and contains the full set of generated files:
- SLAM config YAML
- URDF
- Launch files
- ArduPilot/PX4 parameters
- vision_to_mavros config

## How it works

1. After Phase 5 testing passes, the SLAM agent saves the full config set here
2. On new sessions, if the hardware fingerprint matches, the agent offers to reuse these configs
3. User can accept (skip to Phase 4 install) or regenerate from scratch

## Naming convention

```
known_good_configs/
├── jetson_orin-ouster_os1_64-fast_lio2-ardupilot-humble/
│   ├── slam_config.yaml
│   ├── robot.urdf
│   ├── slam_integration.launch
│   ├── ardupilot_params.param
│   └── metadata.yaml      # date, user notes, test results
├── nuc12-livox_mid360-lio_sam-px4-noetic/
│   └── ...
```
