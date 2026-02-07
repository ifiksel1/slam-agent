# Curated Hardware Profiles

Starter profiles for common SLAM + UAV hardware combinations. These are searched alongside `docs/learned/hardware_profiles.yaml` by the `search_profiles` MCP tool.

## File Format

Each YAML file is named by its fingerprint: `{platform}-{sensor}-{algorithm}-{autopilot}-{ros_version}.yaml`

```yaml
fingerprint: "jetson_orin-ouster_os1_64-fast_lio2-ardupilot-humble"
date: "2026-02-06"
source: "curated"
description: "Short description of this hardware combination"
hardware:
  platform: {type, model, ram_gb, ...}
  lidar: {model, channels, interface, ...}
  camera: null or {model, type, ...}
  imu_source: lidar | flight_controller | external
  flight_controller: {model, firmware, version}
  communication: mavros | dds
  ros_version: noetic | humble | foxy | iron
  slam_algorithm: name
phase1_config:
  # Full config YAML that Phase 1 would produce
validated: true
integration_complete: true
notes: "Testing context, drift metrics, etc."
```

## Adding Profiles

Add profiles manually or let the SLAM agent auto-populate `docs/learned/hardware_profiles.yaml` through integration sessions. Particularly well-validated profiles can be promoted to this directory.
