# SLAM Integration Agent Team Architecture

## Overview

Instead of one monolithic agent loading/unloading 26k tokens of documentation, use specialized sub-agents that each carry only the knowledge they need.

## Architecture

```
User
  |
  v
Coordinator (~500 token prompt)
  - Routes to correct phase agent
  - Maintains progress YAML between phases
  - Passes structured config between agents
  |
  |-- Phase 1 Agent: Hardware Assessor
  |     Input: nothing (fresh start) or progress YAML (resume)
  |     Reads: docs/phases/phase1_assessment.md (~2k tokens)
  |     Does: Asks 3 batched question groups, web searches specs
  |     Output: slam_hardware_config.yaml
  |
  |-- Phase 2 Agent: Compatibility Validator
  |     Input: slam_hardware_config.yaml
  |     Reads: docs/phases/phase2_validation.md (~1k tokens)
  |     Does: Checks compatibility matrix, warns about issues
  |     Output: validated config + install_config.yaml
  |
  |-- Phase 3 Agent: Config Generator
  |     Input: validated config
  |     Reads: docs/phases/phase3_generation.md (~5k tokens)
  |     Does: Generates SLAM config, URDF, launch files, params, Docker
  |     Output: file paths list
  |
  |-- Phase 4 Agent: Installer
  |     Input: install_config.yaml + file paths
  |     Reads: docs/phases/phase4_installation.md (~2k tokens)
  |     Does: Runs install scripts or manual steps, tracks progress
  |     Output: installation_status.yaml
  |
  |-- Phase 5 Agent: Tester
  |     Input: installation_status.yaml
  |     Reads: docs/phases/phase5_testing.md (~1k tokens)
  |     Does: Progressive bench/ground/flight testing checklist
  |     Output: test results
  |
  |-- Phase 6 Agent: Operational Troubleshooter
  |     Input: test results + error descriptions
  |     Reads: docs/phases/phase6_troubleshooting.md (~1k tokens)
  |     Does: Diagnoses SLAM init, vision pose, EKF, drift issues
  |     Output: fix applied, return to testing
  |
  |-- Phase 7 Agent: Optimizer
  |     Input: working system + environment info
  |     Reads: docs/phases/phase7_optimization.md (~1k tokens)
  |     Does: Tunes SLAM params, ArduPilot gains, resource usage
  |     Output: optimized config files
  |
  |-- Troubleshooter Agent (on-demand)
  |     Input: error description + hardware config
  |     Reads: ONE troubleshooting file (~1k tokens each)
  |     Does: Diagnoses and provides fix
  |     Output: fix instructions
  |
  |-- VOXL Agent (on-demand)
        Input: VOXL hardware detection
        Reads: docs/phases/phase9_voxl.md (~2k tokens)
        Does: VOXL-specific validation and setup
        Output: validated VOXL config
```

## Token Comparison

| Metric | Old (monolithic) | New (selective) | New (agent team) |
|--------|-----------------|-----------------|------------------|
| Per-turn context | 8-15k | 3-8k | 1-3k |
| 35-turn session | ~910k | ~420k | ~120-150k |
| Cost (Sonnet) | $3.90 | $2.25 | $0.65-0.80 |

## How to Use with Claude Code

### Option 1: Manual Phase Routing (works today)
```
User: "Help me integrate SLAM with my drone"
Claude: [Reads COORDINATOR.md, loads phase1_assessment.md]
        [Asks batched questions, collects answers]
        [Outputs config YAML]
        [Unloads phase1, loads phase2]
        ...continues through phases...
```

### Option 2: Sub-Agent Delegation (Claude Code with Task tool)
```python
# Coordinator dispatches to specialized agents
Task(
    description="Collect hardware info",
    prompt="Read docs/phases/phase1_assessment.md. Ask the user about their "
           "hardware in 3 batched groups. Output slam_hardware_config.yaml.",
    subagent_type="general-purpose"
)
```

### Option 3: Background Agents (parallel where possible)
```python
# Phase 2 + Phase 3 can partially overlap:
# - Validate compatibility (Phase 2)
# - Start generating configs for confirmed components (Phase 3)
# Phase 4 components can install in parallel:
# - Install MAVROS while cloning SLAM repo
# - Install sensor driver while building SLAM
```

## Coordinator Prompt Template

Use this as the system prompt or initial instruction:

```
You are coordinating a SLAM integration for a GPS-denied autonomous UAV.

Current state: [PHASE_NUMBER] - [PHASE_NAME]
Hardware config: [YAML summary from Phase 1]

Your job:
1. Load the current phase file from docs/phases/
2. Execute that phase's instructions
3. Output structured results
4. Report back when phase is complete

Rules:
- Only read the file for your current phase
- Use web search for hardware lookups
- Generate actual values, not templates
- Report completion with structured output
```

## Resume Flow

```
User provides progress YAML
  |
  v
Coordinator parses YAML
  - Extracts: current_phase, hardware_config, completed_phases
  - Determines: next phase to execute
  |
  v
Loads ONLY next phase file
  - Skips all completed phases (no re-reading)
  - Has hardware config from YAML (no re-asking questions)
  |
  v
Continues from exact stopping point
```

## Data Flow Between Phases

```
Phase 1 --> slam_hardware_config.yaml --> Phase 2
Phase 2 --> validated_config + install_config.yaml --> Phase 3
Phase 3 --> file_paths_list --> Phase 4
Phase 4 --> installation_status --> Phase 5
Phase 5 --> test_results --> Phase 6 (if issues) or Phase 7 (if working)
Phase 6 --> fixes_applied --> Phase 5 (re-test)
Phase 7 --> optimized_config --> Phase 5 (re-test) --> Done
```

Each handoff is a structured YAML document, not free text. This keeps inter-phase communication at ~200 tokens instead of carrying full conversation history.
