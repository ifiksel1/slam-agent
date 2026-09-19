# Claude Agent Configuration for SLAM Integration

## Quick Start

### Claude Code
```
"I need help integrating SLAM with my drone.
Read docs/COORDINATOR.md for the workflow, then start with Phase 1."
```

### Claude Desktop / Claude.ai
1. Copy content from `.claude/slam_integration_agent.md` into Custom Instructions
2. Start conversation: "Help me integrate SLAM with my drone"

### Claude API
```python
import anthropic
client = anthropic.Anthropic()

with open('.claude/slam_integration_agent.md', 'r') as f:
    system_prompt = f.read()

# Fable 5.1 is the orchestrator tier (see "Model Routing" below).
# Reasoning depth is set by output_config.effort (low|medium|high|xhigh|max),
# not by a thinking budget: `budget_tokens` and an explicit `thinking` param
# both return 400 on this model. Use xhigh for coding and agentic work.
response = client.beta.messages.create(
    model="claude-fable-5-1",
    max_tokens=16000,
    output_config={"effort": "xhigh"},
    betas=["server-side-fallback-2026-07-01"],
    fallbacks="default",          # on a policy decline, retry on a fallback model in-call
    system=system_prompt,
    messages=[{"role": "user", "content": "I want to integrate SLAM with my drone"}]
)

if response.stop_reason == "refusal":
    raise RuntimeError(f"declined: {response.stop_details}")
print(response.content[0].text)
```

## Model Routing

Claude Code reads this from `.claude/agents/` + `.claude/settings.local.json`:

| Role | Model | Defined in |
|---|---|---|
| Orchestrator (main session) | Fable 5.1 (`claude-fable-5-1`) | `.claude/settings.local.json` -> `"model": "fable"` |
| Coding | Opus 5 (`claude-opus-5`) | `.claude/agents/slam-coder.md` |
| Code review | Fable 5.1 | `.claude/agents/code-checker.md` |
| Small / bounded tasks | Sonnet 5 (`claude-sonnet-5`) | `.claude/agents/slam-helper.md` |

The orchestrator delegates implementation to `slam-coder`, then has `code-checker`
review the diff before it is run on hardware or committed. `settings.local.json` is
gitignored, so the model choice is per-machine; the agent definitions are committed.

## How It Works

The agent reads `docs/COORDINATOR.md` which routes it to load one phase file at a time:
- Phase 1: Hardware assessment (3 batched question groups)
- Phase 2: Compatibility validation
- Phase 3: Config/launch/URDF generation
- Phase 4: Installation (automated scripts available)
- Phase 5: Progressive testing
- Phase 6: Operational troubleshooting (SLAM init, vision pose, EKF, drift)
- Phase 7: Optimization & tuning (SLAM params, ArduPilot gains, resources)
- Phase 8: Path planning (waypoint nav, SUPER+ROG-Map, EGO-Planner, Nav2)
  - 8a: Waypoint navigation (simple MAVROS setpoints)
  - 8b: SUPER + ROG-Map + OMMPC (high-speed obstacle avoidance)
  - 8c: EGO-Planner-v2 / FUEL (3D trajectory / exploration)
  - 8d: Nav2 (ROS 2, corridor/tunnel)
- Troubleshooting: Loaded on-demand per issue

See `docs/AGENT_TEAM.md` for the full architecture.

## Files
- `.claude/slam_integration_agent.md` - Thin agent dispatcher (points to COORDINATOR.md)
- `docs/COORDINATOR.md` - Phase routing and rules
- `docs/phases/*.md` - Individual phase instructions (Phase 8 split into 8a-8d sub-files)
- `docs/troubleshooting/*.md` - Per-issue troubleshooting
