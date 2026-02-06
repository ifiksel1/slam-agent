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

response = client.messages.create(
    model="claude-sonnet-4-5-20250929",
    max_tokens=4096,
    system=system_prompt,
    messages=[{"role": "user", "content": "I want to integrate SLAM with my drone"}]
)
```

## How It Works

The agent reads `docs/COORDINATOR.md` which routes it to load one phase file at a time:
- Phase 1: Hardware assessment (3 batched question groups)
- Phase 2: Compatibility validation
- Phase 3: Config/launch/URDF generation
- Phase 4: Installation (automated scripts available)
- Phase 5: Progressive testing
- Phase 6: Operational troubleshooting (SLAM init, vision pose, EKF, drift)
- Phase 7: Optimization & tuning (SLAM params, ArduPilot gains, resources)
- Troubleshooting: Loaded on-demand per issue

See `docs/AGENT_TEAM.md` for the full architecture.

## Files
- `.claude/slam_integration_agent.md` - Thin agent dispatcher (points to COORDINATOR.md)
- `docs/COORDINATOR.md` - Phase routing and rules
- `docs/phases/*.md` - Individual phase instructions
- `docs/troubleshooting/*.md` - Per-issue troubleshooting
