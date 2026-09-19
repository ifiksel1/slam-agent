---
name: model-routing-policy
description: Which Claude model does what in the slam-agent repo — Fable orchestrates, Opus codes, Fable checks, Sonnet handles small tasks — and where that is wired.
metadata:
  type: feedback
---

Model topology for slam-agent, set 2026-09-19, replacing the older
"Sonnet 4.5 for code, Haiku for trivial" policy (which was prose only and
named a stale model generation):

- **Orchestrator / main session: Fable 5.1** — `"model": "fable"` in
  `/home/dev/slam-agent/.claude/settings.local.json` (gitignored, personal).
- **Coding: Opus 5** — `.claude/agents/slam-coder.md` (`model: opus`). Non-trivial
  implementation, refactors, estimator patches, anything where a bug costs a
  rebuild or a re-flight.
- **Checking: Fable 5.1** — `.claude/agents/code-checker.md` (`model: fable`),
  read-only adversarial review of slam-coder's diff.
- **Small tasks: Sonnet** — `.claude/agents/slam-helper.md` (`model: sonnet`).
  Lookup, log/CSV triage, bag inspection, mechanical edits.

**Why:** the earlier policy lived only in MEMORY.md as advisory prose, so
"automatic" depended on the assistant remembering to pass a model override each
time. Agent-definition frontmatter is read by the harness, so routing is now
structural. The review step is structural too: the orchestrator *is* Fable, so
it always reads what Opus returns.

**How to apply:** delegate implementation to `slam-coder`, then review with
`code-checker` before running on hardware or committing. Send bounded lookups to
`slam-helper` rather than doing them in the main thread. The agent files are
committed; the model choice is not, so it does not bind anyone else cloning the
repo. Related: [[clean-slate-between-repeated-runs]],
[[ros-node-teardown-comm-truncation]].
