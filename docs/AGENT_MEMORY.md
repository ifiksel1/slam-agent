# Shared Claude-agent memory

Claude Code keeps a persistent, file-based memory per project at
`~/.claude/projects/<repo-path-hash>/memory/`. That path is **outside the repo and
machine-local** (`.claude/projects/` is gitignored), so by default what one machine's
agent learns never reaches another. This wires that memory to the repo so **every
machine's Claude shares one memory store**, synced over git.

> Looking for the distilled, human-readable lessons instead? See [`docs/lessons/`](lessons/).
> That's a curated snapshot; *this* is the agent's live, read/write memory.

## How it works

- A dedicated **orphan branch `agent-memory`** holds only the memory `.md` files — no
  code. It's independent of `main` and every feature branch, so syncing memory never
  collides with code work or rides along on a feature PR.
- That branch is checked out as a **git worktree** at `<repo>/.agent-memory/`
  (gitignored on code branches; registered per-machine, not shared).
- Each machine's `~/.claude/projects/<hash>/memory` is a **symlink** to that worktree.
  So the agent reads and writes the shared files transparently; writes show up as
  uncommitted changes on the `agent-memory` branch.
- `scripts/agent_memory.sh sync` commits + pulls + pushes that branch.

```
~/.claude/projects/-home-dev-slam-agent/memory  ─symlink→  <repo>/.agent-memory  ─worktree→  branch: agent-memory ⇄ origin
```

## One-time setup

**First machine (already done here):** created the branch from the existing memory.
```bash
scripts/agent_memory.sh init      # seeds 'agent-memory' from this machine's memory, links it, pushes
```

**Every other machine:** after cloning the repo,
```bash
scripts/agent_memory.sh join      # checks out the shared branch as a worktree + symlinks this machine's memory
```
`join` backs up any pre-existing local memory to `…/memory.local-backup.<timestamp>` before linking.

> Launch Claude from the repo root so the project-path hash matches. If your layout
> differs, point it explicitly: `CLAUDE_MEMORY_LINK=/path/to/.claude/.../memory scripts/agent_memory.sh join`.

## Daily use

```bash
scripts/agent_memory.sh sync      # pull others' updates + commit & push yours
scripts/agent_memory.sh status    # show link + worktree + branch head
```

- **Pull at the start** of a session and **sync after** the agent has learned something.
- Merges are per-file, so concurrent edits on *different* memory files merge cleanly.
  `MEMORY.md` (the index) is the one file two machines might both edit — if it conflicts,
  resolve it like any text merge (keep both machines' new bullet lines).

### Optional: automate the sync with hooks
To pull on session start and push on stop, add to `.claude/settings.json` (or use the
`update-config` skill). Not enabled by default — pushing on every Stop can be chatty:
```jsonc
{
  "hooks": {
    "SessionStart": [{ "hooks": [{ "type": "command", "command": "bash scripts/agent_memory.sh sync || true" }] }],
    "Stop":         [{ "hooks": [{ "type": "command", "command": "bash scripts/agent_memory.sh sync || true" }] }]
  }
}
```

## Reverting a machine
The symlink is reversible: `rm ~/.claude/projects/<hash>/memory` and restore the
`memory.local-backup.<timestamp>` dir (or just `git worktree remove .agent-memory` and
drop the symlink). Nothing else on the machine is touched.
