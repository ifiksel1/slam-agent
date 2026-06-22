#!/bin/bash
# Shared Claude-agent memory for the slam-agent repo.
#
# The agent's persistent memory normally lives OUTSIDE the repo at
#   ~/.claude/projects/<repo-path-hash>/memory/   (gitignored, machine-local)
# so it does NOT travel between machines. This wires that path to a dedicated,
# git-synced 'agent-memory' branch (checked out as a worktree at <repo>/.agent-memory)
# so every machine's Claude reads & writes ONE shared memory store.
#
# The 'agent-memory' branch is an ORPHAN branch holding only the memory .md files —
# it carries no code and is independent of main / feature branches, so syncing memory
# never collides with your code work.
#
# Usage:
#   scripts/agent_memory.sh init    # FIRST machine: create the branch from current memory + link
#   scripts/agent_memory.sh join    # OTHER machines: check out the existing branch + link
#   scripts/agent_memory.sh sync    # pull + commit + push the shared memory (run often)
#   scripts/agent_memory.sh status  # show link + worktree state
set -euo pipefail

# pwd -P resolves symlinks so the hash matches the canonical path Claude Code keys
# its project-memory dir on (this repo may be reached via a symlinked path).
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WT="$REPO/.agent-memory"            # worktree dir (gitignored on code branches)
BRANCH=agent-memory
REMOTE=origin

# This machine's Claude project-memory path is derived from the repo's absolute path
# (Claude Code names the project dir by replacing '/' with '-'). Launch Claude from the
# repo root so this matches. Override with CLAUDE_MEMORY_LINK=/path if your layout differs.
HASH="$(printf '%s' "$REPO" | sed 's#/#-#g')"
LINK="${CLAUDE_MEMORY_LINK:-$HOME/.claude/projects/$HASH/memory}"

link_memory() {
  mkdir -p "$(dirname "$LINK")"
  if [ -L "$LINK" ]; then
    echo "[=] memory already linked: $LINK -> $(readlink "$LINK")"
    return
  fi
  if [ -e "$LINK" ]; then
    local bak="$LINK.local-backup.$(date +%Y%m%d%H%M%S)"
    mv "$LINK" "$bak"
    echo "[+] backed up this machine's local memory -> $bak"
  fi
  ln -s "$WT" "$LINK"
  echo "[+] linked $LINK -> $WT"
}

have_worktree() { git -C "$REPO" worktree list --porcelain 2>/dev/null | grep -qx "worktree $WT"; }

case "${1:-status}" in
  init)
    if git -C "$REPO" show-ref --verify --quiet "refs/heads/$BRANCH"; then
      echo "branch '$BRANCH' already exists — use 'join' instead." >&2; exit 1
    fi
    SRC="$LINK"
    [ -d "$SRC" ] && [ ! -L "$SRC" ] || { echo "no existing local memory dir at $SRC to seed from" >&2; exit 1; }
    # Create a detached worktree, turn it into an orphan branch with ONLY the memory files.
    git -C "$REPO" worktree add --detach "$WT" HEAD
    git -C "$WT" checkout --orphan "$BRANCH"
    git -C "$WT" rm -rf . >/dev/null 2>&1 || true
    cp -a "$SRC"/. "$WT"/
    git -C "$WT" add -A
    git -C "$WT" commit -q -m "memory: seed shared agent-memory from $(hostname)"
    git -C "$WT" push -u "$REMOTE" "$BRANCH"
    # Replace this machine's local memory with a link to the shared worktree.
    mv "$SRC" "$SRC.local-backup.$(date +%Y%m%d%H%M%S)"
    ln -s "$WT" "$LINK"
    echo "[+] initialized shared memory on branch '$BRANCH' and linked $LINK"
    ;;

  join)
    if ! have_worktree; then
      git -C "$REPO" fetch "$REMOTE" "$BRANCH"
      if git -C "$REPO" show-ref --verify --quiet "refs/heads/$BRANCH"; then
        git -C "$REPO" worktree add "$WT" "$BRANCH"
      else
        git -C "$REPO" worktree add -b "$BRANCH" "$WT" "$REMOTE/$BRANCH"
      fi
    fi
    link_memory
    echo "[+] joined shared memory. Run 'scripts/agent_memory.sh sync' to stay current."
    ;;

  sync)
    have_worktree || { echo "no worktree — run 'join' (or 'init') first." >&2; exit 1; }
    git -C "$WT" add -A
    git -C "$WT" diff --cached --quiet || \
      git -C "$WT" commit -q -m "memory: sync $(date -u +%Y-%m-%dT%H:%M:%SZ) from $(hostname)"
    git -C "$WT" pull --no-rebase --no-edit "$REMOTE" "$BRANCH"
    git -C "$WT" push "$REMOTE" "$BRANCH"
    echo "[+] memory synced ($BRANCH)."
    ;;

  status)
    echo "repo:        $REPO"
    echo "worktree:    $WT  ($(have_worktree && echo present || echo MISSING))"
    echo "link:        $LINK"
    [ -L "$LINK" ] && echo "  -> $(readlink "$LINK")" || echo "  (not a symlink — not shared yet)"
    have_worktree && { echo "branch head:"; git -C "$WT" log --oneline -1 2>/dev/null || true; }
    ;;

  *) echo "usage: $0 {init|join|sync|status}" >&2; exit 1 ;;
esac
