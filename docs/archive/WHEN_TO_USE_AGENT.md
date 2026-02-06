# When Should Claude Use "The Agent"?

**Quick Answer**: The agent is a **Cursor-specific convenience wrapper**. The real logic is in the guide files themselves.

---

## 🎯 What is "The Agent"?

The **SLAM Integration Agent** (`.cursor/agents/slam_integration_agent.md`) is a Cursor IDE-specific configuration that:

1. **Tells Cursor's AI** to automatically follow the SLAM integration guide
2. **Pre-configures** the AI with the workflow, phase structure, and best practices
3. **Auto-loads** the guide files when you select the agent
4. **Specializes** the AI for SLAM integration tasks

**Key Point**: The agent file is just a **wrapper/configuration**. All the actual instructions are in:
- `docs/AI_SYSTEM_BUILDER_GUIDE.md` (main guide)
- `docs/AI_GUIDE_INDEX.md` (routing hub)
- `.cursorrules` (workspace rules)

---

## 📍 When to Use the Agent

### ✅ Use the Agent When:

1. **In Cursor IDE** - You're using Cursor and want specialized SLAM help
   - Select agent: `Ctrl+Shift+P` → "Select AI Agent" → "SLAM Integration Assistant"
   - Agent auto-loads the guide and follows the 8-phase workflow

2. **Starting a new SLAM integration** - Building from scratch
   - Agent guides you through all phases systematically
   - Asks hardware questions, generates configs, installs packages

3. **You want the full workflow** - Complete Phase 1-7 process
   - Agent follows the structured methodology
   - Ensures nothing is skipped

4. **You want automatic guide loading** - Convenience feature
   - Agent tells Cursor to load the guide automatically
   - No need to manually reference files

### ❌ Don't Use the Agent When:

1. **In Claude Code or other editors** - Agent is Cursor-specific
   - Instead: Attach `AI_SYSTEM_BUILDER_GUIDE.md` directly
   - Or reference it with `@AI_SYSTEM_BUILDER_GUIDE.md`

2. **Quick troubleshooting** - You have a specific issue
   - Better: `@AI_SYSTEM_BUILDER_GUIDE.md Phase 8.1 - coordinate frames`
   - Or: `@SLAM_INTEGRATION_DIAGNOSTICS.md`

3. **You just need a config template** - Not building full system
   - Better: `@SLAM_INTEGRATION_TEMPLATE.md`
   - Or: Reference specific sections

4. **You're an expert** - You know what you need
   - Better: Direct file references
   - Or: Manual guide reading

---

## 🔄 Agent vs Direct Guide Usage

### Using the Agent (Cursor Only)

**Workflow**:
```
1. Open Cursor in slam_integration/
2. Select "SLAM Integration Assistant" agent
3. Say: "I want to integrate SLAM"
4. Agent automatically:
   - Loads AI_GUIDE_INDEX.md
   - Loads Phase 1 from AI_SYSTEM_BUILDER_GUIDE.md
   - Follows 8-phase workflow
   - Uses browser tools proactively
   - Generates config files
```

**Pros**:
- ✅ Automatic guide loading
- ✅ Pre-configured workflow
- ✅ Specialized for SLAM integration
- ✅ Follows best practices automatically

**Cons**:
- ❌ Cursor-only (doesn't work in Claude Code)
- ❌ Less flexible (follows fixed workflow)
- ❌ May be overkill for simple questions

### Using the Guide Directly (Any Editor)

**Workflow**:
```
1. Attach/reference AI_SYSTEM_BUILDER_GUIDE.md
2. Say: "Follow Phase 1 of the guide"
3. Or: "@AI_SYSTEM_BUILDER_GUIDE.md Phase 8.1 - my drone moves backward"
4. AI reads specific sections as needed
```

**Pros**:
- ✅ Works in any editor (Claude Code, VS Code, etc.)
- ✅ More flexible (read only what you need)
- ✅ Can jump to specific phases/issues
- ✅ Same content, just manual loading

**Cons**:
- ❌ Need to manually reference files
- ❌ Need to specify which phase/section
- ❌ Less automatic workflow

---

## 🎓 Decision Tree

```
Are you in Cursor IDE?
├─ YES → Use the Agent (convenient, auto-loads guide)
│   └─ Starting new integration? → Agent is perfect
│   └─ Quick question? → Still use agent, or @ mention specific section
│
└─ NO (Claude Code, VS Code, etc.) → Use Guide Directly
    └─ Attach AI_SYSTEM_BUILDER_GUIDE.md
    └─ Reference specific sections as needed
```

---

## 💡 Best Practices

### For New Users (Cursor)
1. **Start with the agent** - It guides you through everything
2. **Let it ask all Phase 1 questions** - Don't skip
3. **Follow the workflow** - Phase 1 → 2 → 3 → 4 → 5
4. **Save progress YAML** after each phase

### For Experienced Users (Cursor)
1. **Use agent for new integrations** - Still helpful for systematic approach
2. **Use @ mentions for quick questions** - `@AI_SYSTEM_BUILDER_GUIDE.md Phase 8.4`
3. **Skip to relevant phase** - "I'm at Phase 4, help me install MAVROS"

### For Claude Code Users
1. **Attach the guide files** - `AI_GUIDE_INDEX.md` + `AI_SYSTEM_BUILDER_GUIDE.md`
2. **Be explicit** - "Follow Phase 1 of the guide"
3. **Reference specific sections** - "Read lines 3455-3525 for workspace setup"
4. **Can't use agent** - But guide works the same way

---

## 🔍 What the Agent Actually Does

The agent file (`.cursor/agents/slam_integration_agent.md`) is essentially:

```markdown
# Instructions to Cursor's AI:

1. When user selects this agent:
   - Load AI_GUIDE_INDEX.md
   - Load AI_SYSTEM_BUILDER_GUIDE.md (selectively by phase)
   - Follow the 8-phase workflow
   - Use browser tools proactively
   - Generate config files with real values
   - Offer installation scripts at Phase 4

2. Workflow:
   - Phase 1: Ask hardware questions
   - Phase 2: Validate compatibility
   - Phase 3: Generate files
   - Phase 4: Install packages (offer scripts)
   - Phase 5: Testing
   - Phase 8: Troubleshooting (on-demand)

3. Best practices:
   - Don't use placeholders
   - Check what's installed first
   - Generate progress YAML
   - Use browser tools for specs
```

**It's just a configuration file** that tells Cursor's AI how to use the guide!

---

## 📊 Comparison Table

| Scenario | Use Agent? | Why |
|----------|-----------|-----|
| **Cursor + New Integration** | ✅ Yes | Auto-loads guide, follows workflow |
| **Cursor + Quick Question** | ⚠️ Maybe | Agent works, but @ mention is faster |
| **Cursor + Troubleshooting** | ⚠️ Maybe | Agent can help, but Phase 8 direct is faster |
| **Claude Code + Any Task** | ❌ No | Agent is Cursor-only, use guide directly |
| **VS Code + Any Task** | ❌ No | Agent is Cursor-only, use guide directly |
| **Expert User** | ⚠️ Maybe | Agent still helpful for systematic approach |

---

## 🎯 Summary

**The Agent is**:
- A Cursor-specific convenience wrapper
- Pre-configures AI to follow the guide
- Auto-loads documentation
- Specializes AI for SLAM integration

**The Guide is**:
- The actual source of truth
- Works in any editor
- Contains all instructions
- Can be used directly

**When in Cursor**: Use the agent for convenience  
**When in other editors**: Use the guide directly  
**For quick questions**: @ mention specific sections  
**For full integration**: Agent (Cursor) or guide (any editor)

---

## 🚀 Quick Reference

**In Cursor**:
- New integration → Select "SLAM Integration Assistant" agent
- Quick question → `@AI_SYSTEM_BUILDER_GUIDE.md Phase 8.1`
- Troubleshooting → `@SLAM_INTEGRATION_DIAGNOSTICS.md`

**In Claude Code**:
- New integration → Attach guide files, say "Follow Phase 1"
- Quick question → Attach guide, reference specific section
- Troubleshooting → Attach diagnostics file

**The agent and guide contain the same information** - the agent just makes it easier to use in Cursor!
