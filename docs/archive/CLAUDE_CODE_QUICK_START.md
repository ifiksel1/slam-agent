# Quick Start: Using SLAM Integration with Claude Code

**Quick reference for using this guide in Claude Code (or any AI editor)**

---

## 🚀 3-Step Setup

### Step 1: Attach Files

**Attach these files to Claude Code**:
1. `docs/AI_GUIDE_INDEX.md` (routing hub - start here!)
2. `docs/AI_SYSTEM_BUILDER_GUIDE.md` (main guide - will be read selectively)

### Step 2: Start Conversation

**Copy-paste this opening message**:

```
I need help integrating SLAM with my drone. I've attached:
- AI_GUIDE_INDEX.md (routing instructions)
- AI_SYSTEM_BUILDER_GUIDE.md (main guide)

Please follow the optimized workflow:
1. Read the index to understand the routing
2. Ask if I have a progress YAML to resume from
3. Load only the phase I need (using offset/limit)
4. Help me through that phase

I'm starting fresh - begin with Phase 1 hardware assessment.
```

### Step 3: Follow Along

- Claude will ask Phase 1 questions about your hardware
- Answer each question
- After Phase 1, Claude will generate `install_config.yaml` automatically
- At Phase 4, Claude will offer to run installation scripts
- Save progress YAML after each phase (ask: "Can I get my progress file?")

---

## 📋 What Claude Will Do

### Phase 1-2: Assessment & Validation
- Ask 10 hardware questions
- Validate compatibility
- Generate `scripts/install_config.yaml` from your answers
- Create progress YAML

### Phase 3: File Generation
- Generate SLAM config files
- Create URDF or static transforms
- Generate launch files
- Create ArduPilot parameters

### Phase 4: Installation ⭐ NEW!
- **Option A**: Claude runs `./install_slam_integration.sh install_config.yaml` automatically
- **Option B**: Claude provides command, you run it
- **Option C**: Manual step-by-step installation

### Phase 5: Testing
- Guide through bench → ground → flight testing
- Run diagnostics
- Verify system

---

## 🛠️ Installation Scripts (NEW!)

**Claude can now automate installation!**

After Phase 1, Claude generates:
```yaml
# scripts/install_config.yaml
ros_version: "ROS1"
ros_distro: "noetic"
workspace_path: "~/catkin_ws"
flight_controller: "ArduPilot"
lidar_type: "Ouster"
slam_algorithm: "FAST-LIO2"
```

At Phase 4, Claude offers:
```
"I can automate the installation. Would you like me to:
A) Run the installation script automatically (recommended)
B) You run it manually: cd scripts && ./install_slam_integration.sh install_config.yaml
C) Follow manual step-by-step installation"
```

**If you choose Option A**:
- Claude runs: `cd scripts && ./install_slam_integration.sh install_config.yaml`
- Monitors output
- Handles errors
- Verifies installation

**If Claude Code can't execute scripts**:
- Claude shows the command
- You copy/paste to terminal
- Report results back
- Claude continues

---

## 💾 Saving Progress

**After each phase, ask Claude**:
```
"Can I get my progress YAML file?"
```

**Save it**:
```bash
# Save to file
~/slam_progress_$(date +%Y%m%d_%H%M%S).yaml
```

**To resume later**:
```
"I want to resume my SLAM integration. Here's my progress file:

[Paste YAML content]

Please continue from Phase X."
```

---

## 🔧 Troubleshooting

### "Claude isn't following the guide"
**Solution**: Be explicit:
```
"Please read AI_SYSTEM_BUILDER_GUIDE.md Phase 1 (lines 60-1505) 
and ask me Q1 about hardware platform."
```

### "Claude can't run scripts"
**Solution**: That's OK! Claude will show commands, you run them:
```
Claude: "Run: cd scripts && ./install_slam_integration.sh install_config.yaml"
You: [Copy, paste to terminal, run]
You: "Script completed successfully"
Claude: "Great! Let's verify installation..."
```

### "File too large to attach"
**Solution**: Use selective reading:
```
"Please read AI_GUIDE_INDEX.md first, then load only Phase 1 
(lines 60-1505) from AI_SYSTEM_BUILDER_GUIDE.md. Don't load 
the entire guide."
```

---

## 📊 Token Usage Tips

**Efficient approach** (recommended):
- Attach `AI_GUIDE_INDEX.md` (small)
- Claude reads phases selectively
- ~420k tokens per session
- ~$2.25 cost (Sonnet 4.5)

**Full guide approach**:
- Attach entire `AI_SYSTEM_BUILDER_GUIDE.md`
- ~910k tokens per session
- ~$3.90 cost
- Slower responses

**Resume approach** (fastest):
- Provide progress YAML
- Skip Phase 1 questions
- ~330k tokens per session
- ~$1.51 cost

---

## ✅ Checklist

- [ ] Attached `AI_GUIDE_INDEX.md` and `AI_SYSTEM_BUILDER_GUIDE.md`
- [ ] Started conversation with clear instructions
- [ ] Answered Phase 1 questions
- [ ] Saved progress YAML after Phase 2
- [ ] Claude generated `install_config.yaml` (check `scripts/` folder)
- [ ] At Phase 4: Chose installation option (A/B/C)
- [ ] Ran installation (Claude or manually)
- [ ] Verified installation completed
- [ ] Continued to Phase 5 testing

---

## 🎯 Key Differences from Cursor

| Feature | Cursor | Claude Code |
|---------|--------|-------------|
| File attachment | Auto-loads | Manual attach |
| Script execution | Can run directly | Shows commands (you run) |
| Progress saving | Auto-saves | Ask for YAML |
| Context | Persistent | Session-based |

**But the workflow is identical!** Same questions, same files, same process.

---

## 📚 More Information

- **Full guide**: `USING_WITH_CLAUDE_CODE.md`
- **Installation scripts**: `scripts/README_INSTALL.md`
- **Progress YAML**: `docs/SAVE_RESUME_FEATURE.md`
- **Optimization**: `docs/OPTIMIZATION_README.md`

---

**Ready to start?** Attach the files and paste the opening message above! 🚀
