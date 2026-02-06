# Using SLAM Integration with Claude Code (or Other AI Editors)

**Version**: 1.0  
**Date**: November 24, 2025  
**Purpose**: Guide for using slam_integration outside of Cursor IDE

---

## 📦 What You Need to Copy

The `slam_integration` folder is **fully portable** and doesn't depend on Cursor. Copy the entire folder to your new computer:

```bash
# On source computer (Cursor):
cd /path/to/Ultra-onboard
tar -czf slam_integration.tar.gz slam_integration/

# Transfer to new computer, then:
tar -xzf slam_integration.tar.gz
```

### Core Files (Required)

These files contain all the documentation and logic:

```
slam_integration/
├── docs/
│   ├── AI_SYSTEM_BUILDER_GUIDE.md          ⭐ Main interactive guide (9971 lines)
│   ├── AI_GUIDE_INDEX.md                   ⭐ Optimized routing hub
│   ├── OPTIMIZATION_README.md              ⭐ Token optimization docs
│   ├── SAVE_RESUME_FEATURE.md              ⭐ Progress YAML guide
│   ├── SLAM_ARDUPILOT_INTEGRATION_GUIDE.md    Technical reference
│   ├── SLAM_INTEGRATION_TEMPLATE.md           Config templates
│   └── SLAM_INTEGRATION_DIAGNOSTICS.md        Troubleshooting
├── scripts/
│   ├── install_slam_integration.sh         ⭐ NEW: Main installation script
│   ├── install_core_ros_packages.sh        ⭐ NEW: Core ROS packages
│   ├── install_mavros.sh                   ⭐ NEW: MAVROS installer
│   ├── install_dds_bridge.sh               ⭐ NEW: DDS/micro-ROS installer
│   ├── install_lidar_driver.sh             ⭐ NEW: LiDAR driver installer
│   ├── install_camera_driver.sh            ⭐ NEW: Camera driver installer
│   ├── install_slam_algorithm.sh           ⭐ NEW: SLAM algorithm installer
│   ├── install_vision_to_mavros.sh         ⭐ NEW: vision_to_mavros installer
│   ├── verify_installation.sh              ⭐ NEW: Installation verifier
│   ├── install_config_template.yaml        ⭐ NEW: Config template
│   ├── README_INSTALL.md                   ⭐ NEW: Installation docs
│   └── slam_diagnostics.sh                    Automated checker
├── README.md                                  Start here!
├── QUICK_START.md                             Quick reference
└── INDEX.md                                   File catalog
```

### Cursor-Specific Files (Optional)

These only work in Cursor but aren't required:

```
slam_integration/
├── .cursor/
│   └── agents/
│       └── slam_integration_agent.md       Custom Cursor agent
└── .cursorrules                            Cursor workspace rules
```

**Note**: The AI logic is IN the documentation files, not the Cursor config. The Cursor files just tell Cursor's AI to read those docs.

---

## 🚀 Using with Claude Code

**⭐ NEW: Installation Scripts Available!**

The guide now includes **automated installation scripts** that Claude Code can use:

- **Main script**: `scripts/install_slam_integration.sh`
- **Component scripts**: Individual installers for each component
- **Config generation**: Claude can generate `install_config.yaml` from Phase 1 answers
- **See**: `scripts/README_INSTALL.md` for full documentation

**How Claude Code uses them**:
1. After Phase 1: Claude generates `install_config.yaml` automatically
2. At Phase 4: Claude offers to run the installation script
3. Claude can execute: `cd scripts && ./install_slam_integration.sh install_config.yaml`
4. Or provide commands for you to run manually

---

### Option 1: Attach the Main Guide Directly

**Simplest approach** - just attach the main guide:

1. **Open Claude Code** on your new computer
2. **Attach** the main guide file:
   ```
   File → Attach → slam_integration/docs/AI_SYSTEM_BUILDER_GUIDE.md
   ```
3. **Start the conversation**:
   ```
   "I need help integrating SLAM with my drone. Please follow the 
   AI_SYSTEM_BUILDER_GUIDE.md to help me through the process.
   
   Start with Phase 1: Ask me the hardware assessment questions."
   ```

**Pros**:
- ✅ Complete guide in one file (5156 lines)
- ✅ All phases included
- ✅ Works immediately

**Cons**:
- ❌ Uses ~910k tokens (entire guide loaded at once)
- ❌ Higher cost per session
- ❌ Slower responses

---

### Option 2: Use Optimized Selective Reading (Recommended)

**More efficient** - use the optimized routing hub:

1. **Attach the index file**:
   ```
   File → Attach → slam_integration/docs/AI_GUIDE_INDEX.md
   ```

2. **Start with routing instructions**:
   ```
   "I need help integrating SLAM with my drone. I've attached AI_GUIDE_INDEX.md 
   which explains how to selectively load phases from AI_SYSTEM_BUILDER_GUIDE.md.
   
   Please follow the optimization workflow:
   1. Read the index to understand the routing
   2. Ask if I have a progress YAML to resume from
   3. Load only the phase I need (using offset/limit)
   4. Help me through that phase
   
   Start with Phase 1 assessment."
   ```

3. **Claude will then**:
   - Read AI_GUIDE_INDEX.md (13 KB)
   - Ask if you're resuming (check for progress YAML)
   - Load ONLY Phase 1 from AI_SYSTEM_BUILDER_GUIDE.md (lines 60-1100)
   - Ask hardware questions
   - Generate progress YAML after each phase
   - Load next phase when ready

**Pros**:
- ✅ 54% token savings (~420k vs 910k)
- ✅ 42% cost savings
- ✅ Faster responses (smaller context)
- ✅ Resume-friendly (via progress YAML)

**Cons**:
- ❌ Requires manual file attachments per phase
- ❌ Claude must support selective file reading

---

### Option 3: Resume from Progress YAML

**If you already started** in Cursor and saved progress:

1. **Find your progress YAML**:
   ```bash
   # On old computer (Cursor):
   # Save when AI generates it during Phase 1/2
   # File will be named: slam_progress_YYYYMMDD.yaml
   ```

2. **Transfer progress file** to new computer

3. **In Claude Code**:
   ```
   "I want to resume my SLAM integration. Here's my progress file:
   
   [Paste or attach slam_progress_YYYYMMDD.yaml]
   
   Please:
   1. Read the progress file
   2. Load the AI guide (AI_GUIDE_INDEX.md + AI_SYSTEM_BUILDER_GUIDE.md)
   3. Continue from where I left off (phase X)
   4. Use my saved hardware configuration"
   ```

4. **Claude will**:
   - Parse your progress YAML
   - See you completed Phase X
   - Load Phase X+1 documentation
   - Continue seamlessly without re-asking Phase 1 questions!

**Pros**:
- ✅ Seamless continuation
- ✅ No repeated questions
- ✅ All your hardware specs saved
- ✅ Works across computers/editors

---

## 📂 File Attachment Guide

### For Complete Session (Option 1)

Attach in this order:
1. `AI_SYSTEM_BUILDER_GUIDE.md` (main guide)

That's it! Everything is in one file.

### For Optimized Session (Option 2)

**Initial attachment**:
1. `AI_GUIDE_INDEX.md` (routing hub)
2. `AI_SYSTEM_BUILDER_GUIDE.md` (will be read selectively)

**Subsequent phases** (as needed):
- Same files, Claude reads different line ranges

### For Resume Session (Option 3)

Attach in this order:
1. `slam_progress_YYYYMMDD.yaml` (your saved progress)
2. `AI_GUIDE_INDEX.md` (routing hub)
3. `AI_SYSTEM_BUILDER_GUIDE.md` (for continuation)

---

## 🔄 Comparison: Cursor vs Claude Code

### What Works the Same

| Feature | Cursor | Claude Code | Notes |
|---------|--------|-------------|-------|
| AI guide workflow | ✅ Yes | ✅ Yes | Same documentation |
| Phase-by-phase process | ✅ Yes | ✅ Yes | Same questions |
| File generation | ✅ Yes | ✅ Yes | Same configs |
| Progress YAML | ✅ Yes | ✅ Yes | Same format |
| Browser tools | ✅ Yes | ⚠️ Maybe | Depends on Claude Code version |
| Resume capability | ✅ Yes | ✅ Yes | Via progress YAML |

### What's Different

| Feature | Cursor | Claude Code | Workaround |
|---------|--------|-------------|------------|
| Custom agent | ✅ Auto-loads guide | ❌ Manual attachment | Attach guide manually |
| Workspace rules | ✅ Auto-applied | ❌ N/A | Instructions in guide |
| Context sharing | ✅ Persistent | ⚠️ Session-based | Save progress YAML often |
| File navigation | ✅ Integrated | ❌ Manual | Specify file paths |
| Terminal commands | ✅ Can run | ⚠️ Shows only | Copy/paste to terminal |

### What You'll Need to Do Manually

**In Cursor**: AI automatically knows to follow the guide  
**In Claude Code**: You tell Claude to follow the guide

**In Cursor**: Progress saved in workspace context  
**In Claude Code**: Save progress YAML to file manually

**In Cursor**: Can run terminal commands directly  
**In Claude Code**: Copy commands and run in your terminal

---

## 💡 Best Practices for Claude Code

### 1. Start Each Session Clearly

Good opening:
```
"I'm working on SLAM integration for my drone. I've attached:
- AI_GUIDE_INDEX.md (routing instructions)
- AI_SYSTEM_BUILDER_GUIDE.md (main guide)

Please follow the guide to help me integrate [SLAM_ALGORITHM] 
with [MY_HARDWARE]. Start with Phase 1 hardware assessment."
```

Bad opening:
```
"Help me with SLAM"
```
(Too vague - Claude won't know to use the guide)

---

### 2. Save Progress YAML Frequently

**After each major phase**:
```
"Can I get my progress YAML file?"

[Claude generates YAML]

"Thanks! I'll save this to resume later."
```

**Save to**:
```bash
~/slam_progress_$(date +%Y%m%d).yaml
```

---

### 3. Use Selective Reading for Efficiency

Instead of loading entire 7000-line guide:
```
"Please read AI_GUIDE_INDEX.md first to understand the routing.
Then load only Phase 1 (lines 60-1100) from AI_SYSTEM_BUILDER_GUIDE.md.
Don't load the entire guide yet."
```

---

### 4. Explicitly Request Tools/Capabilities

If you want Claude to look things up:
```
"I'll tell you I have an Ouster OS1-64 LiDAR. Please use web search 
to look up the specifications (channels, resolution, topics) before 
asking me for details."
```

---

### 5. Keep Context Focused

If conversation gets long:
```
"Let's save progress and start a fresh session for Phase 4.
Can I get the progress YAML?"

[New session]

"Resuming from Phase 3. Here's my progress file: [attach YAML]"
```

---

## 🐛 Troubleshooting

### "Claude doesn't seem to be following the guide"

**Solution**: Be explicit in your request:
```
"Please read the attached AI_SYSTEM_BUILDER_GUIDE.md and follow 
it exactly. You should:
1. Start with Phase 1 opening statement (lines 60-80)
2. Ask me Q1 about hardware platform (lines 86-140)
3. Continue through all 10 questions
4. Don't skip any validation steps"
```

---

### "I can't attach the guide - file too large"

**Solution**: Use selective reading:
```
"The guide is large. Please:
1. Read AI_GUIDE_INDEX.md (small file) for routing instructions
2. I'll tell you which phase I need
3. You read only those line ranges from AI_SYSTEM_BUILDER_GUIDE.md"
```

---

### "Claude lost context mid-session"

**Solution**: Resume with progress YAML:
```
"I need to continue my SLAM integration. Here's my progress YAML:

[Paste YAML content]

Please parse this, see I completed Phase 2, and load Phase 3 
instructions (lines 1372-2600) from AI_SYSTEM_BUILDER_GUIDE.md."
```

---

### "Commands aren't running automatically"

**Expected behavior**: Claude Code shows commands, you run them.

**Workflow**:
1. Claude: "Run this: `rospack find mavros`"
2. You: Copy, paste to terminal, run
3. You: "Result: /opt/ros/noetic/share/mavros"
4. Claude: "Great! MAVROS is installed. Moving on..."

**For Installation Scripts**:
1. Claude: "I'll run the installation script: `cd scripts && ./install_slam_integration.sh install_config.yaml`"
2. **If Claude Code can execute scripts**: It runs automatically
3. **If not**: Claude shows the command, you run it in terminal
4. You: "Script completed successfully"
5. Claude: "Great! Installation complete. Let's verify..."

---

### "No browser tools available"

**Workaround**: Look things up yourself:
```
You: "I have a Hesai JT128 LiDAR"

Claude: "Let me look up specs... [tries browser, fails]"

You: "Let me help - it has 128 channels, publishes on /hesai/pandar, 
uses sensor_msgs/PointCloud2 format."

Claude: "Perfect! Thanks for providing that. Based on PointCloud2 
format, it's compatible with FAST-LIO..."
```

---

## 📊 Token Usage Comparison

| Approach | Tokens per Session | Cost (Sonnet 4.5) | Speed |
|----------|-------------------|-------------------|-------|
| Full guide (Option 1) | ~910k | $3.90 | Slower (large context) |
| Selective reading (Option 2) | ~420k | $2.25 | Faster (focused context) |
| Resume with YAML | ~330k | $1.51 | Fastest (skip Phase 1) |

**Recommendation**: Use Option 2 (selective reading) for best balance.

---

## 📝 Example Session Transcript

**User**: "I need help integrating SLAM with my drone. I've attached AI_GUIDE_INDEX.md. Please follow the optimized workflow to help me set up FAST-LIO with my Jetson Orin NX and Ouster OS1-64."

**Claude**: "I'll help you integrate SLAM! I've read the AI_GUIDE_INDEX.md and understand the optimized workflow.

Before we start, do you have a progress YAML file from a previous session? If not, we'll start fresh with Phase 1.

If starting fresh, I'll:
1. Load Phase 1 (lines 60-1100) from AI_SYSTEM_BUILDER_GUIDE.md
2. Ask you 10 hardware assessment questions
3. Validate compatibility
4. Generate a progress YAML you can save

Ready to begin?"

**User**: "Starting fresh, no progress file yet. Let's begin!"

**Claude**: *[Reads lines 60-1100 of AI_SYSTEM_BUILDER_GUIDE.md]*

"Perfect! Let's start with the SLAM Integration System Builder.

I'll gather information about your hardware and requirements, then generate custom configuration files for your specific setup. This will take about 20-30 minutes.

**Phase 1: Hardware Assessment**

**Q1: Computing Platform**

You mentioned Jetson Orin NX. Let me verify the specifications...

Which version do you have?
- a) 8GB RAM
- b) 16GB RAM

Also, what OS are you running?
- Ubuntu 20.04
- Ubuntu 22.04
- JetPack version?"

*[Session continues...]*

---

## ✅ Quick Start Checklist

- [ ] Copy `slam_integration/` folder to new computer
- [ ] Install Claude Code or have access to Claude
- [ ] Decide: Full guide, selective reading, or resume approach
- [ ] Attach appropriate files (see sections above)
- [ ] Start with clear instructions to Claude
- [ ] Save progress YAML after each major phase
- [ ] Run generated commands in your terminal
- [ ] Continue phase-by-phase until complete

---

## 🆘 Getting Help

If you encounter issues using this with Claude Code:

1. **Check this guide** for your specific scenario
2. **Reference**: `SAVE_RESUME_FEATURE.md` for progress YAML details
3. **Reference**: `OPTIMIZATION_README.md` for selective reading details
4. **Fallback**: Use Option 1 (full guide) if optimization doesn't work

---

## 🎓 Summary

The `slam_integration` system is **fully portable** and works with any AI assistant that can:
- Read attached files
- Follow structured instructions
- Generate text files (configs, launch files)

**Key differences from Cursor**:
- Manual file attachment (vs automatic in Cursor)
- Explicit instructions needed (vs implicit agent behavior)
- Manual command execution (vs integrated terminal)
- Session-based context (vs persistent workspace)

**But the core functionality is identical** - the AI guides you through the same process using the same documentation!

**Ready to start?** Choose your approach above and begin! 🚀


