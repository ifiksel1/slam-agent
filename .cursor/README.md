# Cursor AI Integration for SLAM System Builder

This directory contains Cursor-specific configurations to enable AI assistants to help you build SLAM integration systems using the comprehensive guides in this package.

---

## 🚀 Quick Start

### Option 1: Use the SLAM Integration Agent (Recommended for New Integrations)

**When to use**: Starting a new SLAM integration from scratch, want systematic guidance

1. **Open Cursor** in the `slam_integration/` directory
2. **Press** `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. **Type** "Agent" and select "Select AI Agent"
4. **Choose** "SLAM Integration Assistant" from the list
5. **Start chatting**: "I want to integrate SLAM with my drone"

The agent will:
- ✅ Automatically load the guide files
- ✅ Follow the 8-phase workflow systematically
- ✅ Ask hardware assessment questions
- ✅ Generate installation config from your answers
- ✅ Offer automated installation scripts at Phase 4
- ✅ Guide you through complete integration

**See**: `WHEN_TO_USE_AGENT.md` for detailed guidance on when to use agent vs direct guide.

### Option 2: Reference Files Directly

In any Cursor chat, use `@` to reference the guides:

```
@AI_SYSTEM_BUILDER_GUIDE.md How do I set up Ethernet for an Ouster LiDAR?
```

```
@AI_SYSTEM_BUILDER_GUIDE.md My drone moves backward when I send forward commands
```

```
@SLAM_ARDUPILOT_INTEGRATION_GUIDE.md Show me the LIO-SAM configuration
```

### Option 3: Workspace Rules (Automatic)

The `.cursorrules` file automatically applies when you:
- Open any file in `slam_integration/`
- Chat about SLAM-related topics
- Generate or edit SLAM configuration files

**No setup needed** - it just works! ✅

---

## 📂 Files in This Directory

### `agents/slam_integration_agent.md`
**Cursor Agent Definition** - Specialized AI agent for SLAM integration.

**What it does**:
- Follows the 8-phase methodology from AI_SYSTEM_BUILDER_GUIDE.md
- Asks targeted questions to understand your hardware
- Uses browser tools to look up specifications
- Generates complete, ready-to-use configuration files
- Provides troubleshooting help

**When to use**: When building a new SLAM integration from scratch.

### `.cursorrules`
**Workspace Rules** - Automatic context for all SLAM work.

**What it does**:
- Reminds AI about documentation structure
- Provides quick reference for common issues
- Enforces best practices (safety, progressive testing)
- Maps symptoms to Phase 8 solutions

**When to use**: Always active when in `slam_integration/` directory.

---

## 🎯 How to Use the Agent

### Starting a New Integration

```
You: "I want to integrate FAST-LIO with an Ouster OS1-64 on a Jetson Orin NX"

Agent: "Great! I'll help you build a complete SLAM integration system...
        Let's start with your hardware. Your Jetson Orin NX comes in 
        8GB and 16GB versions. Which do you have?"
```

The agent will:
1. ✅ Ask about your hardware (platform, sensors, flight controller, LiDAR connection)
2. ✅ Look up specifications using browser tools
3. ✅ Validate compatibility
4. ✅ **CREATE/SETUP ROS WORKSPACE** (if needed)
5. ✅ **INSTALL ALL PACKAGES** (MAVROS, sensor drivers, SLAM, vision_to_mavros)
6. ✅ **CONFIGURE LIDAR NETWORK** (netplan, ping test, UDP verification)
7. ✅ Generate custom config files with YOUR values
8. ✅ Create launch files, URDF, ArduPilot parameters
9. ✅ Build workspace and verify installation
10. ✅ Provide step-by-step integration instructions
11. ✅ Give you a testing protocol
12. ✅ Help troubleshoot issues

### Getting Help with Existing System

```
You: "My drone moves backward when I command it forward"

Agent: [References Phase 8.1 - Coordinate Frame Confusion]
       "This is a coordinate frame issue. Let's verify your TF tree..."
```

### Troubleshooting

```
You: "SLAM is too slow, CPU is at 100%"

Agent: [References Phase 8.4 - Performance Tuning]
       "Let's optimize your system. First, check your CPU governor..."
```

---

## 📖 Available Documentation

The agent has access to these documents:

| Document | Size | Purpose |
|----------|------|---------|
| **AI_SYSTEM_BUILDER_GUIDE.md** | 99 KB<br>3,361 lines | **Primary reference**<br>• Phases 1-7: Interactive setup<br>• Phase 8: Common issues |
| SLAM_ARDUPILOT_INTEGRATION_GUIDE.md | 31 KB<br>1,064 lines | Technical details<br>LIO-SAM example |
| SLAM_INTEGRATION_TEMPLATE.md | 13 KB<br>481 lines | Quick templates |
| SLAM_INTEGRATION_DIAGNOSTICS.md | 20 KB<br>793 lines | Systematic debugging |
| QUICK_START.md | 10 KB<br>327 lines | One-page reference |

---

## 🛠️ Advanced: Customize the Agent

Edit `agents/slam_integration_agent.md` to:
- Change interaction style
- Add custom workflows
- Include project-specific defaults
- Add additional references

Example customization:

```markdown
## Project-Specific Defaults

When user doesn't specify:
- Default platform: Jetson Orin NX 16GB
- Default LiDAR: Ouster OS1-64
- Default SLAM: FAST-LIO2
- Default ROS: ROS1 Noetic
```

---

## 🔍 Browser Tool Usage

The agent is configured to **proactively use browser tools** to:

1. **Look up GitHub repositories** when you mention a SLAM algorithm
   - Example: "FAST-LIO" → Navigates to hku-mars/FAST_LIO, extracts topics, dependencies

2. **Search sensor specifications** when you mention hardware
   - Example: "Ouster OS1-64" → Finds datasheet, extracts channels, resolution, IP

3. **Verify flight controller IMU specs** for better config defaults
   - Example: "Pixhawk 4" → Finds ICM-20689 IMU, extracts noise parameters

4. **Check ROS driver compatibility**
   - Example: Searches for "ouster_ros" package, verifies ROS1/ROS2 support

**This saves you hours of manual research!** 🎉

---

## 💡 Tips for Best Results

### 1. Be Specific About Hardware
**Good**: "Jetson Orin NX 16GB with Ouster OS1-64 and Pixhawk 6X"  
**Bad**: "Jetson and a LiDAR"

### 2. Mention Your Goal
**Good**: "I need GPS-denied navigation for warehouse inspection"  
**Bad**: "I want SLAM"

### 3. Ask Targeted Questions
**Good**: "How do I configure Ethernet for my Ouster LiDAR?"  
**Bad**: "Help with LiDAR"

### 4. Reference Phase 8 for Issues
**Good**: "I'm hitting the issue in Phase 8.1 - coordinate frames are wrong"  
**Bad**: "It's broken"

### 5. Use @ Mentions
**Good**: `@AI_SYSTEM_BUILDER_GUIDE.md Phase 8.4 - my SLAM is slow`  
**Bad**: Trying to describe the problem from memory

---

## 🎓 Learning Path

### Beginner (Never used SLAM before)
1. Start with agent: "I want to build a SLAM system from scratch"
2. Let agent guide you through ALL questions
3. Follow testing protocol exactly
4. Reference Phase 8 when stuck

### Intermediate (Have used SLAM before)
1. Quickly review `@QUICK_START.md`
2. Use agent for config generation
3. Skip to Phase 4 (Integration)
4. Reference Phase 8 for specific issues

### Advanced (Integrating custom SLAM)
1. Give agent GitHub URL of custom SLAM algorithm
2. Agent will analyze it and guide you
3. Use `@SLAM_INTEGRATION_TEMPLATE.md` as reference
4. Customize generated files as needed

---

## 🆘 Common Questions

**Q: Can the agent access my hardware specs automatically?**  
A: Yes! If you provide model numbers (e.g., "Jetson Orin NX"), the agent uses browser tools to look up RAM, CPU, GPU specs.

**Q: Does it work with PX4?**  
A: Yes! The agent supports both ArduPilot and PX4. 
- **ROS1 + PX4**: Uses MAVROS (MAVLink bridge)
- **ROS2 + PX4**: Uses DDS/micro-ROS Agent (native, more efficient) ⭐
- It will automatically detect your configuration and use the best method!

**Q: Can it help with cameras instead of LiDAR?**  
A: Yes! The agent supports visual SLAM (ORB-SLAM3, VINS-Fusion, etc.) and VIO (OpenVINS, etc.).

**Q: What if my SLAM algorithm isn't listed?**  
A: Give the agent a GitHub URL. It will navigate to the repo, read the README, and figure out how to integrate it.

**Q: Can I save the conversation?**  
A: Yes! The agent generates actual files you can save. Copy them to your workspace and commit to git.

---

## 🐛 Troubleshooting the Agent

**Agent doesn't appear in list**:
- Restart Cursor
- Check file exists: `.cursor/agents/slam_integration_agent.md`
- Verify no syntax errors in agent file

**Agent doesn't use browser tools**:
- Check if browser tools are enabled in Cursor settings
- Try explicitly asking: "Can you look up the specs for X?"

**Agent gives generic answers**:
- Use @ mention: `@AI_SYSTEM_BUILDER_GUIDE.md` 
- Reference specific phases: "Follow Phase 1 of the guide"

**Want different behavior**:
- Edit `.cursor/agents/slam_integration_agent.md`
- Or edit `.cursorrules` for workspace-wide changes

---

## 📞 Getting Help

If you encounter issues with the agent or guides:

1. **Check Phase 8** in AI_SYSTEM_BUILDER_GUIDE.md
2. **Run diagnostics**: `scripts/slam_diagnostics.sh`
3. **Reference flowchart**: Phase 8.9 troubleshooting flowchart
4. **Ask targeted questions** with @ mentions

---

**Happy Building!** 🚁✨

The SLAM Integration Assistant is here to help you build robust, flight-ready GPS-denied navigation systems. Start chatting and let's get your drone flying autonomously!

