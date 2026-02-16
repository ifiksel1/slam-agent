# MCP Server Quick Reference

## Status: ✅ OPERATIONAL

The MCP server is configured and ready for use with Claude Code.

## How to Use

### In Claude Code
The MCP server will automatically start when Claude Code launches. You'll have access to 15 specialized SLAM tools.

### Key Tools

**Search for hardware profiles:**
```
Use: search_profiles(platform="jetson_orin", sensor="ouster")
```

**Get complete profile:**
```
Use: get_profile(fingerprint="jetson_orin-ouster_os1_64-fast_lio2-ardupilot-humble")
```

**Run diagnostics:**
```
Use: run_diagnostic(diagnostic_name="slam_diagnostics")
```

**Search solutions database:**
```
Use: search_solutions(tags="coordinate_frames,fast_lio2")
```

## Verification Commands

Check if MCP server is healthy:
```bash
/home/dev/slam-agent/mcp/verify_mcp.sh
```

Test wrapper script directly:
```bash
/home/dev/slam-agent/mcp/run_mcp_server.sh --help
```

## Troubleshooting

### MCP server not starting
1. Run verification script: `/home/dev/slam-agent/mcp/verify_mcp.sh`
2. Check Miniforge Python: `/home/dev/miniforge3/bin/python3.12 --version`
3. Verify packages: `PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -m pip list | grep mcp`

### Need to reinstall packages
```bash
PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -m pip install --force-reinstall mcp pyyaml
```

### ROS environment check
If concerned about ROS safety:
```bash
python3 -c "import rospy; print('ROS OK')"
roscore  # Should still work
```

## Important Files

- **Wrapper script**: `/home/dev/slam-agent/mcp/run_mcp_server.sh`
- **MCP server**: `/home/dev/slam-agent/mcp/slam_mcp_server.py`
- **Configuration**: `/home/dev/slam-agent/.mcp.json`
- **Requirements**: `/home/dev/slam-agent/mcp/requirements.txt`
- **Verification**: `/home/dev/slam-agent/mcp/verify_mcp.sh`

## Configuration

```json
{
  "mcpServers": {
    "slam-tools": {
      "command": "/home/dev/slam-agent/mcp/run_mcp_server.sh",
      "args": []
    }
  },
  "enabledMcpjsonServers": ["slam-tools"]
}
```

## Environment Isolation

The wrapper script ensures:
- `PYTHONNOUSERSITE=1` - Ignores user site-packages
- `PYTHONPATH=""` - Clears ROS pollution
- Uses Miniforge Python 3.12 directly

This keeps ROS/SLAM environment completely safe.

## Benefits

- **Token Efficiency**: 95%+ reduction for profile searches and diagnostics
- **15 Specialized Tools**: Installation, diagnostics, profiles, solutions
- **Git-Backed Learning**: Shared knowledge across sessions
- **Safe**: ROS/SLAM environment completely isolated

## Next Steps

1. Restart Claude Code to load the MCP server
2. Start using MCP tools in your SLAM work
3. Run `/home/dev/slam-agent/mcp/verify_mcp.sh` if you encounter issues

---

For detailed setup information, see: `/home/dev/slam-agent/mcp/MCP_SETUP_README.md`
For complete success report, see: `/home/dev/slam-agent/MCP_SETUP_SUCCESS.md`
