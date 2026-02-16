# MCP Server Setup for Jetson Orin

## Problem
The Jetson Orin system has a broken Python environment due to conflicting packages between:
- System Python 3.8 (ROS Noetic dependencies)
- Miniforge Python 3.12 (for MCP server)
- ROS workspace environment variables that pollute Python sys.path

When trying to run the MCP server directly, Python would try to import broken system packages (like `distutils` from `/usr/lib/python3/dist-packages`) which caused module import failures.

## Solution
Created a wrapper script that isolates the Python environment:

**File**: `/home/dev/slam-agent/mcp/run_mcp_server.sh`

The wrapper:
1. Sets `PYTHONNOUSERSITE=1` to ignore user site-packages
2. Clears `PYTHONPATH` to prevent ROS workspace pollution
3. Uses Miniforge Python 3.12 directly: `/home/dev/miniforge3/bin/python3.12`

## Installation
Packages installed to Miniforge Python with clean environment:
```bash
PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -m pip install mcp pyyaml
```

## Configuration
**File**: `/home/dev/slam-agent/.mcp.json`
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

## Testing
```bash
# Test wrapper script starts without errors
timeout 3 /home/dev/slam-agent/mcp/run_mcp_server.sh

# Should timeout (exit 124) which means server is running
# Any other exit code or error output indicates a problem
```

## Why This Works
- Miniforge Python 3.12 has all required packages in its isolated environment
- The wrapper ensures clean PYTHONPATH so system packages can't interfere
- ROS/SLAM workspace is completely unaffected (uses system Python 3.8)
- No conda environments needed - direct installation to Miniforge base

## Maintenance
To update MCP packages:
```bash
PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -m pip install --upgrade mcp pyyaml
```

## Benefits
- Reduces token usage by using MCP tools instead of file operations
- Provides specialized SLAM tools (diagnostics, profile management, learning persistence)
- Keeps ROS environment completely isolated and safe
