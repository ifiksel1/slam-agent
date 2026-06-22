# Python 3.10 Setup for Jetson MCP Server

## Problem
- Jetson running Ubuntu 20.04 with Python 3.8.10 on ARM64
- MCP package requires Python 3.10+
- pip3 install fails: "Could not find a version that satisfies the requirement mcp>=1.0.0"
- Deadsnakes PPA doesn't support ARM64 architecture

## ✅ Working Solution
Use Miniforge (conda for ARM64) to create a Python 3.10 environment.

### Why This Works
- Miniforge provides ARM64-optimized conda packages
- Coexists with system Python 3.8
- Isolated environment prevents dependency conflicts
- Better compatibility for scientific/robotics packages

### Installation
1. Run: `bash setup_python310.sh` (provided in repo)
2. Or manually:
   ```bash
   unset PYTHONPATH
   cd /tmp
   wget https://github.com/conda-forge/miniforge/releases/download/24.11.0-0/Miniforge3-24.11.0-0-Linux-aarch64.sh
   bash Miniforge3-24.11.0-0-Linux-aarch64.sh -b -p ~/miniforge3
   export PATH="$HOME/miniforge3/bin:$PATH"
   mamba create -n mcp-py310 python=3.10 -y
   ~/miniforge3/envs/mcp-py310/bin/python -m pip install -r mcp/requirements.txt
   ```

### Verification
```bash
~/miniforge3/envs/mcp-py310/bin/python -c "from mcp.server.fastmcp import FastMCP; print('MCP OK')"
```

### .mcp.json Update
```json
{
  "mcpServers": {
    "slam-tools": {
      "command": "/home/dev/miniforge3/envs/mcp-py310/bin/python",
      "args": ["mcp/slam_mcp_server.py"]
    }
  }
}
```
**Note:** Don't commit - this is machine-specific local config
