#!/bin/bash
# Wrapper script to run MCP server with clean Python environment
# This ensures system Python packages (especially broken ROS ones) don't interfere

# Nuclear option: completely isolate from ROS and system Python
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1

# Resolve this script's directory so the server path is location-independent
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Pick a Python interpreter, in priority order:
#   1. SLAM_MCP_PYTHON env var (explicit override)
#   2. miniforge / conda python under $HOME (isolated from ROS, has mcp+pyyaml)
#   3. python3 on PATH
if [ -n "$SLAM_MCP_PYTHON" ]; then
    PYTHON="$SLAM_MCP_PYTHON"
elif [ -x "$HOME/miniforge3/bin/python3" ]; then
    PYTHON="$HOME/miniforge3/bin/python3"
elif [ -x "$HOME/miniconda3/bin/python3" ]; then
    PYTHON="$HOME/miniconda3/bin/python3"
else
    PYTHON="$(command -v python3)"
fi

# Force the chosen interpreter's site-packages only (-I = isolated mode)
exec "$PYTHON" -I "$SCRIPT_DIR/slam_mcp_server.py" "$@"
