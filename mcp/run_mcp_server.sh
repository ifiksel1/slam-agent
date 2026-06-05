#!/bin/bash
# Wrapper script to run the SLAM MCP server with a clean Python environment.
# This ensures system Python packages (especially broken ROS ones) don't interfere.
#
# Portable: works for any user and any clone location.
#   - The server path is resolved relative to this script (follows symlinks).
#   - The Python interpreter is auto-discovered, or set SLAM_MCP_PYTHON to override.

set -euo pipefail

# Nuclear option: completely isolate from ROS and system Python
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1

# Resolve this script's own directory so the server path doesn't depend on $HOME
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SERVER="$SCRIPT_DIR/slam_mcp_server.py"

# Pick a Python interpreter that has `mcp` + `pyyaml` installed:
#   1. $SLAM_MCP_PYTHON (explicit override)
#   2. a Miniforge/conda/anaconda install in the user's home
#   3. the currently-active conda env
#   4. python3 on PATH
pick_python() {
    if [[ -n "${SLAM_MCP_PYTHON:-}" ]]; then
        echo "$SLAM_MCP_PYTHON"; return
    fi
    for candidate in \
        "$HOME/miniforge3/bin/python" \
        "$HOME/miniconda3/bin/python" \
        "$HOME/anaconda3/bin/python" \
        "${CONDA_PREFIX:-}/bin/python"; do
        [[ -x "$candidate" ]] && { echo "$candidate"; return; }
    done
    command -v python3
}

PYTHON="$(pick_python)"

# -I: isolated mode (ignore env vars and user site-packages)
exec "$PYTHON" -I "$SERVER" "$@"
