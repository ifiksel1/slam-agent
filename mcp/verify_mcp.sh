#!/bin/bash
# Quick verification script for MCP server health.
# Portable: works for any user and any clone location.
#   - Paths are resolved relative to this script (follows symlinks).
#   - The Python interpreter is auto-discovered the same way the wrapper does,
#     or set SLAM_MCP_PYTHON to override.

echo "=== MCP Server Health Check ==="
echo ""

# Resolve this script's own directory so nothing depends on $HOME or cwd
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WRAPPER="$SCRIPT_DIR/run_mcp_server.sh"
MCP_JSON="$(dirname "$SCRIPT_DIR")/.mcp.json"

# Pick a Python interpreter (mirrors run_mcp_server.sh):
#   1. $SLAM_MCP_PYTHON  2. conda/forge installs in $HOME  3. active conda env  4. python3
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

echo "1. Checking wrapper script exists..."
if [ -x "$WRAPPER" ]; then
    echo "   ✓ Wrapper script found and executable"
    echo "     Path: $WRAPPER"
else
    echo "   ✗ Wrapper script missing or not executable"
    echo "     Expected: $WRAPPER"
    exit 1
fi

echo ""
echo "2. Checking Python interpreter..."
if [ -n "$PYTHON" ] && [ -x "$(command -v "$PYTHON" 2>/dev/null || echo "$PYTHON")" ]; then
    echo "   ✓ Python interpreter found"
    echo "     Path: $PYTHON"
    echo "     Version: $("$PYTHON" --version 2>&1)"
else
    echo "   ✗ No usable Python interpreter found"
    echo "     Set SLAM_MCP_PYTHON to a Python with 'mcp' and 'pyyaml' installed"
    exit 1
fi

echo ""
echo "3. Checking MCP package installation..."
if PYTHONNOUSERSITE=1 PYTHONPATH="" "$PYTHON" -c "import mcp.server.fastmcp" 2>/dev/null; then
    echo "   ✓ MCP package found and importable"
else
    echo "   ✗ MCP package not installed or import failed"
    echo "     Install with: PYTHONNOUSERSITE=1 PYTHONPATH= \"$PYTHON\" -m pip install -r \"$SCRIPT_DIR/requirements.txt\""
    exit 1
fi

echo ""
echo "4. Checking PyYAML installation..."
if PYTHONNOUSERSITE=1 PYTHONPATH="" "$PYTHON" -c "import yaml" 2>/dev/null; then
    echo "   ✓ PyYAML found"
else
    echo "   ✗ PyYAML not installed or import failed"
    exit 1
fi

echo ""
echo "5. Testing MCP server startup..."
"$WRAPPER" </dev/null >/dev/null 2>&1 &
SERVER_PID=$!
sleep 1
if ps -p $SERVER_PID > /dev/null 2>&1; then
    echo "   ✓ MCP server starts and runs"
    kill $SERVER_PID 2>/dev/null
    wait $SERVER_PID 2>/dev/null
elif wait $SERVER_PID 2>/dev/null; then
    echo "   ✓ MCP server starts (exits cleanly when no stdio connection)"
else
    echo "   ✗ MCP server failed to start"
    exit 1
fi

echo ""
echo "6. Checking .mcp.json configuration..."
if [ -f "$MCP_JSON" ]; then
    echo "   ✓ .mcp.json found"
    if grep -q "run_mcp_server.sh" "$MCP_JSON"; then
        echo "   ✓ Configuration points to wrapper script"
    else
        echo "   ⚠ Configuration may not use wrapper script"
    fi
else
    echo "   ✗ .mcp.json not found"
    echo "     Expected: $MCP_JSON"
    exit 1
fi

echo ""
echo "=== All checks passed! ==="
echo ""
echo "MCP server is ready for use with Claude Code."
echo "Claude Code should now be able to use MCP tools to reduce token usage."
