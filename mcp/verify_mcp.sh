#!/bin/bash
# Quick verification script for MCP server health
# Run this to check if MCP server is properly configured

echo "=== MCP Server Health Check ==="
echo ""

echo "1. Checking wrapper script exists..."
if [ -x "/home/dev/slam-agent/mcp/run_mcp_server.sh" ]; then
    echo "   ✓ Wrapper script found and executable"
else
    echo "   ✗ Wrapper script missing or not executable"
    exit 1
fi

echo ""
echo "2. Checking Miniforge Python..."
if [ -x "/home/dev/miniforge3/bin/python3.12" ]; then
    echo "   ✓ Miniforge Python 3.12 found"
    VERSION=$(/home/dev/miniforge3/bin/python3.12 --version)
    echo "     Version: $VERSION"
else
    echo "   ✗ Miniforge Python 3.12 not found"
    exit 1
fi

echo ""
echo "3. Checking MCP package installation..."
PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -c "import mcp.server.fastmcp; print('   ✓ MCP package found and importable')" 2>&1
if [ $? -ne 0 ]; then
    echo "   ✗ MCP package not installed or import failed"
    exit 1
fi

echo ""
echo "4. Checking PyYAML installation..."
PYTHONNOUSERSITE=1 PYTHONPATH="" /home/dev/miniforge3/bin/python3.12 -c "import yaml; print('   ✓ PyYAML found')" 2>&1
if [ $? -ne 0 ]; then
    echo "   ✗ PyYAML not installed or import failed"
    exit 1
fi

echo ""
echo "5. Testing MCP server startup..."
/home/dev/slam-agent/mcp/run_mcp_server.sh </dev/null >/dev/null 2>&1 &
SERVER_PID=$!
sleep 1
if ps -p $SERVER_PID > /dev/null 2>&1; then
    echo "   ✓ MCP server starts and runs"
    kill $SERVER_PID 2>/dev/null
    wait $SERVER_PID 2>/dev/null
elif [ $? -eq 0 ]; then
    echo "   ✓ MCP server starts (exits when no stdio connection)"
else
    echo "   ✗ MCP server failed to start"
    exit 1
fi

echo ""
echo "6. Checking .mcp.json configuration..."
if [ -f "/home/dev/slam-agent/.mcp.json" ]; then
    echo "   ✓ .mcp.json found"
    if grep -q "run_mcp_server.sh" /home/dev/slam-agent/.mcp.json; then
        echo "   ✓ Configuration points to wrapper script"
    else
        echo "   ⚠ Configuration may not use wrapper script"
    fi
else
    echo "   ✗ .mcp.json not found"
    exit 1
fi

echo ""
echo "=== All checks passed! ==="
echo ""
echo "MCP server is ready for use with Claude Code."
echo "Claude Code should now be able to use MCP tools to reduce token usage."
