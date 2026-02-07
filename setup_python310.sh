#!/bin/bash
# Setup Python 3.10 for SLAM MCP server on Jetson ARM64
# This script uses Miniforge (conda) which supports ARM64 architecture
# Usage: bash setup_python310.sh

set -e

echo "=== Installing Python 3.10 for SLAM MCP Server ==="
echo "Architecture: $(dpkg --print-architecture)"
echo "Current Python: $(python3 --version)"
echo ""

# Step 1: Install Miniforge if not present
if [ ! -d ~/miniforge3 ]; then
    echo "[1/3] Installing Miniforge (ARM64-optimized conda)..."
    cd /tmp
    wget https://github.com/conda-forge/miniforge/releases/download/24.11.0-0/Miniforge3-24.11.0-0-Linux-aarch64.sh -O miniforge.sh
    bash miniforge.sh -b -p ~/miniforge3
    rm miniforge.sh
    cd -
    echo "✓ Miniforge installed"
else
    echo "✓ Miniforge already installed"
fi

# Step 2: Create Python 3.10 environment
echo "[2/3] Creating Python 3.10 environment..."
unset PYTHONPATH
export PATH="$HOME/miniforge3/bin:$PATH"
mamba create -n mcp-py310 python=3.10 -y > /dev/null
echo "✓ Environment created"

# Step 3: Install MCP requirements
echo "[3/3] Installing MCP requirements..."
~/miniforge3/envs/mcp-py310/bin/python -m pip install -q -r mcp/requirements.txt
echo "✓ MCP installed"

echo ""
echo "=== Verification ==="
~/miniforge3/envs/mcp-py310/bin/python --version
~/miniforge3/envs/mcp-py310/bin/python -c "from mcp.server.fastmcp import FastMCP; print('✓ MCP OK')"

echo ""
echo "✅ Setup complete!"
echo ""
echo ".mcp.json has been updated to use:"
echo "  /home/dev/miniforge3/envs/mcp-py310/bin/python"
echo ""
echo "MCP server is ready for use!"
