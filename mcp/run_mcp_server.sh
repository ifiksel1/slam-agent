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

# Force miniforge's site-packages only
exec /home/dev/miniforge3/bin/python3.12 -I /home/dev/slam-agent/mcp/slam_mcp_server.py "$@"
