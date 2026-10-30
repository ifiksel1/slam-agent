#!/usr/bin/env bash
# Click and run: inside the run's ROS container (closed_loop.sh's FAULT_HOOK), the click nodes from the run's
# C++ install space (/ws_bx/install: l4_click, l4_sim_viz next to the bridge and the executive).
#   nodes.sh [fly_height_m] [record 0|1]
source /opt/ros/noetic/setup.bash
source /ws_bx/install/setup.bash
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
# closed_loop.sh's bag (/planner/cloud and SUPER's markers: ~25 GB an hour) only with RECORD=1
if [ "${2:-0}" != 1 ]; then pkill -INT -f "rosbag record" && echo "bag recording stopped (RECORD=1 keeps it)"; fi
rosrun l4_sim_viz sim_viz > /run/logs/sim_viz.log 2>&1 &                  # SIM ONLY: /map, /map_low, /tf, /robot/body
rosrun l4_click click_goal _fly_height:=${1:-1.0} > /run/logs/click_goal.log 2>&1 &
echo "click nodes up (fly_height ${1:-1.0} m)"
wait
