#!/usr/bin/env bash
# Click and run: the Foxglove WebSocket bridge, in its own container (l4-sim-l2:foxglove) that joins the run's
# network namespace (--network container:l2-ros-<name>; still no host network), and a relay of its port to
# the unix socket /run/foxglove.sock (the run directory, which the host relays to a TCP port).
# Clients may publish only the click topics; no services, no parameters (no arming from Foxglove).
# A 3D panel left on Foxglove's default publish topics (/clicked_point, /move_base_simple/goal, /initialpose:
# every layout but click_and_run_layout.json) works too: the bridge remaps them to the click topics, so its
# publishers only ever reach /click_and_run/*. Without this the bridge refused the clicks ("not whitelisted",
# 2026-11-02 18:44, layout closed_loop_drive_fly_layout) and nothing reached click_goal. MAVROS's own
# /move_base_simple/goal (frame map) stays apart: nothing of the bridge publishes on it.
source /opt/ros/noetic/setup.bash
source /super_ws/devel/setup.bash   # SUPER's message types (quadrotor_msgs) for the bridge's schemas
export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 ROSCONSOLE_STDOUT_LINE_BUFFERED=1  # log live
rosparam set /foxglove_bridge/capabilities "[clientPublish, connectionGraph]"
rosparam set /foxglove_bridge/client_topic_whitelist "['^/click_and_run/drive$', '^/click_and_run/fly$', '^/click_and_run/pose_estimate$', '^/clicked_point$', '^/move_base_simple/goal$', '^/initialpose$']"
rosparam set /foxglove_bridge/service_whitelist "['^$']"
rosparam set /foxglove_bridge/param_whitelist "['^$']"
rosrun foxglove_bridge foxglove_bridge __name:=foxglove_bridge _port:=8765 _address:=127.0.0.1 \
  /clicked_point:=/click_and_run/drive /move_base_simple/goal:=/click_and_run/fly /initialpose:=/click_and_run/pose_estimate \
  > /run/logs/foxglove_bridge.log 2>&1 &
rm -f /run/foxglove.sock
socat UNIX-LISTEN:/run/foxglove.sock,fork,mode=777 TCP:127.0.0.1:8765 > /run/logs/foxglove_relay.log 2>&1 &
wait
