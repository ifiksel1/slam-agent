#!/usr/bin/env bash
# Click and run: the Foxglove WebSocket bridge, in its own container (l4-sim-l2:foxglove) that joins the run's
# network namespace (--network container:l2-ros-<name>; still no host network), and a relay of its port to
# the unix socket /run/foxglove.sock (the run directory, which the host relays to a TCP port).
# Clients may publish only the click topics; no services, no parameters (no arming from Foxglove).
source /opt/ros/noetic/setup.bash
source /super_ws/devel/setup.bash   # SUPER's message types (quadrotor_msgs) for the bridge's schemas
export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311
rosparam set /foxglove_bridge/capabilities "[clientPublish, connectionGraph]"
rosparam set /foxglove_bridge/client_topic_whitelist "['^/click_and_run/drive$', '^/click_and_run/fly$', '^/click_and_run/pose_estimate$']"
rosparam set /foxglove_bridge/service_whitelist "['^$']"
rosparam set /foxglove_bridge/param_whitelist "['^$']"
rosrun foxglove_bridge foxglove_bridge __name:=foxglove_bridge _port:=8765 _address:=127.0.0.1 \
  > /run/logs/foxglove_bridge.log 2>&1 &
rm -f /run/foxglove.sock
socat UNIX-LISTEN:/run/foxglove.sock,fork,mode=777 TCP:127.0.0.1:8765 > /run/logs/foxglove_relay.log 2>&1 &
wait
