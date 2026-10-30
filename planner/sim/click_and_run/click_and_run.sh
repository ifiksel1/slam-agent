#!/usr/bin/env bash
# Click and run (2026-10-30): the L2 closed loop (closed_loop.sh: QuadRover SITL, SUPER, the track planner, the
# landing check, the planner bridge and the mission executive) with the executive in its interactive mode, live
# in Foxglove: click in the 3D panel and the robot drives or flies there. SITL ONLY: the run has no host
# network (sitl_harness.sh); only the Foxglove bridge's WebSocket is relayed out (a unix socket in the run
# directory -> host TCP FG_PORT). Runs until stopped (Ctrl-C here, or click_and_run.sh stop).
#
#   click_and_run.sh [map.pcd start_x start_y start_yaw_deg]     default: the HP Gate L2 course, 0 0 -90.7
#   click_and_run.sh stop
#
# Foxglove: open a connection, Foxglove WebSocket, ws://<this host>:8765 (on Shannon's desktop
# ws://localhost:8765); layout click_and_run_layout.json. In the 3D panel's publish tool (top right):
#   Publish point  -> drive there (on the ground; in the air it lands where it is first)
#   Publish pose   -> fly there, FLY_HEIGHT above the floor, ending facing the arrow (from the ground it
#                     takes off first)
# A new click replaces the running goal. The goal marker: yellow sent, orange running, green reached, red
# refused / unreachable / stopped, grey replaced.
#
# Env: L2_BASE (default ~/sim/click: planner_sim, planner, slam, runs, cpp_ws/install), NAME (run name,
#   default click), FG_PORT (8765), FG_BIND (0.0.0.0), FLY_HEIGHT (1.0 m), CPUS (5: the ROS container; SITL 2,
#   the Foxglove bridge 1), RECORD (0: closed_loop.sh's bag is stopped at the start, ~25 GB an hour; 1 keeps
#   it, runs/<name>/closed_loop.bag), SESSION_S (14400: then fly_mission.py lands SITL and the run ends), SUPER_CFG,
#   L4_BRIDGE_WS (the install space with l4_bridge, l4_executive, l4_click, l4_sim_viz; default
#   $L2_BASE/cpp_ws/install). Anything closed_loop.sh / sitl_harness.sh take (DRIVE_PARMS, LIDAR_UP, ...)
#   passes through.
set -uo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)   # planner_sim
export L2_BASE=${L2_BASE:-$HOME/sim/click}
NAME=${NAME:-click}
PORT=${FG_PORT:-8765}
H=$HERE/sitl_harness.sh
RUN=$L2_BASE/runs/$NAME
ROS=l2-ros-$NAME; FG=l2-fg-$NAME

cleanup() {
  trap - EXIT INT TERM
  pkill -f "socat TCP-LISTEN:$PORT," 2>/dev/null
  docker rm -f $FG >/dev/null 2>&1
  $H down $NAME >/dev/null 2>&1
  [ -n "${CL:-}" ] && kill $CL 2>/dev/null
  echo "click and run $NAME: stopped (logs $RUN/logs)"
}
if [ "${1:-}" = stop ]; then cleanup; exit 0; fi

map=${1:-$HERE/course/hp_gate_l2.pcd}; sx=${2:-0}; sy=${3:-0}; syaw=${4:--90.7}
cfg=${SUPER_CFG:-$HERE/super_closed_loop_hp.yaml}
mission=$L2_BASE/planner/missions/interactive.yaml
if ss -ltn | grep -q ":$PORT "; then echo "port $PORT is in use (FG_PORT=...)"; exit 1; fi
docker info >/dev/null 2>&1 || { echo "cannot reach docker (not in the docker group in this shell? run: newgrp docker)"; exit 1; }
docker image inspect l4-sim-l2:foxglove >/dev/null 2>&1 || { echo "no image l4-sim-l2:foxglove (Dockerfile.foxglove)"; exit 1; }

export CPUS=${CPUS:-5}
export L4_BRIDGE_WS=${L4_BRIDGE_WS:-$L2_BASE/cpp_ws/install}
export FAULT_HOOK="bash /planner_sim/click_and_run/nodes.sh ${FLY_HEIGHT:-1.0} ${RECORD:-0}"
export RECORD_EXTRA="/click_and_run/drive /click_and_run/fly /mission_executive/goal /click_and_run/status ${RECORD_EXTRA:-}"
# a previous run's files (SITL writes them as root)
docker run --rm -v $L2_BASE/runs:/r alpine rm -rf /r/$NAME >/dev/null 2>&1
trap cleanup EXIT INT TERM
$HERE/closed_loop.sh $NAME $map $sx $sy $syaw $mission $cfg ${SESSION_S:-14400} &
CL=$!

# the Foxglove bridge once the run's roscore is up, then the host relay
for _ in $(seq 1 120); do
  docker exec $ROS bash -c "source /opt/ros/noetic/setup.bash; rosparam get /run_id" >/dev/null 2>&1 && break
  sleep 1
done
docker run -d --name $FG --network container:$ROS --cpus 1 -v $RUN:/run -v $HERE:/planner_sim:ro \
  --entrypoint bash l4-sim-l2:foxglove /planner_sim/click_and_run/foxglove.sh >/dev/null
for _ in $(seq 1 30); do [ -S $RUN/foxglove.sock ] && break; sleep 1; done
socat TCP-LISTEN:$PORT,bind=${FG_BIND:-0.0.0.0},reuseaddr,fork UNIX-CONNECT:$RUN/foxglove.sock &
echo "click and run $NAME: Foxglove at ws://<this host's address>:$PORT ($(hostname -I | tr ' ' '\n' | grep -E '^(192|10)\.' | tr '\n' ' ')), ws://localhost:$PORT on this desktop."
echo "  The robot takes clicks once $RUN/logs/fly_mission.log shows phase IDLE (about a minute)."
wait $CL
