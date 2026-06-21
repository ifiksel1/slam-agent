#!/bin/bash
# Robustly kill any prior SuperOdom + bag-play processes inside the container BEFORE a fresh launch.
#
# Why this exists: `pkill -f "ros2 launch super_odometry"` does NOT reliably kill the launch
# supervisor or its bash wrapper, so repeated launches STACK UP -> two SuperOdom instances ->
# the "flicker / two running at once" bug in Foxglove. And `pkill -f "super_odometry"` can match
# (and kill) the invoking shell itself. This guard kills by exact PID from `ps`, excluding its own
# shell AND zombies, and loops until no LIVE (non-defunct) SuperOdom binary remains.
#
# NOTE on zombies: the container runs `sleep infinity` as PID 1, which does NOT reap children, so
# killed nodes linger as <defunct> (state Z). Zombies hold no DDS/CPU and are harmless, but they
# must be EXCLUDED from the "is it clean?" check (they are dead). To stop them accumulating across
# many replays, recreate the container with `docker run --init ...` (tini reaps zombies).
#
# Usage: superodom_cleanup.sh [CONTAINER]   (default container: superodom)
set -u
CTR="${1:-superodom}"

docker exec "$CTR" bash -c '
  SELF=$$
  # live (non-zombie) SuperOdom/bag PIDs, excluding grep and THIS shell
  live_pids() {
    ps -eo pid,stat,args \
      | grep -E "super_odometry|ros2 bag play" \
      | grep -v grep \
      | awk -v s="$SELF" "\$1 != s && \$2 !~ /Z/ {print \$1}"
  }
  killed_any=0
  for attempt in 1 2 3 4 5 6; do
    pids=$(live_pids)
    [ -z "$pids" ] && break
    kill -9 $pids 2>/dev/null && killed_any=1
    sleep 1
  done
  left=$(live_pids | wc -l)
  if [ "$left" -ne 0 ]; then
    echo "WARN: $left live SuperOdom/bag process(es) still alive after cleanup"; exit 1
  fi
  [ "$killed_any" -eq 1 ] && echo "cleanup: prior SuperOdom/bag killed" || echo "cleanup: nothing to kill"
'
