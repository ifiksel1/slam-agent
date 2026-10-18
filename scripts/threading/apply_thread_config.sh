#!/bin/bash
# Port the 4.1.4 FAST-LIO threading config to a robot.
#
#   4 OpenMP threads (MP_PROC_NUM=4), laserMapping pinned to one thread per fast core
#   (taskset -c 0,2,4,6) and idle OpenMP workers sleeping (OMP_WAIT_POLICY=passive).
#   See docs/FASTLIO_THREADING.md for the benchmark and the reasoning.
#
# Usage (on the robot, from the slam-agent checkout):
#   scripts/threading/apply_thread_config.sh check            # read-only: what is the robot running?
#   scripts/threading/apply_thread_config.sh build            # set MP_PROC_NUM=4 in the container + catkin build fast_lio
#   scripts/threading/apply_thread_config.sh restart          # disarmed-only restart of slam-fastlio + verification
#
# The launch-file half (taskset + OMP env on the laserMapping node) ships in
# config/fastlio_to_fc.launch in this repo, which is bind-mounted into the container,
# so pulling this branch is enough for that part.
set -euo pipefail

CONTAINER="${CONTAINER:-slam-hesai-fastlio}"
UNIT="${UNIT:-slam-fastlio}"
CPUS="${CPUS:-0,2,4,6}"
THREADS="${THREADS:-4}"
GUI="${GUI:-http://localhost:8080}"
CM=/root/slam_ws/src/FAST_LIO_SLAM/FAST_LIO_SLAM/FAST-LIO/CMakeLists.txt
FLAGS=/root/slam_ws/build/fast_lio/CMakeFiles/fastlio_mapping.dir/flags.make
BIN=/root/slam_ws/devel/.private/fast_lio/lib/fast_lio/fastlio_mapping
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
LAUNCH="$ROOT/config/fastlio_to_fc.launch"
STAMP="$(date +%Y-%m-%d)"

say()  { echo "[threading] $*"; }
die()  { echo "[threading] ERROR: $*" >&2; exit 1; }
cx()   { docker exec "$CONTAINER" bash -lc "$*"; }

check_topology() {
    # The 0,2,4,6 pinning assumes the nuc-04 layout: CPUs 0-7 = 4 fast cores x 2 hyperthreads,
    # CPUs 8-11 = slow cores. Print the layout so a different CPU is caught before pinning.
    say "CPU: $(lscpu | sed -n 's/^Model name:\s*//p')"
    lscpu -e=CPU,CORE,MAXMHZ | sed 's/^/[threading]   /'
    local cores; cores=$(lscpu -e=CPU,CORE | awk -v l="$CPUS" 'BEGIN{n=split(l,a,",");for(i=1;i<=n;i++)w[a[i]]=1} NR>1&&($1 in w){print $2}' | sort -u | wc -l)
    local want; want=$(echo "$CPUS" | tr ',' '\n' | wc -l)
    [ "$cores" -eq "$want" ] || say "WARNING: CPUs $CPUS map to $cores physical cores (expected $want); pick one CPU per fast core for this machine"
}

check_running() {
    local p; p=$(docker exec "$CONTAINER" pgrep -x fastlio_mapping || true)
    say "compiled MP_PROC_NUM: $(cx "grep -o 'MP_PROC_NUM=[0-9]*' $FLAGS 2>/dev/null || echo 'not built'")"
    say "launch prefix in repo: $(grep -o 'launch-prefix="taskset[^"]*"' "$LAUNCH" | head -1 || echo none)"
    say "OMP env in repo launch: $(grep -c 'OMP_WAIT_POLICY' "$LAUNCH") line(s)"
    if [ -n "$p" ]; then
        say "running fastlio_mapping pid $p: $(docker exec "$CONTAINER" grep Cpus_allowed_list /proc/$p/status | cut -f2) | $(docker exec "$CONTAINER" bash -c "tr '\0' '\n' </proc/$p/environ | grep OMP_WAIT_POLICY || echo 'OMP_WAIT_POLICY unset'") | $(docker exec "$CONTAINER" bash -c "ls /proc/$p/task | wc -l") threads"
    else
        say "fastlio_mapping not running"
    fi
}

armed_state() { curl -s "$GUI/api/slam/status" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("armed"))' 2>/dev/null || echo unknown; }

case "${1:-check}" in
  check)
    check_topology; check_running ;;
  build)
    check_topology
    cx "cp -p $CM $CM.bak-$STAMP && cp -p $BIN $BIN.bak-pre-threads-$STAMP"
    cx "sed -i -E '0,/add_definitions\\(-DMP_PROC_NUM=[0-9]+\\)/s//add_definitions(-DMP_PROC_NUM=$THREADS)/; 0,/message\\(\"core for MP: [0-9]+\"\\)/s//message(\"core for MP: $THREADS\")/' $CM"
    cx "grep -n 'MP_PROC_NUM' $CM | head -1"
    # Build with NO taskset/cpuset: FAST-LIO's CMakeLists derives MP_PROC_NUM from the CPU
    # count CMake sees, and a restricted build silently compiles fewer threads.
    cx "source /opt/ros/noetic/setup.bash && cd /root/slam_ws && catkin build fast_lio --no-status 2>&1 | tail -3"
    got=$(cx "grep -o 'MP_PROC_NUM=[0-9]*' $FLAGS" | head -1)
    [ "$got" = "MP_PROC_NUM=$THREADS" ] || die "build has $got, expected MP_PROC_NUM=$THREADS"
    say "built with $got; old binary kept as $BIN.bak-pre-threads-$STAMP"
    say "persist across container recreate (optional): docker tag slam-system:latest slam-system:pre-threads-$STAMP && docker commit $CONTAINER slam-system:latest" ;;
  restart)
    a=$(armed_state); [ "$a" = "False" ] || die "vehicle armed state is '$a'; restart only when confirmed disarmed"
    # A rosmaster left over from bag replays (other containers share host port 11311) would be
    # adopted by the new roslaunch and orphan every node when it exits.
    if ss -ltn | grep -q ':11311 '; then
        say "a ROS master is listening on 11311; stopping $UNIT first and re-checking"
        sudo systemctl stop "$UNIT"; sleep 5
        ss -ltn | grep -q ':11311 ' && die "something else owns port 11311; stop it (e.g. a replay roscore) before restarting"
    fi
    sudo systemctl restart "$UNIT"; sleep 30
    check_running
    cx "source /opt/ros/noetic/setup.bash; timeout 8 rostopic hz /Odometry 2>&1 | grep average | tail -1" || true ;;
  *) echo "usage: $(basename "$0") [check|build|restart]" >&2; exit 2 ;;
esac
