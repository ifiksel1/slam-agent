#!/usr/bin/env bash
#
# switch_restart_mode.sh — flip the drift_monitor restart pipeline between:
#   of    -> EKF source failover to Optical Flow (SRC1->SRC2, back on recovery).
#            No flight-mode change. Vehicle keeps flying on OF while SLAM re-inits. (DEFAULT)
#   land  -> command the FC into LAND, then restart SLAM.
#            *** LAND is a real flight-mode change: it commands a descent when flying. ***
#
# Applies to BOTH triggers automatically (pilot CH9 hold + automatic drift-CRITICAL),
# because both call one shared response function that branches on restart_mode.
#
# Change is PERSISTENT:
#   - edits restart_mode in the launch file (bind-mounted from the repo + git-tracked)
#     -> survives container recreation and full reboots.
#   - also sets it live on the running node (rosparam + respawn) so it takes effect now.
#
# This script NEVER arms and NEVER triggers a restart/land itself. It only selects which
# response a future CH9/drift trigger will run.
#
# Usage:
#   ./switch_restart_mode.sh of        # switch to Optical-Flow failover
#   ./switch_restart_mode.sh land      # switch to LAND response
#   ./switch_restart_mode.sh --status  # print the current mode (launch file + live node)
#
set -euo pipefail

# --- config (override via env) ---
REPO="${SLAM_REPO:-$HOME/Documents/slam-agent}"
LAUNCH="${SLAM_LAUNCH:-$REPO/config/fastlio_to_fc_quadrover.launch}"
CONTAINER="${SLAM_CONTAINER:-slam-hesai-fastlio}"
NODE="/drift_monitor"

RED=$'\e[31m'; GRN=$'\e[32m'; YLW=$'\e[33m'; BLD=$'\e[1m'; RST=$'\e[0m'
die() { echo "${RED}error:${RST} $*" >&2; exit 1; }

# run a command inside the SLAM container with ROS sourced
in_container() {
  docker exec "$CONTAINER" bash -lc "source /opt/ros/*/setup.bash 2>/dev/null; source /root/slam_ws/devel/setup.bash 2>/dev/null; $*"
}

launch_mode() {   # read restart_mode value from the launch file
  grep -oP '<arg name="restart_mode"\s+default="\K[^"]+' "$LAUNCH" 2>/dev/null || echo "?"
}
live_mode() {     # read restart_mode from the running param server (empty if node/param absent)
  in_container "rosparam get $NODE/restart_mode" 2>/dev/null | tr -d '[:space:]' || true
}

print_status() {
  echo "${BLD}restart_mode${RST}"
  echo "  launch file : $(launch_mode)   ($LAUNCH)"
  local lm; lm="$(live_mode)"
  echo "  live node   : ${lm:-<node not running / param unset>}"
}

# --- args ---
[ $# -eq 1 ] || { echo "usage: $0 {of|land|--status}"; exit 2; }
case "$1" in
  --status|-s|status) print_status; exit 0 ;;
  of|land) MODE="$1" ;;
  *) die "unknown mode '$1' (expected: of | land | --status)" ;;
esac

# --- preflight ---
[ -f "$LAUNCH" ] || die "launch file not found: $LAUNCH (set SLAM_REPO or SLAM_LAUNCH)"
grep -q '<arg name="restart_mode"' "$LAUNCH" || die "restart_mode arg missing from $LAUNCH"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || die "container '$CONTAINER' not running"

CUR="$(launch_mode)"
if [ "$MODE" = "land" ]; then
  echo "${YLW}${BLD}WARNING:${RST} 'land' makes a CH9 hold / drift-CRITICAL command the FC into LAND"
  echo "${YLW}         (a real descent when flying). Bench-test DISARMED only.${RST}"
fi

# --- 1. persist: edit the launch file (idempotent) ---
if [ "$CUR" = "$MODE" ]; then
  echo "launch file already restart_mode=$MODE (no edit)"
else
  # replace only the restart_mode arg's default="..."
  sed -i -E 's#(<arg name="restart_mode"[^>]*default=")[^"]*(")#\1'"$MODE"'\2#' "$LAUNCH"
  [ "$(launch_mode)" = "$MODE" ] || die "launch-file edit failed (value is now '$(launch_mode)')"
  echo "${GRN}✔${RST} launch file: restart_mode $CUR -> $MODE (persistent)"
fi

# --- 2. apply live: set param + restart the node (respawn=true brings it back) ---
if [ -n "$(live_mode)" ] || in_container "rosnode list 2>/dev/null" | grep -qx "$NODE"; then
  in_container "rosparam set $NODE/restart_mode $MODE"
  in_container "rosnode kill $NODE" >/dev/null 2>&1 || true
  echo -n "restarting $NODE "
  for _ in $(seq 1 15); do
    sleep 1; echo -n "."
    if in_container "rosnode list 2>/dev/null" | grep -qx "$NODE"; then
      if [ "$(live_mode)" = "$MODE" ]; then echo " ${GRN}up${RST}"; break; fi
    fi
  done
else
  echo "${YLW}note:${RST} $NODE not running — launch-file change will apply on next start."
fi

echo
print_status
