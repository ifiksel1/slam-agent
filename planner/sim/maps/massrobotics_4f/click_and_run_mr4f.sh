#!/usr/bin/env bash
# Click and run (click_and_run.sh) on the Mass Robotics 4th floor map, SITL only, on Shannon.
# Its own run (NAME mr4f) and Foxglove port (FG_PORT, default 8775), so it can sit next to the owner's
# ~/sim/click run on 8765. Foxglove: ws://<shannon>:8775, layout foxglove/layouts/click_and_run_layout.json.
#
#   click_and_run_mr4f.sh          start (runs until Ctrl-C)
#   click_and_run_mr4f.sh stop     stop it from another shell
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
. $HERE/mr4f.env
export NAME=${NAME:-mr4f} FG_PORT=${FG_PORT:-8775} SUPER_CFG=${SUPER_CFG:-$MR4F_SUPER}
C=$L2_BASE/planner_sim/click_and_run/click_and_run.sh
if [ "${1:-}" = stop ]; then exec $C stop; fi
exec $C $MR4F_MAP $MR4F_START
