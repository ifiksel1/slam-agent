#!/usr/bin/env bash
# One closed-loop mission (closed_loop.sh) on the Mass Robotics 4th floor map, SITL only, on Shannon.
#
#   closed_loop_mr4f.sh <run_name> [mission_map_frame.yaml] [timeout_s]
#     default mission: $L2_BASE/planner/missions/mr4f_smoke.yaml; logs in $L2_BASE/runs/<run_name>/
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
. $HERE/mr4f.env
name=${1:?run name}; mission=${2:-$L2_BASE/planner/missions/mr4f_smoke.yaml}; timeout=${3:-900}
docker run --rm -v $L2_BASE/runs:/r alpine rm -rf /r/$name >/dev/null 2>&1   # a previous run (root-owned)
cd $L2_BASE && exec planner_sim/closed_loop.sh $name $MR4F_MAP $MR4F_START $mission $MR4F_SUPER $timeout
