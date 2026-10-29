#!/usr/bin/env bash
# Stage the l4-planner build context: the Dockerfile, the SUPER patch and config, and planner/ (the C++
# packages under planner/ros, which the Dockerfile builds, and their config; tests left out; symlinks kept,
# the Dockerfile copies the packages with cp -L).
#   docker/planner/stage.sh <out-dir>
set -euo pipefail
R=$(cd "$(dirname "$0")/../.." && pwd); O=${1:?out dir}
rm -rf "$O"; mkdir -p "$O"
cp "$R/docker/planner/Dockerfile" "$R/docker/sim/patch_seed_line.py" "$R/docker/sim/config/super_revolute.yaml" "$O/"
rsync -a --exclude __pycache__ --exclude test "$R/planner" "$O/"
echo "staged $O"
