#!/bin/bash
# Benchmark ONE clean field bag through multiple SLAM frameworks and print a comparison.
# 1. validates the bag is dropout-free (a blacked-out take diverges every framework — garbage in),
# 2. runs SuperOdom  (process_field_bag.sh)  -> map.pcd + trajectory.csv + metrics,
# 3. runs FAST-LIO   (run_fastlio.sh)         -> fastlio_map.pcd + fastlio_traj.csv + metrics,
# 4. prints both summary lines side by side so you can compare drift / divergence / smoothness.
#
# Usage: benchmark_bag.sh <bag_dir_name>   (a folder under ~/superodom_ws/field/)
# Boost the clock first: sudo ~/superodom_ws/set_cpu_clock.sh boost
set -uo pipefail
BAGNAME="${1:?usage: benchmark_bag.sh <bag_dir_name>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIELD_HOST="$HOME/superodom_ws/field/$BAGNAME"

echo "================ BENCHMARK: $BAGNAME ================"
echo "--- 1/3  dropout check ---"
if ! python3 "$HERE/check_bag_gaps.py" "$FIELD_HOST"; then
  echo
  echo "⚠ This bag has a sensor blackout — every framework will diverge on it. Re-record before benchmarking."
  echo "  (continuing anyway so you can see the failure, but the numbers past the gap are meaningless.)"
fi

echo
echo "--- 2/3  SuperOdom ---"
bash "$HERE/process_field_bag.sh" "$BAGNAME" || echo "  (SuperOdom run errored — see output above)"

echo
echo "--- 3/3  FAST-LIO ---"
bash "$HERE/run_fastlio.sh" "$BAGNAME" || echo "  (FAST-LIO run errored — see output above)"

echo
echo "================ COMPARISON: $BAGNAME ================"
SO="$HOME/superodom_ws/results/field/$BAGNAME"
FL="$HOME/slam-gpu/bags/$BAGNAME"
printf '%-12s | %s\n' "framework" "summary (path / peak / final_disp / max_single_step)"
printf '%-12s | %s\n' "SuperOdom" "$(grep -h 'samples=' "$SO/"*.log 2>/dev/null | tail -1 || echo 'n/a — see '"$SO")"
printf '%-12s | %s\n' "FAST-LIO" "$(grep -hE 'FASTLIO SUMMARY' /tmp/fl_mon.log 2>/dev/null | tail -1 | sed 's/FASTLIO SUMMARY //' || echo 'n/a')"
echo
echo "maps:  $SO/map.pcd   vs   $FL/fastlio_map.pcd   (open both in CloudCompare)"
echo "EVAL: lower max_single_step = no divergence; final_disp_from_start ~ loop-closure drift if you returned to start."
