#!/bin/bash
# A/B two EllipseLIO configs on the SAME bag, N interleaved runs each, to compare
# runaway rate + loop closure. Each run_bag.sh call recreates the container fresh
# (clean DDS slate per run). Interleaved order (A,B,A,B,...) cancels time/thermal drift.
#
# Usage: ab_imu_config.sh <bag_dir> <cfgA.yaml> <cfgB.yaml> [N=10] [thresh_m=3.0]
set -uo pipefail
BAG="${1:?usage: ab_imu_config.sh <bag_dir> <cfgA> <cfgB> [N] [thresh]}"
CFGA="${2:?cfgA}"; CFGB="${3:?cfgB}"
N="${4:-10}"; TH="${5:-3.0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ELLIPSELIO_WS:-$HOME/ellipselio_ws}"
RES="$WS/results/field"
CSV="$RES/ab_${BAG}_$(printf '%s' "${CFGA%.yaml}_vs_${CFGB%.yaml}").csv"
echo "run,arm,config,final_disp_m,peak_m,max_step_m,runaway" > "$CSV"

parse() { # $1=sampler.log  -> echoes "FD PK MS"
  local L="$1"
  local fd pk ms
  fd=$(grep -oE 'final_disp_from_start=[0-9.]+' "$L" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  pk=$(grep -oE 'peak_excursion=[0-9.]+'        "$L" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  ms=$(grep -oE 'max_single_step=[0-9.]+'       "$L" 2>/dev/null | tail -1 | grep -oE '[0-9.]+$')
  echo "${fd:-NaN} ${pk:-NaN} ${ms:-NaN}"
}

run_one() { # $1=arm(A/B) $2=cfg $3=run_index
  local arm="$1" cfg="$2" i="$3"
  local tag="_ab_${arm}_run${i}"
  echo "================= arm $arm  cfg=$cfg  run $i/$N ================="
  "$HERE/run_bag.sh" "$BAG" "$cfg" "$tag" >/dev/null 2>&1 || true
  local slog="$RES/${BAG}${tag}/sampler.log"
  read -r FD PK MS < <(parse "$slog")
  local ra=0
  awk -v fd="${FD}" -v th="$TH" 'BEGIN{ if(fd=="NaN"){exit 2}; exit !(fd+0>th+0) }' && ra=1
  [ "$FD" = "NaN" ] && ra="CRASH"
  echo "$i,$arm,$cfg,$FD,$PK,$MS,$ra" >> "$CSV"
  echo ">>> arm $arm run $i: final_disp=${FD}m peak=${PK}m max_step=${MS}m  runaway=$ra"
}

echo "A/B: $BAG   A=$CFGA   B=$CFGB   N=$N   runaway_thresh=${TH}m"
for i in $(seq 1 "$N"); do
  run_one A "$CFGA" "$i"
  run_one B "$CFGB" "$i"
done

echo; echo "########################## A/B SUMMARY ##########################"
python3 - "$CSV" "$TH" <<'PY'
import csv,sys,statistics as st
csvf,th=sys.argv[1],float(sys.argv[2])
rows=list(csv.DictReader(open(csvf)))
for arm,cfg in sorted({(r['arm'],r['config']) for r in rows}):
    a=[r for r in rows if r['arm']==arm]
    fds=[float(r['final_disp_m']) for r in a if r['final_disp_m']!='NaN']
    crash=sum(1 for r in a if r['final_disp_m']=='NaN')
    ra=sum(1 for f in fds if f>th)
    clean=[f for f in fds if f<=th]
    n=len(a)
    print(f"\narm {arm}  [{cfg}]  n={n}")
    print(f"  runaway (>%.1fm): %d/%d = %.0f%%   crashes: %d"%(th,ra,n,100*ra/n,crash))
    if clean:
        print(f"  clean-run closure: median=%.3fm  min=%.3fm  max=%.3fm  (n=%d)"%(
              st.median(clean),min(clean),max(clean),len(clean)))
print(f"\nCSV: {csvf}")
PY
echo "################################################################"
