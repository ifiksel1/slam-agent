#!/usr/bin/env bash
# Induce REAL FAST-LIO publish latency on the bench, to exercise the latency
# detector end to end. This is the T3 method from docs/LATENCY_BENCH_RESULTS.md.
#
# Method: duty-cycle laserMapping with SIGSTOP/SIGCONT -- stopped STOP_MS,
# running RUN_MS. The scan queue keeps filling while the process is stopped, so
# per-scan cost effectively exceeds the 50 ms budget at 20 Hz and a backlog
# accumulates. That is the 25 August failure mechanism. It is NOT the same as
# loading the whole machine, which would also degrade mavros and the very
# measurement the test depends on.
#
# STOP_MS is deliberately kept under the 1.0 s health watchdog so the LATENCY
# path is exercised and not the health-loss path. Raise it past 1000 and you are
# testing something else.
#
# SAFETY
#   - refuses to run unless the vehicle is disarmed
#   - SIGCONT on ANY exit, including Ctrl-C and errors, via trap
#   - bounded duration; no FC command is ever issued by this script
#
# Usage:  scripts/bench/latency_overrun.sh [DURATION_S] [STOP_MS] [RUN_MS]
# Default 30 s at 400/100 ms, i.e. ~20% speed, which is what T3 used.

set -uo pipefail

DURATION=${1:-30}
STOP_MS=${2:-400}
RUN_MS=${3:-100}
CONTAINER=${CONTAINER:-slam-hesai-fastlio}

if [ "$STOP_MS" -ge 1000 ]; then
    echo "REFUSING: STOP_MS >= 1000 crosses the 1.0 s health watchdog." >&2
    echo "That tests health-loss, not latency. Use a smaller value." >&2
    exit 1
fi

armed=$(docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/noetic/setup.bash; timeout 8 rostopic echo -n1 /mavros/state/armed 2>/dev/null | head -1' \
    2>/dev/null | tr -d '[:space:]')
if [ "$armed" != "False" ]; then
    echo "REFUSING: /mavros/state.armed reads '${armed:-unknown}', expected False." >&2
    exit 1
fi
echo "vehicle disarmed - ok"

PID=$(docker exec "$CONTAINER" pgrep -f fastlio_mapping | head -1)
[ -n "$PID" ] || { echo "laserMapping not running" >&2; exit 1; }
echo "laserMapping pid $PID, duty-cycling ${STOP_MS}ms stopped / ${RUN_MS}ms running for ${DURATION}s"

# Whatever happens from here, the process gets resumed.
resume() {
    docker exec "$CONTAINER" kill -CONT "$PID" 2>/dev/null || true
    echo "SIGCONT sent - laserMapping resumed"
}
trap resume EXIT INT TERM

end=$(( $(date +%s) + DURATION ))
while [ "$(date +%s)" -lt "$end" ]; do
    docker exec "$CONTAINER" kill -STOP "$PID" 2>/dev/null || break
    sleep "$(awk "BEGIN{print $STOP_MS/1000}")"
    docker exec "$CONTAINER" kill -CONT "$PID" 2>/dev/null || break
    sleep "$(awk "BEGIN{print $RUN_MS/1000}")"
done

echo "load off; watch the drain"
