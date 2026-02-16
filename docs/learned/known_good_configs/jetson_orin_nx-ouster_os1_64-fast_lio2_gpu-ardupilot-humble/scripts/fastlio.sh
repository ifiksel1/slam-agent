#!/bin/bash
# FAST-LIO GPU control script (start/stop/restart)
# Usage: ./fastlio.sh {start|stop|restart|status}

CONTAINER="slam_gpu_system"
CONFIG="/opt/slam_ws/config/fast_lio_gpu.yaml"
LOG="/tmp/fastlio.log"
PROC="fastlio_mapping"

get_pid() {
    # Find non-zombie PID matching the exact binary name (not bash wrappers)
    docker exec "$CONTAINER" bash -c "
        for pid in \$(pgrep -x $PROC 2>/dev/null); do
            if ! grep -q '^State:.*Z' /proc/\$pid/status 2>/dev/null; then
                echo \$pid; break
            fi
        done
    "
}

do_start() {
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "FAST-LIO already running (PID $PID)"
        return 1
    fi
    echo "Starting FAST-LIO GPU..."
    docker exec -d "$CONTAINER" bash -c \
        "source /opt/ros/humble/install/setup.bash && \
         source /opt/slam_ws/install/setup.bash && \
         export ROS_DOMAIN_ID=1 && \
         ros2 run fast_lio fastlio_mapping --ros-args --params-file $CONFIG > $LOG 2>&1"
    sleep 4
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "FAST-LIO started (PID $PID)"
        docker exec "$CONTAINER" tail -5 "$LOG"
    else
        echo "FAST-LIO failed to start. Logs:"
        docker exec "$CONTAINER" tail -20 "$LOG"
        return 1
    fi
}

do_stop() {
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "FAST-LIO not running"
        return 0
    fi
    echo "Stopping FAST-LIO (PID $PID)..."
    docker exec "$CONTAINER" kill "$PID"
    sleep 1
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "FAST-LIO stopped"
    else
        echo "Force killing..."
        docker exec "$CONTAINER" kill -9 "$PID"
        echo "FAST-LIO killed"
    fi
}

do_status() {
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "FAST-LIO running (PID $PID)"
        docker exec "$CONTAINER" tail -3 "$LOG" 2>/dev/null
    else
        echo "FAST-LIO not running"
    fi
}

case "${1:-}" in
    start)   do_start ;;
    stop)    do_stop ;;
    restart) do_stop && sleep 1 && do_start ;;
    status)  do_status ;;
    logs)    docker exec "$CONTAINER" tail -${2:-30} "$LOG" ;;
    *)       echo "Usage: $0 {start|stop|restart|status|logs [N]}" ;;
esac
