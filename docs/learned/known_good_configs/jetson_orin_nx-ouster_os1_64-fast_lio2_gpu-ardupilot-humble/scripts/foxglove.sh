#!/bin/bash
# Foxglove Bridge control script (start/stop/restart)
# Usage: ./foxglove.sh {start|stop|restart|status|logs [N]}

CONTAINER="slam_gpu_system"
LOG="/tmp/foxglove.log"
PROC="foxglove_bridge"

get_pid() {
    # Find non-zombie PID matching the exact binary name
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
        echo "Foxglove Bridge already running (PID $PID)"
        return 1
    fi
    echo "Starting Foxglove Bridge..."
    docker exec -d "$CONTAINER" bash -c \
        "source /opt/ros/humble/install/setup.bash && \
         source /opt/slam_ws/install/setup.bash && \
         export ROS_DOMAIN_ID=1 && \
         ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > $LOG 2>&1"
    sleep 3
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "Foxglove Bridge started (PID $PID)"
        docker exec "$CONTAINER" tail -5 "$LOG"
    else
        echo "Foxglove Bridge failed to start. Logs:"
        docker exec "$CONTAINER" tail -20 "$LOG"
        return 1
    fi
}

do_stop() {
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "Foxglove Bridge not running"
        return 0
    fi
    echo "Stopping Foxglove Bridge (PID $PID)..."
    docker exec "$CONTAINER" kill "$PID"
    sleep 1
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "Foxglove Bridge stopped"
    else
        echo "Force killing..."
        docker exec "$CONTAINER" kill -9 "$PID"
        echo "Foxglove Bridge killed"
    fi
}

do_status() {
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "Foxglove Bridge running (PID $PID)"
        echo "  Foxglove Studio: ws://$(hostname -I | awk '{print $1}'):8765"
        docker exec "$CONTAINER" tail -3 "$LOG" 2>/dev/null
    else
        echo "Foxglove Bridge not running"
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
