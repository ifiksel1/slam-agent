#!/bin/bash
# Ouster ROS 2 driver control script (start/stop/restart)
# Usage: ./ouster.sh {start|stop|restart|status|logs [N]}

CONTAINER="slam_gpu_system"
CONFIG="/opt/slam_ws/config/ouster_driver.yaml"
LOG="/tmp/ouster.log"
PROC="os_driver"

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
        echo "Ouster driver already running (PID $PID)"
        return 1
    fi
    echo "Starting Ouster ROS 2 driver..."
    docker exec -d "$CONTAINER" bash -c \
        "source /opt/ros/humble/install/setup.bash && \
         source /opt/slam_ws/install/setup.bash && \
         export ROS_DOMAIN_ID=1 && \
         ros2 launch ouster_ros driver.launch.py params_file:=$CONFIG ouster_ns:=ouster viz:=False > $LOG 2>&1"
    sleep 6
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "Ouster driver started (PID $PID)"
        docker exec "$CONTAINER" tail -5 "$LOG"
    else
        echo "Ouster driver failed to start. Logs:"
        docker exec "$CONTAINER" tail -20 "$LOG"
        return 1
    fi
}

do_stop() {
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "Ouster driver not running"
        return 0
    fi
    echo "Stopping Ouster driver (PID $PID)..."
    docker exec "$CONTAINER" kill "$PID"
    sleep 1
    PID=$(get_pid)
    if [ -z "$PID" ]; then
        echo "Ouster driver stopped"
    else
        echo "Force killing..."
        docker exec "$CONTAINER" kill -9 "$PID"
        echo "Ouster driver killed"
    fi
}

do_status() {
    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "Ouster driver running (PID $PID)"
        docker exec "$CONTAINER" tail -3 "$LOG" 2>/dev/null
    else
        echo "Ouster driver not running"
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
