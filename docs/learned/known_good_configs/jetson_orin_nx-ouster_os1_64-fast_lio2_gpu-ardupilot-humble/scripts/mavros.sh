#!/bin/bash
# MAVROS bridge control script (start/stop/restart)
# Usage: ./mavros.sh {start|stop|restart|status|logs [N]}

CONTAINER="slam_mavros"
COMPOSE_FILE="/home/dev/slam-gpu/docker-compose.yml"
LOG="/tmp/mavros.log"
PROC="mavros_node"

container_exists() {
    docker inspect "$CONTAINER" >/dev/null 2>&1
}

container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]
}

get_pid() {
    # Find non-zombie PID matching the exact binary name (not bash wrappers)
    docker exec "$CONTAINER" bash -c "
        for pid in \$(pgrep -f $PROC 2>/dev/null); do
            if ! grep -q '^State:.*Z' /proc/\$pid/status 2>/dev/null; then
                echo \$pid; break
            fi
        done
    " 2>/dev/null
}

mavros_connected() {
    # Check if MAVROS is connected to FCU
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash && \
        ros2 topic echo /mavros/state --once 2>/dev/null | grep -q 'connected: true'
    " 2>/dev/null
}

do_start() {
    if container_running; then
        echo "MAVROS container already running"
        PID=$(get_pid)
        if [ -n "$PID" ]; then
            echo "MAVROS node active (PID $PID)"
            return 0
        fi
    fi

    echo "Starting MAVROS container..."
    docker compose -f "$COMPOSE_FILE" up -d mavros

    echo "Waiting for MAVROS to start..."
    sleep 8

    if container_running; then
        PID=$(get_pid)
        if [ -n "$PID" ]; then
            echo "MAVROS container started (PID $PID)"

            # Check connection status
            sleep 2
            if mavros_connected; then
                echo "✓ MAVROS connected to ArduPilot"
            else
                echo "⚠ MAVROS running but not yet connected (this may take 10-20 seconds)"
            fi

            # Show recent logs
            docker logs "$CONTAINER" --tail 10
        else
            echo "MAVROS container started but node not detected. Logs:"
            docker logs "$CONTAINER" --tail 20
            return 1
        fi
    else
        echo "Failed to start MAVROS container. Logs:"
        docker logs "$CONTAINER" --tail 30
        return 1
    fi
}

do_stop() {
    if ! container_running; then
        echo "MAVROS container not running"
        return 0
    fi

    echo "Stopping MAVROS container..."
    docker compose -f "$COMPOSE_FILE" stop mavros
    sleep 2

    if ! container_running; then
        echo "MAVROS container stopped"
    else
        echo "Force stopping..."
        docker compose -f "$COMPOSE_FILE" kill mavros
        echo "MAVROS container killed"
    fi
}

do_status() {
    if ! container_exists; then
        echo "MAVROS container does not exist (run 'docker compose up -d mavros' first)"
        return 1
    fi

    if ! container_running; then
        echo "MAVROS container exists but is not running"
        return 1
    fi

    PID=$(get_pid)
    if [ -n "$PID" ]; then
        echo "MAVROS container running (PID $PID)"

        # Check connection status
        if mavros_connected; then
            echo "✓ MAVROS connected to ArduPilot"
        else
            echo "⚠ MAVROS not connected (check /dev/ttyACM0 and baud rate)"
        fi

        # Show recent logs
        echo ""
        echo "Recent activity:"
        docker logs "$CONTAINER" --tail 5 2>/dev/null
    else
        echo "MAVROS container running but node not detected"
        echo ""
        echo "Recent logs:"
        docker logs "$CONTAINER" --tail 10
    fi
}

case "${1:-}" in
    start)   do_start ;;
    stop)    do_stop ;;
    restart) do_stop && sleep 2 && do_start ;;
    status)  do_status ;;
    logs)    docker logs "$CONTAINER" --tail "${2:-30}" ;;
    *)       echo "Usage: $0 {start|stop|restart|status|logs [N]}" ;;
esac
