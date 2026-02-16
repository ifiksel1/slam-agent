#!/bin/bash
# SLAM System Control — manages all Docker containers
# Usage: slam-system.sh {up|down|restart|status|logs}
#
# Containers managed:
#   slam_gpu_system  — Ouster driver + FAST-LIO GPU
#   slam_mavros      — MAVROS + vision bridge

set -euo pipefail

COMPOSE_DIR="/home/dev/slam-gpu"
COMPOSE_FILE="$COMPOSE_DIR/docker-compose.yml"
SERVICES="slam-gpu mavros"

cd "$COMPOSE_DIR"

usage() {
    echo "Usage: $(basename "$0") {up|down|restart|status|logs [LINES]}"
    echo ""
    echo "Commands:"
    echo "  up        Start all SLAM containers (detached)"
    echo "  down      Stop and remove all SLAM containers"
    echo "  restart   Restart all SLAM containers"
    echo "  status    Show container status and health"
    echo "  logs      Tail container logs (default: 50 lines)"
    exit 1
}

cmd_up() {
    echo "Starting SLAM system..."
    docker compose up -d $SERVICES
    echo ""
    cmd_status
}

cmd_down() {
    echo "Stopping SLAM system..."
    docker compose stop $SERVICES
}

cmd_restart() {
    echo "Restarting SLAM system..."
    docker compose restart $SERVICES
    echo ""
    echo "Waiting 10s for initialization..."
    sleep 10
    cmd_status
}

cmd_status() {
    echo "=== SLAM System Status ==="
    docker ps --filter "name=slam_gpu_system" --filter "name=slam_mavros" \
        --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    echo ""

    # Check FAST-LIO
    echo "--- FAST-LIO ---"
    if docker exec slam_gpu_system pgrep -f "fastlio_mapping" >/dev/null 2>&1; then
        echo "  FAST-LIO: RUNNING"
    else
        echo "  FAST-LIO: NOT RUNNING"
    fi

    # Check Ouster
    echo "--- Ouster Driver ---"
    if docker exec slam_gpu_system pgrep -f "os_driver" >/dev/null 2>&1; then
        echo "  Ouster: RUNNING"
    else
        echo "  Ouster: NOT RUNNING"
    fi

    # Check MAVROS
    echo "--- MAVROS ---"
    if docker exec slam_mavros pgrep -f "mavros_node" >/dev/null 2>&1; then
        echo "  MAVROS: RUNNING"
    else
        echo "  MAVROS: NOT RUNNING"
    fi

    # Check Vision Bridge
    echo "--- Vision Bridge ---"
    BRIDGE_COUNT=$(docker exec slam_mavros pgrep -cf "vision_bridge" 2>/dev/null || echo "0")
    if [ "$BRIDGE_COUNT" -eq 1 ]; then
        echo "  Vision Bridge: RUNNING"
    elif [ "$BRIDGE_COUNT" -gt 1 ]; then
        echo "  Vision Bridge: RUNNING (WARNING: $BRIDGE_COUNT instances!)"
    else
        echo "  Vision Bridge: NOT RUNNING"
    fi
}

cmd_logs() {
    local lines="${1:-50}"
    echo "=== slam_gpu_system (last $lines lines) ==="
    docker logs --tail "$lines" slam_gpu_system 2>&1
    echo ""
    echo "=== slam_mavros (last $lines lines) ==="
    docker logs --tail "$lines" slam_mavros 2>&1
}

case "${1:-}" in
    up)      cmd_up ;;
    down)    cmd_down ;;
    restart) cmd_restart ;;
    status)  cmd_status ;;
    logs)    cmd_logs "${2:-50}" ;;
    *)       usage ;;
esac
