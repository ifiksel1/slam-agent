#!/bin/bash
# Deploy SLAM Integration System via Docker
# Usage: ./deploy_docker_slam.sh [OPTIONS]
# Options: --build, --start, --stop, --test, --all

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_WS="${SLAM_WS_PATH:-~/slam_ws}"
IMAGE_NAME="slam_integration:latest"
CONTAINER_NAME="slam_launch"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_status() { echo -e "${GREEN}[✓]${NC} $1"; }
echo_error() { echo -e "${RED}[✗]${NC} $1"; }
echo_info() { echo -e "${YELLOW}[*]${NC} $1"; }

# Function: Build Docker image
build_image() {
    echo_info "Building Docker image: $IMAGE_NAME"

    if [ ! -f "$SLAM_WS/Dockerfile" ]; then
        echo_error "Dockerfile not found at $SLAM_WS/Dockerfile"
        return 1
    fi

    echo_info "Starting build (this will take 60-90 minutes)..."
    cd "$SLAM_WS"

    if docker build -t "$IMAGE_NAME" -f Dockerfile . ; then
        echo_status "Docker image built successfully"

        # Verify image
        SIZE=$(docker images "$IMAGE_NAME" --format "{{.Size}}")
        echo_status "Image size: $SIZE"

        # Test package discovery
        echo_info "Verifying packages in image..."
        PKG_COUNT=$(docker run --rm "$IMAGE_NAME" bash -c \
            "source /catkin_ws/devel/setup.bash && rospack list 2>/dev/null | wc -l" 2>/dev/null || echo "0")

        if [ "$PKG_COUNT" -gt 100 ]; then
            echo_status "Found $PKG_COUNT ROS packages"
        else
            echo_error "Package discovery failed or count too low: $PKG_COUNT"
            return 1
        fi

        return 0
    else
        echo_error "Docker build failed"
        return 1
    fi
}

# Function: Start containers
start_containers() {
    echo_info "Starting SLAM system with docker-compose..."

    if [ ! -f "$SLAM_WS/docker-compose.yml" ]; then
        echo_error "docker-compose.yml not found at $SLAM_WS/docker-compose.yml"
        return 1
    fi

    cd "$SLAM_WS"

    if docker compose up -d slam_launch ; then
        echo_status "SLAM system started"

        # Wait for initialization
        echo_info "Waiting for system initialization (10 seconds)..."
        sleep 10

        # Check container status
        if docker compose ps | grep -q "slam_launch.*Up"; then
            echo_status "Container is running"

            # Display logs
            echo_info "Recent logs:"
            docker compose logs slam_launch | tail -20

            return 0
        else
            echo_error "Container failed to start"
            docker compose logs slam_launch | tail -30
            return 1
        fi
    else
        echo_error "Failed to start containers"
        return 1
    fi
}

# Function: Stop containers
stop_containers() {
    echo_info "Stopping SLAM system..."
    cd "$SLAM_WS"

    if docker compose down ; then
        echo_status "SLAM system stopped"
        return 0
    else
        echo_error "Failed to stop containers"
        return 1
    fi
}

# Function: Run diagnostic tests
run_tests() {
    echo_info "Running diagnostic tests..."

    if ! docker compose ps | grep -q "slam_launch.*Up"; then
        echo_error "Container not running. Start with --start first"
        return 1
    fi

    cd "$SLAM_WS"

    # Test 1: ROS nodes
    echo_info "Test 1: Checking active ROS nodes..."
    NODE_COUNT=$(docker compose exec -T slam_launch bash -c \
        "source /catkin_ws/devel/setup.bash && rosnode list 2>/dev/null | wc -l" || echo "0")

    if [ "$NODE_COUNT" -gt 3 ]; then
        echo_status "Found $NODE_COUNT active ROS nodes"
    else
        echo_error "ROS nodes not running properly: $NODE_COUNT"
    fi

    # Test 2: Topics
    echo_info "Test 2: Checking ROS topics..."
    TOPIC_COUNT=$(docker compose exec -T slam_launch bash -c \
        "source /catkin_ws/devel/setup.bash && rostopic list 2>/dev/null | wc -l" || echo "0")

    if [ "$TOPIC_COUNT" -gt 5 ]; then
        echo_status "Found $TOPIC_COUNT ROS topics publishing"
    else
        echo_error "Topics not publishing properly: $TOPIC_COUNT"
    fi

    # Test 3: Package discovery
    echo_info "Test 3: Verifying package discovery..."
    PKG_LIST=$(docker compose exec -T slam_launch bash -c \
        "source /catkin_ws/devel/setup.bash && for pkg in fast_lio ouster_ros std_detector orin_slam_integration aloam_velodyne vision_to_mavros; do rospack find \$pkg 2>/dev/null && echo OK || echo FAIL; done" || echo "FAIL")

    if echo "$PKG_LIST" | grep -q "OK"; then
        OK_COUNT=$(echo "$PKG_LIST" | grep -c "OK" || echo "0")
        echo_status "Found $OK_COUNT/6 required packages"
    else
        echo_error "Package discovery failed"
    fi

    # Test 4: SLAM odometry topic
    echo_info "Test 4: Checking SLAM odometry..."
    ODOM_INFO=$(docker compose exec -T slam_launch bash -c \
        "source /catkin_ws/devel/setup.bash && rostopic info /Odometry 2>&1 | head -3" || echo "NOT FOUND")

    if echo "$ODOM_INFO" | grep -q "Publishers"; then
        echo_status "SLAM odometry topic active"
    else
        echo_error "SLAM odometry topic not publishing"
    fi

    return 0
}

# Function: Show help
show_help() {
    cat << EOF
SLAM Docker Deployment Tool

Usage: $0 [OPTION]

Options:
    --build         Build Docker image (60-90 min)
    --start         Start SLAM system containers
    --stop          Stop SLAM system containers
    --test          Run diagnostic tests
    --all           Build, start, and test (full deployment)
    --logs          Show container logs
    --shell         Access container shell
    --help          Show this help message

Examples:
    # Full deployment
    $0 --all

    # Just build
    $0 --build

    # Start after building
    $0 --start

    # Monitor system
    $0 --logs

    # Debug
    $0 --shell

Environment Variables:
    SLAM_WS_PATH    Path to slam_ws (default: ~/slam_ws)

EOF
}

# Main execution
main() {
    local action="${1:---help}"

    case "$action" in
        --build)
            build_image
            ;;
        --start)
            start_containers
            ;;
        --stop)
            stop_containers
            ;;
        --test)
            run_tests
            ;;
        --all)
            build_image && start_containers && run_tests
            ;;
        --logs)
            cd "$SLAM_WS"
            docker compose logs -f slam_launch
            ;;
        --shell)
            cd "$SLAM_WS"
            docker compose exec slam_launch bash
            ;;
        --help|-h)
            show_help
            ;;
        *)
            echo_error "Unknown option: $action"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
