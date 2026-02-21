#!/bin/bash
# Flight Data Recorder - Manages ROS bag recording for SLAM flights
# Usage: ./flight_recorder.sh {start|stop|status|list|last|clean}

FLIGHTS_ROOT="/home/dev/slam-agent/flights"
PID_FILE="/tmp/flight_recorder.pid"
CURRENT_FLIGHT_FILE="/tmp/flight_recorder_current"
INDEX_FILE="$FLIGHTS_ROOT/flight_index.yaml"

# Default ROS 2 topics (slam_mavros container)
ROS2_TOPICS_DEFAULT=(
    /ouster/imu
    /fast_lio_gpu/odometry
    /fast_lio_gpu/path
    /tf
    /tf_static
    /mavros/vision_pose/pose
    /mavros/local_position/pose
    /mavros/local_position/velocity_local
    /mavros/state
    /mavros/battery
    /mavros/imu/data
    /mavros/imu/data_raw
    /mavros/rc/in
    /mavros/rc/out
    /mavros/setpoint_raw/target_attitude
    /mavros/target_actuator_control
    /mavros/esc_telemetry/telemetry
    /mavros/esc_status/status
    /mavros/global_position/global
    /mavros/mission/waypoints
)

# ROS 1 topics (slam_system container)
ROS1_TOPICS_DEFAULT=(
    /ouster/imu
    /Odometry
    /slam/path
    /tf
    /tf_static
    /mavros/vision_pose/pose
    /mavros/local_position/pose
    /mavros/local_position/velocity_local
    /mavros/state
    /mavros/battery
    /mavros/imu/data
    /mavros/imu/data_raw
    /mavros/rc/in
    /mavros/rc/out
    /mavros/esc_telemetry
    /mavros/esc_status
    /mavros/global_position/global
    /mavros/mission/waypoints
)

# Function to detect ROS version from container
detect_ros_version() {
    local container=$1
    if docker exec "$container" which ros2 &>/dev/null; then
        echo "ros2"
    elif docker exec "$container" which rosbag &>/dev/null; then
        echo "ros1"
    else
        echo "unknown"
    fi
}

# Function to detect SLAM odometry topic for ROS1
detect_slam_odometry_ros1() {
    local container=$1
    # Try common topic patterns
    for topic in /Odometry /lio_sam/mapping/odometry /fast_lio/odometry /lio/odometry; do
        if docker exec "$container" bash -c "rostopic list 2>/dev/null | grep -q '^${topic}$'" 2>/dev/null; then
            echo "$topic"
            return
        fi
    done
    echo "/Odometry"  # Fallback default
}

# Function to generate next flight number
get_next_flight_number() {
    local max_num=0
    if [ -d "$FLIGHTS_ROOT" ]; then
        for dir in "$FLIGHTS_ROOT"/[0-9]*_*; do
            if [ -d "$dir" ]; then
                local num=$(basename "$dir" | cut -d_ -f1)
                if [ "$num" -gt "$max_num" ]; then
                    max_num=$num
                fi
            fi
        done
    fi
    printf "%03d" $((max_num + 1))
}

# Function to snapshot configuration files
snapshot_configs() {
    local flight_dir=$1
    local container=$2
    local ros_version=$3
    local snapshot_dir="$flight_dir/config_snapshot"

    mkdir -p "$snapshot_dir"

    echo "Snapshotting configurations..."

    # Copy SLAM config based on ROS version
    if [ "$ros_version" = "ros2" ]; then
        # ROS 2 configs from slam_mavros
        docker exec "$container" bash -c "[ -f /opt/slam_ws/config/fast_lio_gpu.yaml ]" 2>/dev/null && \
            docker cp "$container:/opt/slam_ws/config/fast_lio_gpu.yaml" "$snapshot_dir/" 2>/dev/null || true
        docker exec "$container" bash -c "[ -f /ws/config/fast_lio_gpu.yaml ]" 2>/dev/null && \
            docker cp "$container:/ws/config/fast_lio_gpu.yaml" "$snapshot_dir/" 2>/dev/null || true
    else
        # ROS 1 configs from slam_system
        docker exec "$container" bash -c "find /root/slam_ws/src -name '*.yaml' -path '*/config/*'" 2>/dev/null | while read -r config; do
            docker cp "$container:$config" "$snapshot_dir/" 2>/dev/null || true
        done
    fi

    # Copy Ouster driver config
    for container_check in slam_mavros slam_system slam_gpu_system; do
        if docker ps --format '{{.Names}}' | grep -q "^${container_check}$"; then
            docker exec "$container_check" bash -c "find /opt/slam_ws/config /ws/config /root/slam_ws/src -name '*ouster*.yaml' 2>/dev/null" 2>/dev/null | while read -r config; do
                docker cp "$container_check:$config" "$snapshot_dir/$(basename $config)" 2>/dev/null || true
            done
        fi
    done

    # Copy URDF/robot description
    for container_check in slam_mavros slam_system; do
        if docker ps --format '{{.Names}}' | grep -q "^${container_check}$"; then
            docker exec "$container_check" bash -c "find /ws/config /opt/slam_ws/config -name '*.urdf' 2>/dev/null" 2>/dev/null | while read -r urdf; do
                docker cp "$container_check:$urdf" "$snapshot_dir/$(basename $urdf)" 2>/dev/null || true
            done
        fi
    done

    # Dump ArduPilot parameters if MAVROS is running
    if docker ps --format '{{.Names}}' | grep -q "^slam_mavros$"; then
        if [ "$ros_version" = "ros2" ]; then
            timeout 5 docker exec slam_mavros bash -c "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=1 && ros2 service call /mavros/param/pull std_srvs/srv/Trigger" &>/dev/null || true
            sleep 1
            timeout 5 docker exec slam_mavros bash -c "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=1 && ros2 run mavros mavparamget -n /mavros -o /tmp/ardupilot_params.parm" &>/dev/null && \
                docker cp slam_mavros:/tmp/ardupilot_params.parm "$snapshot_dir/" 2>/dev/null || true
        fi
    fi

    echo "Config snapshot saved to $snapshot_dir"
}

# Function to create flight metadata
create_metadata() {
    local flight_dir=$1
    local notes=$2
    local ros_version=$3
    local full_mode=$4
    local topics=$5

    cat > "$flight_dir/metadata.yaml" <<EOF
flight_number: $(basename "$flight_dir" | cut -d_ -f1)
timestamp: $(date -Iseconds)
ros_version: $ros_version
recording_mode: $([ "$full_mode" = "true" ] && echo "full" || echo "default")
notes: "$notes"
topics_recorded:
$(echo "$topics" | tr ' ' '\n' | sed 's/^/  - /')
status: recording
start_time: $(date -Iseconds)
EOF
}

# Function to update metadata on stop
update_metadata() {
    local flight_dir=$1
    local bag_dir="$flight_dir/bag"

    # Calculate duration from bag file
    local duration="unknown"
    local bag_size="unknown"

    if [ -d "$bag_dir" ]; then
        # Calculate total size
        bag_size=$(du -sh "$bag_dir" | cut -f1)

        # Try to get duration from bag info (ROS 2 mcap or ROS 1 bag)
        if ls "$bag_dir"/*.mcap &>/dev/null; then
            # ROS 2 mcap - extract duration if ros2 bag info is available
            local flight_name=$(basename "$flight_dir")
            local container_bag_dir="/ws/flights/${flight_name}/bag"
            local bag_filename=$(basename $(ls "$bag_dir"/*.mcap | head -1))
            local container_bag_path="${container_bag_dir}/${bag_filename}"
            duration=$(timeout 10 docker exec slam_mavros bash -c "source /opt/ros/humble/setup.bash && ros2 bag info '$container_bag_path' 2>/dev/null | grep 'Duration:' | awk '{print \$2}'" 2>/dev/null || echo "unknown")
        elif ls "$bag_dir"/*.bag &>/dev/null; then
            # ROS 1 bag - Note: slam_system container doesn't have flights mount yet
            # Fall back to host-side rosbag info if available, or skip
            duration="unknown"
        fi
    fi

    cat >> "$flight_dir/metadata.yaml" <<EOF
stop_time: $(date -Iseconds)
duration: $duration
bag_size: $bag_size
status: completed
EOF
}

# Function to update flight index
update_flight_index() {
    local flight_dir=$1
    local flight_num=$(basename "$flight_dir" | cut -d_ -f1)
    local timestamp=$(date -Iseconds)

    # Create index if it doesn't exist
    if [ ! -f "$INDEX_FILE" ]; then
        echo "flights:" > "$INDEX_FILE"
    fi

    # Append flight entry
    cat >> "$INDEX_FILE" <<EOF
- flight_number: $flight_num
  directory: $(basename "$flight_dir")
  timestamp: $timestamp
  path: $flight_dir
EOF
}

# Start recording
do_start() {
    # Parse arguments
    local notes=""
    local full_mode=false
    local ros_version=""

    while [ $# -gt 0 ]; do
        case "$1" in
            --notes)
                notes="$2"
                shift 2
                ;;
            --full)
                full_mode=true
                shift
                ;;
            --ros1)
                ros_version="ros1"
                shift
                ;;
            *)
                echo "Unknown option: $1"
                return 1
                ;;
        esac
    done

    # Check if already recording
    if [ -f "$PID_FILE" ]; then
        local container_pid=$(cat "$PID_FILE")
        local container="${container_pid%%:*}"
        local recorder_pid="${container_pid##*:}"

        if [ -n "$recorder_pid" ] && [ -n "$container" ] && docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
            echo "Recording already in progress"
            echo "  Container: $container"
            echo "  Recorder PID: $recorder_pid"
            [ -f "$CURRENT_FLIGHT_FILE" ] && echo "  Flight: $(cat $CURRENT_FLIGHT_FILE)"
            return 1
        else
            echo "Stale PID file found, removing..."
            rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        fi
    fi

    # Auto-detect ROS version if not specified
    if [ -z "$ros_version" ]; then
        if docker ps --format '{{.Names}}' | grep -q "^slam_mavros$"; then
            ros_version=$(detect_ros_version slam_mavros)
        elif docker ps --format '{{.Names}}' | grep -q "^slam_system$"; then
            ros_version=$(detect_ros_version slam_system)
        else
            echo "Error: No SLAM container running (slam_mavros or slam_system)"
            return 1
        fi

        if [ "$ros_version" = "unknown" ]; then
            echo "Error: Could not detect ROS version"
            return 1
        fi
        echo "Detected ROS version: $ros_version"
    fi

    # Set container and topics based on ROS version
    local container
    local topics
    if [ "$ros_version" = "ros2" ]; then
        container="slam_mavros"
        topics="${ROS2_TOPICS_DEFAULT[*]}"
    else
        container="slam_system"
        # Auto-detect SLAM odometry topic
        local slam_odom=$(detect_slam_odometry_ros1 "$container")
        topics="${ROS1_TOPICS_DEFAULT[*]}"
        # Replace /Odometry with detected topic
        topics="${topics/\/Odometry/$slam_odom}"
    fi

    # Add point cloud if full mode
    if [ "$full_mode" = "true" ]; then
        topics="$topics /ouster/points"
        echo "Full mode enabled - recording point cloud data"
    fi

    # Create flight directory
    mkdir -p "$FLIGHTS_ROOT"
    local flight_num=$(get_next_flight_number)
    local flight_name="${flight_num}_$(date +%Y%m%d_%H%M%S)"
    local flight_dir="$FLIGHTS_ROOT/$flight_name"
    local bag_dir="$flight_dir/bag"

    mkdir -p "$bag_dir"
    mkdir -p "$flight_dir/reports"

    echo "Starting recording: $flight_name"
    echo "Container: $container"
    echo "ROS version: $ros_version"
    echo "Topics: ${#topics[@]} topics"

    # Snapshot configurations
    snapshot_configs "$flight_dir" "$container" "$ros_version"

    # Create metadata
    create_metadata "$flight_dir" "$notes" "$ros_version" "$full_mode" "$topics"

    # Compute container-side bag directory path
    local container_bag_dir
    if [ "$ros_version" = "ros2" ]; then
        if [ "$container" = "slam_mavros" ]; then
            container_bag_dir="/ws/flights/${flight_name}/bag"
        elif [ "$container" = "slam_gpu_system" ]; then
            container_bag_dir="/opt/slam_ws/flights/${flight_name}/bag"
        else
            echo "Error: Unknown ROS 2 container: $container"
            return 1
        fi
    else
        # ROS 1 slam_system - doesn't have flights mount yet, would need to add volume mount
        echo "Error: ROS 1 container doesn't support flight recording yet (no volume mount)"
        return 1
    fi

    # Start recording in background
    set +e  # Disable exit-on-error for this section
    if [ "$ros_version" = "ros2" ]; then
        # ROS 2 bag record (mcap format)
        docker exec -d "$container" bash -c "
            source /opt/ros/humble/setup.bash && \
            export ROS_DOMAIN_ID=1 && \
            cd $container_bag_dir && \
            ros2 bag record -o recording $topics > /tmp/recorder.log 2>&1
        "
    else
        # ROS 1 bag record
        docker exec -d "$container" bash -c "
            source /opt/ros/noetic/setup.bash && \
            cd $container_bag_dir && \
            rosbag record -O recording.bag $topics > /tmp/recorder.log 2>&1
        "
    fi
    set -e  # Re-enable exit-on-error

    # Wait for recorder to start and find its PID inside container
    sleep 3

    local recorder_pid
    if [ "$ros_version" = "ros2" ]; then
        recorder_pid=$(docker exec "$container" bash -c "pgrep -f 'ros2 bag record' | head -1" 2>/dev/null)
    else
        recorder_pid=$(docker exec "$container" bash -c "pgrep -f 'rosbag record' | head -1" 2>/dev/null)
    fi

    if [ -z "$recorder_pid" ]; then
        echo "✗ Recording failed to start"
        docker exec "$container" tail -20 /tmp/recorder.log 2>/dev/null || true
        rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        return 1
    fi

    # Save container:PID and current flight
    echo "$container:$recorder_pid" > "$PID_FILE"
    echo "$flight_dir" > "$CURRENT_FLIGHT_FILE"

    echo "✓ Recording started successfully"
    echo "  Flight directory: $flight_dir"
    echo "  Container: $container"
    echo "  Recorder PID (in container): $recorder_pid"
    echo "  Stop with: $(basename $0) stop"
}

# Stop recording
do_stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "No recording in progress"
        return 1
    fi

    local container_pid=$(cat "$PID_FILE")
    local flight_dir=$(cat "$CURRENT_FLIGHT_FILE")

    # Parse container:pid format
    local container="${container_pid%%:*}"
    local recorder_pid="${container_pid##*:}"

    if [ -z "$recorder_pid" ] || [ -z "$container" ]; then
        echo "Invalid PID file format: $container_pid"
        rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        return 1
    fi

    # Check if process still exists in container
    if ! docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "Recording process not found (container: $container, PID: $recorder_pid)"
        echo "Flight directory: $flight_dir"
        rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        return 1
    fi

    echo "Stopping recording..."
    echo "  Container: $container"
    echo "  Recorder PID: $recorder_pid"
    echo "  Flight: $(basename $flight_dir)"

    # Send SIGINT to stop recording gracefully (inside container)
    docker exec "$container" kill -INT "$recorder_pid" 2>/dev/null || true

    # Wait for process to finish (up to 10 seconds)
    local timeout=10
    while [ $timeout -gt 0 ] && docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; do
        sleep 1
        timeout=$((timeout - 1))
    done

    if docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "Force killing recording process..."
        docker exec "$container" kill -9 "$recorder_pid" 2>/dev/null || true
    fi

    # Update metadata
    update_metadata "$flight_dir"

    # Update flight index
    update_flight_index "$flight_dir"

    # Clean up
    rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"

    echo "✓ Recording stopped"
    echo "  Flight directory: $flight_dir"

    # Show bag file info
    local bag_size=$(du -sh "$flight_dir/bag" | cut -f1)
    echo "  Bag size: $bag_size"
}

# Show status
do_status() {
    if [ ! -f "$PID_FILE" ]; then
        echo "No recording in progress"
        return 0
    fi

    local container_pid=$(cat "$PID_FILE")
    local flight_dir=$(cat "$CURRENT_FLIGHT_FILE")

    # Parse container:pid format
    local container="${container_pid%%:*}"
    local recorder_pid="${container_pid##*:}"

    if docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "Recording in progress"
        echo "  Container: $container"
        echo "  Recorder PID: $recorder_pid"
        echo "  Flight: $(basename $flight_dir)"
        echo "  Directory: $flight_dir"

        # Calculate recording duration
        if [ -f "$flight_dir/metadata.yaml" ]; then
            local start_time=$(grep "start_time:" "$flight_dir/metadata.yaml" | cut -d' ' -f2)
            local now=$(date -Iseconds)
            echo "  Started: $start_time"
        fi

        # Show current bag size
        if [ -d "$flight_dir/bag" ]; then
            local bag_size=$(du -sh "$flight_dir/bag" | cut -f1)
            echo "  Current size: $bag_size"
        fi
    else
        echo "Recording process not found (container: $container, PID: $recorder_pid)"
        echo "Stale PID file - run 'stop' to clean up"
    fi
}

# List all flights
do_list() {
    if [ ! -d "$FLIGHTS_ROOT" ] || [ -z "$(ls -A $FLIGHTS_ROOT/[0-9]*_* 2>/dev/null)" ]; then
        echo "No flights recorded"
        return 0
    fi

    echo "Recorded flights:"
    echo "NUM  DATE       TIME     SIZE    DURATION  NOTES"
    echo "---  ---------- -------- ------- --------- -----"

    for dir in "$FLIGHTS_ROOT"/[0-9]*_*/; do
        if [ -d "$dir" ]; then
            local num=$(basename "$dir" | cut -d_ -f1)
            local date_part=$(basename "$dir" | cut -d_ -f2)
            local time_part=$(basename "$dir" | cut -d_ -f3)
            local date_fmt="${date_part:0:4}-${date_part:4:2}-${date_part:6:2}"
            local time_fmt="${time_part:0:2}:${time_part:2:2}:${time_part:4:2}"

            local size="-"
            local duration="-"
            local notes="-"

            if [ -d "$dir/bag" ]; then
                size=$(du -sh "$dir/bag" | cut -f1)
            fi

            if [ -f "$dir/metadata.yaml" ]; then
                duration=$(grep "^duration:" "$dir/metadata.yaml" | cut -d' ' -f2 || echo "-")
                notes=$(grep "^notes:" "$dir/metadata.yaml" | cut -d'"' -f2 || echo "-")
                [ -z "$notes" ] && notes="-"
            fi

            printf "%-4s %-10s %-8s %-7s %-9s %s\n" "$num" "$date_fmt" "$time_fmt" "$size" "$duration" "$notes"
        fi
    done
}

# Show last flight info
do_last() {
    local last_dir=$(ls -dt "$FLIGHTS_ROOT"/[0-9]*_*/ 2>/dev/null | head -1)

    if [ -z "$last_dir" ]; then
        echo "No flights recorded"
        return 1
    fi

    echo "Last flight: $(basename $last_dir)"
    echo ""

    if [ -f "$last_dir/metadata.yaml" ]; then
        cat "$last_dir/metadata.yaml"
    else
        echo "No metadata found"
    fi

    echo ""
    echo "Files:"
    ls -lh "$last_dir/bag/" 2>/dev/null || echo "  No bag files"
}

# Clean old flights
do_clean() {
    local keep_last=5

    if [ "$1" = "--keep-last" ] && [ -n "$2" ]; then
        keep_last=$2
    else
        echo "Usage: $0 clean --keep-last N"
        return 1
    fi

    local flights=($(ls -dt "$FLIGHTS_ROOT"/[0-9]*_*/ 2>/dev/null))
    local total=${#flights[@]}

    if [ $total -le $keep_last ]; then
        echo "Only $total flights exist, keeping all (threshold: $keep_last)"
        return 0
    fi

    local to_remove=$((total - keep_last))
    echo "Removing $to_remove old flights (keeping last $keep_last)..."

    for ((i=keep_last; i<total; i++)); do
        local dir="${flights[$i]}"
        echo "  Removing: $(basename $dir)"
        rm -rf "$dir"
    done

    echo "✓ Cleanup complete"
}

# Main command dispatcher
case "${1:-}" in
    start)
        shift
        do_start "$@"
        ;;
    stop)
        do_stop
        ;;
    status)
        do_status
        ;;
    list)
        do_list
        ;;
    last)
        do_last
        ;;
    clean)
        shift
        do_clean "$@"
        ;;
    *)
        echo "Flight Data Recorder"
        echo ""
        echo "Usage: $0 <command> [options]"
        echo ""
        echo "Commands:"
        echo "  start [--notes \"text\"] [--full] [--ros1]"
        echo "      Start recording a new flight"
        echo "      --notes: Optional notes about the flight"
        echo "      --full:  Record point cloud data (high bandwidth)"
        echo "      --ros1:  Force ROS 1 mode (auto-detects by default)"
        echo ""
        echo "  stop"
        echo "      Stop current recording and finalize metadata"
        echo ""
        echo "  status"
        echo "      Show current recording status"
        echo ""
        echo "  list"
        echo "      List all recorded flights"
        echo ""
        echo "  last"
        echo "      Show details of the last recorded flight"
        echo ""
        echo "  clean --keep-last N"
        echo "      Remove old flights, keeping the last N"
        echo ""
        ;;
esac
