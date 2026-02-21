#!/bin/bash
# Flight Data Recorder - Manages ROS bag recording for SLAM flights
# Usage: ./flight_recorder.sh {start|stop|status|list|last|clean}

FLIGHTS_ROOT="/home/dev/slam-agent/flights"
PID_FILE="/tmp/flight_recorder.pid"
LOCK_FILE="/tmp/flight_recorder.lock"
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
    for topic in /Odometry /lio_sam/mapping/odometry /fast_lio/odometry /lio/odometry; do
        if docker exec "$container" bash -c "rostopic list 2>/dev/null | grep -q '^${topic}$'" 2>/dev/null; then
            echo "$topic"
            return
        fi
    done
    echo "/Odometry"  # Fallback default
}

# Function to generate next flight number (fix: force base-10 to avoid octal parsing)
get_next_flight_number() {
    local max_num=0
    if [ -d "$FLIGHTS_ROOT" ]; then
        for dir in "$FLIGHTS_ROOT"/[0-9]*_*; do
            if [ -d "$dir" ]; then
                local num=$((10#$(basename "$dir" | cut -d_ -f1)))
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
        docker exec "$container" bash -c "[ -f /opt/slam_ws/config/fast_lio_gpu.yaml ]" 2>/dev/null && \
            docker cp "$container:/opt/slam_ws/config/fast_lio_gpu.yaml" "$snapshot_dir/" 2>/dev/null || true
        docker exec "$container" bash -c "[ -f /ws/config/fast_lio_gpu.yaml ]" 2>/dev/null && \
            docker cp "$container:/ws/config/fast_lio_gpu.yaml" "$snapshot_dir/" 2>/dev/null || true
    else
        # ROS 1 configs from slam_system
        while IFS= read -r config; do
            [ -n "$config" ] && docker cp "$container:$config" "$snapshot_dir/" 2>/dev/null || true
        done < <(docker exec "$container" bash -c "find /root/slam_ws/src -name '*.yaml' -path '*/config/*'" 2>/dev/null)
    fi

    # Copy Ouster driver config
    for container_check in slam_mavros slam_system slam_gpu_system; do
        if docker ps --format '{{.Names}}' | grep -q "^${container_check}$"; then
            while IFS= read -r config; do
                [ -n "$config" ] && docker cp "$container_check:$config" "$snapshot_dir/$(basename "$config")" 2>/dev/null || true
            done < <(docker exec "$container_check" bash -c "find /opt/slam_ws/config /ws/config /root/slam_ws/src -name '*ouster*.yaml' 2>/dev/null" 2>/dev/null)
        fi
    done

    # Copy URDF/robot description
    for container_check in slam_mavros slam_system; do
        if docker ps --format '{{.Names}}' | grep -q "^${container_check}$"; then
            while IFS= read -r urdf; do
                [ -n "$urdf" ] && docker cp "$container_check:$urdf" "$snapshot_dir/$(basename "$urdf")" 2>/dev/null || true
            done < <(docker exec "$container_check" bash -c "find /ws/config /opt/slam_ws/config -name '*.urdf' 2>/dev/null" 2>/dev/null)
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

    local count
    count=$(ls "$snapshot_dir" 2>/dev/null | wc -l)
    echo "Config snapshot saved to $snapshot_dir ($count files)"
}

# Function to create flight metadata
create_metadata() {
    local flight_dir=$1
    local notes=$2
    local ros_version=$3
    local full_mode=$4
    shift 4
    local topics=("$@")   # receive topics as individual array elements

    {
        echo "flight_number: $(basename "$flight_dir" | cut -d_ -f1)"
        echo "timestamp: $(date -Iseconds)"
        echo "ros_version: $ros_version"
        echo "recording_mode: $([ "$full_mode" = "true" ] && echo "full" || echo "default")"
        echo "notes: \"$notes\""
        echo "topics_recorded:"
        for t in "${topics[@]}"; do
            echo "  - $t"
        done
        echo "status: recording"
        echo "start_time: $(date -Iseconds)"
    } > "$flight_dir/metadata.yaml"
}

# Function to update metadata on stop (replaces status line, appends stop fields)
update_metadata() {
    local flight_dir=$1
    local bag_dir="$flight_dir/bag"

    local duration="unknown"
    local bag_size="unknown"

    if [ -d "$bag_dir" ]; then
        bag_size=$(du -sh "$bag_dir" | cut -f1)

        # Try to get duration via ros2 bag info on the bag *directory*
        if ls "$bag_dir"/*.mcap &>/dev/null || ls "$bag_dir"/*.db3 &>/dev/null; then
            local flight_name
            flight_name=$(basename "$flight_dir")
            local container_bag_dir="/ws/flights/${flight_name}/bag"
            duration=$(timeout 10 docker exec slam_mavros bash -c \
                "source /opt/ros/humble/setup.bash && ros2 bag info '${container_bag_dir}' 2>/dev/null | grep 'Duration:' | awk '{print \$2}'" \
                2>/dev/null || echo "unknown")
        fi
    fi

    # Replace the status line in-place, then append stop fields
    sed -i 's/^status: recording$/status: completed/' "$flight_dir/metadata.yaml"
    {
        echo "stop_time: $(date -Iseconds)"
        echo "duration: $duration"
        echo "bag_size: $bag_size"
    } >> "$flight_dir/metadata.yaml"
}

# Function to update flight index
update_flight_index() {
    local flight_dir=$1
    local flight_num
    flight_num=$(basename "$flight_dir" | cut -d_ -f1)
    local timestamp
    timestamp=$(date -Iseconds)

    if [ ! -f "$INDEX_FILE" ]; then
        echo "flights:" > "$INDEX_FILE"
    fi

    {
        echo "- flight_number: $flight_num"
        echo "  directory: $(basename "$flight_dir")"
        echo "  timestamp: $timestamp"
        echo "  path: $flight_dir"
    } >> "$INDEX_FILE"
}

# Check available disk space (returns available GB)
check_disk_space() {
    df --output=avail "$FLIGHTS_ROOT" 2>/dev/null | tail -1 | awk '{printf "%d", $1 / 1048576}'
}

# Start recording
do_start() {
    # Acquire exclusive lock to prevent concurrent start invocations (fix: race condition)
    exec 200>"$LOCK_FILE"
    if ! flock -n 200; then
        echo "Error: Another flight_recorder instance is already running"
        return 1
    fi

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
            --ros2)
                ros_version="ros2"
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
        local container_pid
        container_pid=$(cat "$PID_FILE")
        local container="${container_pid%%:*}"
        local recorder_pid="${container_pid##*:}"

        if [ -n "$recorder_pid" ] && [ -n "$container" ] && docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
            echo "Recording already in progress"
            echo "  Container: $container"
            echo "  Recorder PID: $recorder_pid"
            [ -f "$CURRENT_FLIGHT_FILE" ] && echo "  Flight: $(cat "$CURRENT_FLIGHT_FILE")"
            return 1
        else
            echo "Stale PID file found, removing..."
            rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        fi
    fi

    # Disk space check before allocating anything
    mkdir -p "$FLIGHTS_ROOT"
    local avail_gb
    avail_gb=$(check_disk_space)
    if [ "$full_mode" = "true" ] && [ "$avail_gb" -lt 10 ]; then
        echo "Error: Only ${avail_gb}GB free. Full mode requires at least 10GB."
        return 1
    elif [ "$avail_gb" -lt 2 ]; then
        echo "Error: Only ${avail_gb}GB free. Need at least 2GB to start recording."
        return 1
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

    # Set container and topics array based on ROS version
    local container
    local topics   # this will be an array
    if [ "$ros_version" = "ros2" ]; then
        container="slam_mavros"
        topics=("${ROS2_TOPICS_DEFAULT[@]}")
    else
        container="slam_system"
        # Auto-detect SLAM odometry topic and substitute in-array (fix: exact element match, not substr)
        local slam_odom
        slam_odom=$(detect_slam_odometry_ros1 "$container")
        topics=()
        for t in "${ROS1_TOPICS_DEFAULT[@]}"; do
            if [ "$t" = "/Odometry" ]; then
                topics+=("$slam_odom")
            else
                topics+=("$t")
            fi
        done
    fi

    # Add point cloud if full mode
    if [ "$full_mode" = "true" ]; then
        topics+=("/ouster/points")
        echo "Full mode enabled - recording point cloud data"
    fi

    # Create flight directory
    local flight_num
    flight_num=$(get_next_flight_number)
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

    # Create metadata (pass topics as individual args)
    create_metadata "$flight_dir" "$notes" "$ros_version" "$full_mode" "${topics[@]}"

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
        # ROS 1 slam_system - requires volume mount for flights directory
        echo "Error: ROS 1 container doesn't support flight recording yet (no volume mount)"
        return 1
    fi

    # Build topic string for the bash -c command (topics array is safe — no spaces or metacharacters)
    local topics_str="${topics[*]}"

    # Write the recorder PID from *inside* the container so we get the actual process, not the wrapper
    docker exec "$container" bash -c "
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=1
        mkdir -p '${container_bag_dir}'
        cd '${container_bag_dir}'
        ros2 bag record -o recording ${topics_str} >/tmp/recorder.log 2>&1 &
        echo \$! > /tmp/recorder.pid
    "

    # Wait for recorder to start, then read the authoritative PID written by the process itself
    sleep 3

    local recorder_pid
    recorder_pid=$(docker exec "$container" cat /tmp/recorder.pid 2>/dev/null | tr -d '[:space:]')

    if [ -z "$recorder_pid" ] || ! docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "✗ Recording failed to start"
        docker exec "$container" tail -20 /tmp/recorder.log 2>/dev/null || true
        rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"
        return 1
    fi

    # Atomically save container:PID and current flight
    echo "$container:$recorder_pid" > "$PID_FILE"
    echo "$flight_dir" > "$CURRENT_FLIGHT_FILE"

    echo "✓ Recording started successfully"
    echo "  Flight directory: $flight_dir"
    echo "  Container: $container"
    echo "  Recorder PID (in container): $recorder_pid"
    echo "  Stop with: $(basename "$0") stop"
}

# Stop recording
do_stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "No recording in progress"
        return 1
    fi

    local container_pid
    container_pid=$(cat "$PID_FILE")

    # Guard: CURRENT_FLIGHT_FILE must exist (fix: missing existence check)
    if [ ! -f "$CURRENT_FLIGHT_FILE" ]; then
        echo "Warning: No current flight file found, cannot update metadata"
        rm -f "$PID_FILE"
        return 1
    fi

    local flight_dir
    flight_dir=$(cat "$CURRENT_FLIGHT_FILE")

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
    echo "  Flight: $(basename "$flight_dir")"

    # Send SIGINT to process group so the recorder finalises the bag (fix: kill may hit wrapper only)
    local pgid
    pgid=$(docker exec "$container" bash -c "ps -o pgid= -p $recorder_pid 2>/dev/null | tr -d ' '")
    if [ -n "$pgid" ] && [ "$pgid" != "0" ]; then
        docker exec "$container" bash -c "kill -INT -$pgid" 2>/dev/null || \
            docker exec "$container" kill -INT "$recorder_pid" 2>/dev/null || true
    else
        docker exec "$container" kill -INT "$recorder_pid" 2>/dev/null || true
    fi

    # Wait for process to finish (up to 10 seconds)
    local timeout=10
    while [ "$timeout" -gt 0 ] && docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; do
        sleep 1
        timeout=$((timeout - 1))
    done

    if docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "Force killing recording process..."
        docker exec "$container" kill -9 "$recorder_pid" 2>/dev/null || true
    fi

    # Update metadata and flight index
    update_metadata "$flight_dir"
    update_flight_index "$flight_dir"

    rm -f "$PID_FILE" "$CURRENT_FLIGHT_FILE"

    echo "✓ Recording stopped"
    echo "  Flight directory: $flight_dir"
    local bag_size
    bag_size=$(du -sh "$flight_dir/bag" 2>/dev/null | cut -f1)
    echo "  Bag size: $bag_size"
}

# Show status
do_status() {
    if [ ! -f "$PID_FILE" ]; then
        echo "No recording in progress"
        return 0
    fi

    local container_pid
    container_pid=$(cat "$PID_FILE")
    local flight_dir=""
    [ -f "$CURRENT_FLIGHT_FILE" ] && flight_dir=$(cat "$CURRENT_FLIGHT_FILE")

    local container="${container_pid%%:*}"
    local recorder_pid="${container_pid##*:}"

    if docker exec "$container" bash -c "kill -0 $recorder_pid" 2>/dev/null; then
        echo "Recording in progress"
        echo "  Container: $container"
        echo "  Recorder PID: $recorder_pid"
        [ -n "$flight_dir" ] && echo "  Flight: $(basename "$flight_dir")"
        [ -n "$flight_dir" ] && echo "  Directory: $flight_dir"

        if [ -n "$flight_dir" ] && [ -f "$flight_dir/metadata.yaml" ]; then
            local start_time
            start_time=$(grep "^start_time:" "$flight_dir/metadata.yaml" | cut -d' ' -f2)
            echo "  Started: $start_time"
        fi

        if [ -n "$flight_dir" ] && [ -d "$flight_dir/bag" ]; then
            local bag_size
            bag_size=$(du -sh "$flight_dir/bag" | cut -f1)
            echo "  Current size: $bag_size"
        fi
    else
        echo "Recording process not found (container: $container, PID: $recorder_pid)"
        echo "Stale PID file - run 'stop' to clean up"
    fi
}

# List all flights
do_list() {
    if [ ! -d "$FLIGHTS_ROOT" ] || [ -z "$(ls -A "$FLIGHTS_ROOT"/[0-9]*_* 2>/dev/null)" ]; then
        echo "No flights recorded"
        return 0
    fi

    echo "Recorded flights:"
    echo "NUM  DATE       TIME     SIZE    DURATION  NOTES"
    echo "---  ---------- -------- ------- --------- -----"

    for dir in "$FLIGHTS_ROOT"/[0-9]*_*/; do
        if [ -d "$dir" ]; then
            local num date_part time_part date_fmt time_fmt size duration notes
            num=$(basename "$dir" | cut -d_ -f1)
            date_part=$(basename "$dir" | cut -d_ -f2)
            time_part=$(basename "$dir" | cut -d_ -f3)
            date_fmt="${date_part:0:4}-${date_part:4:2}-${date_part:6:2}"
            time_fmt="${time_part:0:2}:${time_part:2:2}:${time_part:4:2}"

            size="-"
            duration="-"
            notes="-"

            [ -d "$dir/bag" ] && size=$(du -sh "$dir/bag" | cut -f1)

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
    local last_dir
    last_dir=$(ls -dt "$FLIGHTS_ROOT"/[0-9]*_*/ 2>/dev/null | head -1)

    if [ -z "$last_dir" ]; then
        echo "No flights recorded"
        return 1
    fi

    echo "Last flight: $(basename "$last_dir")"
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

# Clean old flights (fix: use find+process substitution instead of ls parsing)
do_clean() {
    local keep_last

    if [ "$1" = "--keep-last" ] && [ -n "$2" ]; then
        keep_last=$2
    elif [ -n "$1" ] && [[ "$1" =~ ^[0-9]+$ ]]; then
        # Also accept positional: flight_recorder.sh clean 5
        keep_last=$1
    else
        echo "Usage: $0 clean --keep-last N"
        return 1
    fi

    local -a flights=()
    while IFS= read -r -d '' dir; do
        flights+=("$dir")
    done < <(find "$FLIGHTS_ROOT" -maxdepth 1 -name '[0-9]*_*' -type d -print0 | sort -rz)

    local total=${#flights[@]}

    if [ "$total" -le "$keep_last" ]; then
        echo "Only $total flights exist, keeping all (threshold: $keep_last)"
        return 0
    fi

    local to_remove=$((total - keep_last))
    echo "Removing $to_remove old flights (keeping last $keep_last)..."

    for ((i=keep_last; i<total; i++)); do
        local dir="${flights[$i]}"
        echo "  Removing: $(basename "$dir")"
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
        echo "  start [--notes \"text\"] [--full] [--ros1|--ros2]"
        echo "      Start recording a new flight"
        echo "      --notes: Optional notes about the flight"
        echo "      --full:  Record point cloud data (high bandwidth, requires 10GB free)"
        echo "      --ros1:  Force ROS 1 mode"
        echo "      --ros2:  Force ROS 2 mode"
        echo "      (auto-detects ROS version by default)"
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
