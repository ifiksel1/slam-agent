#!/bin/bash
# FIELD DATA COLLECTION — record raw Ouster bags, one per environment.
# Designed to run OFFLINE from a local terminal on the Jetson (no WiFi / no agent needed).
# Records ONLY the raw sensor topics (/ouster/points + /ouster/imu + /tf_static) — everything
# SuperOdom/EllipseLIO needs to be re-run offline at home. Does NOT run SLAM live (keeps it
# robust + low-power; the map + evaluation happen at home where the agent can help).
#
# Workflow: run it, type an environment name, walk your route, press ENTER to stop, repeat.
#   - TIP: walk a LOOP and RETURN TO THE EXACT START so we can measure drift (loop closure).
#   - Clock can stay at STOCK to save battery — recording needs no SLAM compute.
#
# Bags land in ~/superodom_ws/field/<env>_<timestamp>/ (host-visible, gitignored).
# Usage: ./field_record.sh
set -u
CTR=superodom
RMW=rmw_cyclonedds_cpp
IMG=superodom:humble
SENSOR_IP=192.168.2.60
SRC='source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash'
FIELD_HOST="$HOME/superodom_ws/field"
FIELD_CTR=/root/ros2_ws/field
DRV_PARAMS=/root/ros2_ws/src/ouster_os1_64_driver.yaml
mkdir -p "$FIELD_HOST"

say(){ echo -e "\n\033[1;36m$*\033[0m"; }
warn(){ echo -e "\033[1;33m$*\033[0m"; }

# ---------- PREFLIGHT ----------
say "=== FIELD RECORDER preflight ==="
ping -c1 -W2 "$SENSOR_IP" >/dev/null 2>&1 && echo "  sensor $SENSOR_IP: UP" || { warn "  sensor $SENSOR_IP NOT reachable — check the Ouster is powered + cabled"; exit 1; }

# container up? (create from the fixed image, with --init, if missing)
if ! docker ps --format '{{.Names}}' | grep -qx "$CTR"; then
  warn "  container '$CTR' not running — creating from $IMG"
  docker rm -f "$CTR" >/dev/null 2>&1 || true
  docker run -d --name "$CTR" --init --privileged --net=host --ipc=host --shm-size=4gb \
    -e ROS_DOMAIN_ID=0 -v "$HOME/superodom_ws:/root/ros2_ws" "$IMG" sleep infinity >/dev/null
  sleep 3
fi
echo "  container: up"

# driver publishing /ouster/points? start it if not
if ! docker exec "$CTR" bash -lc "$SRC && timeout 4 ros2 topic list 2>/dev/null" | grep -q /ouster/points; then
  echo "  starting Ouster driver (CycloneDDS)..."
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 launch ouster_ros driver.launch.py params_file:=$DRV_PARAMS viz:=false > /tmp/field_driver.log 2>&1"
  for w in $(seq 1 20); do
    docker exec "$CTR" bash -lc "$SRC && timeout 3 ros2 topic list 2>/dev/null" | grep -q /ouster/points && break
    sleep 3
  done
fi
RATE=$(docker exec "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && timeout 6 ros2 topic hz /ouster/points 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1")
echo "  driver: ${RATE:-NO POINTS — check sensor!}"
FREEGB=$(df --output=avail -BG "$FIELD_HOST" | tail -1 | tr -d 'G ')
echo "  disk free: ${FREEGB} GB  (~$((FREEGB/3)) min of raw recording)"
[ "${FREEGB:-0}" -lt 15 ] && warn "  LOW DISK — clear space before recording!"

# ---------- RECORD LOOP ----------
say "Ready. For each spot: type a name, walk (LOOP back to start!), press ENTER to stop."
while true; do
  echo
  read -r -p "Environment name (blank = finish): " ENV
  [ -z "$ENV" ] && break
  STAMP=$(date +%Y%m%d_%H%M%S)
  NAME="${ENV// /_}_$STAMP"
  read -r -p "  Stand at your START point, then press ENTER to begin recording '$ENV'..." _
  docker exec -d "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && ros2 bag record -o $FIELD_CTR/$NAME /ouster/points /ouster/imu /tf_static > /tmp/field_rec_$STAMP.log 2>&1"
  sleep 2
  if docker exec "$CTR" bash -c "pgrep -f 'bag record' >/dev/null"; then
    echo -e "  \033[1;32m● RECORDING '$ENV'\033[0m — walk your route, then RETURN TO START. Press ENTER to STOP."
  else
    warn "  recorder did not start — see /tmp/field_rec_$STAMP.log"; continue
  fi
  read -r _
  docker exec "$CTR" bash -c "pkill -INT -f 'bag record'" 2>/dev/null   # SIGINT = clean DB close + metadata.yaml
  sleep 3
  SZ=$(du -sh "$FIELD_HOST/$NAME" 2>/dev/null | cut -f1)
  DUR=$(docker exec "$CTR" bash -lc "$SRC && ros2 bag info $FIELD_CTR/$NAME 2>/dev/null | grep -oE 'Duration:[^ ]*[0-9.]+s' | head -1")
  echo "  saved '$ENV': size=$SZ  $DUR"
  read -r -p "  Did you return to the exact start point? (y/n + any notes): " NOTE
  echo "$NAME | size=$SZ | $DUR | returned_to_start: $NOTE" >> "$FIELD_HOST/field_log.txt"
done

say "Done. $(ls -1d "$FIELD_HOST"/*/ 2>/dev/null | wc -l) bag(s) in $FIELD_HOST"
echo "Back on WiFi, tell the agent: \"process the field bags\" — it will build maps + evaluate each."
