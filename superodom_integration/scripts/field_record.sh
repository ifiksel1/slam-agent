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

# Scan a finished bag for sensor dropouts. A simultaneous gap in BOTH /ouster/points and
# /ouster/imu = the single Ouster ethernet link dropped (e.g. the tether stretched as you
# walked) — that blackout makes the take useless for SLAM (any odometry diverges on resume).
# Returns 0 = clean, 1 = dropout. $1 = container bag dir.
check_gaps(){
  docker exec -i "$CTR" bash -lc "$SRC && python3 - '$1'" <<'PY'
import sqlite3, glob, os, sys
dbs=glob.glob(os.path.join(sys.argv[1],'*.db3'))
if not dbs: print("  GAP-CHECK: no db3 found"); sys.exit(2)
cur=sqlite3.connect(dbs[0]).cursor()
def scan(topic, thr):
    r=cur.execute("select id from topics where name=?",(topic,)).fetchone()
    if not r: return (topic,0,0,0,0)
    ts=[x[0] for x in cur.execute("select timestamp from messages where topic_id=? order by timestamp",(r[0],))]
    n=len(ts)
    if n<2: return (topic,n,0,0,0)
    rel=[(t-ts[0])/1e9 for t in ts]
    worst=worst_t=nbig=0
    for i in range(len(rel)-1):
        dt=rel[i+1]-rel[i]
        if dt>thr: nbig+=1
        if dt>worst: worst, worst_t = dt, rel[i]
    return (topic,n,worst,worst_t,nbig)
bad=empty=False
for topic,thr in (('/ouster/points',0.15),('/ouster/imu',0.05)):
    _,n,worst,wt,nbig=scan(topic,thr)
    if n<10:
        empty=True; print(f"  ⚠ {topic}: only {n} msgs — sensor was NOT streaming")
    elif worst>0.3:
        bad=True; print(f"  ⚠ {topic}: {worst*1000:.0f}ms DROPOUT at t={wt:.1f}s ({nbig} gaps)")
    else:
        print(f"  ✓ {topic}: {n} msgs, max gap {worst*1000:.0f}ms (clean)")
print("  VERDICT: ⚠⚠ RE-RECORD — NO sensor data (driver wasn't publishing)" if empty
      else "  VERDICT: ⚠⚠ RE-RECORD — sensor blacked out (check the LiDAR cable/tether)" if bad
      else "  VERDICT: ✓ CLEAN — good for SLAM benchmarking")
sys.exit(1 if (bad or empty) else 0)
PY
}

# tell the terminal to STOP sending focus-in/out (ESC[I/ESC[O) + bracketed-paste markers,
# which otherwise get captured into the read prompts and corrupt the bag folder name.
printf '\033[?1004l\033[?2004l'
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
if [ -z "$RATE" ]; then
  warn "  ⛔ /ouster/points is NOT publishing — the sensor isn't streaming, so recording would capture NOTHING."
  warn "     (The Ouster HTTP config can succeed while the UDP data stream fails — e.g. just power-cycled, or the cable/tether dropped.)"
  warn "     Fix: confirm the LiDAR is powered + cabled, wait ~20s, check /tmp/field_driver.log, then re-run. Refusing to record into a dead stream."
  exit 1
fi
echo "  driver: $RATE  (streaming ✓)"
FREEGB=$(df --output=avail -BG "$FIELD_HOST" | tail -1 | tr -d 'G ')
echo "  disk free: ${FREEGB} GB  (~$((FREEGB/3)) min of raw recording)"
[ "${FREEGB:-0}" -lt 15 ] && warn "  LOW DISK — clear space before recording!"

# ---------- RECORD LOOP ----------
say "Ready. For each spot: type a name, walk (LOOP back to start!), press ENTER to stop."
while true; do
  echo
  read -r -p "Environment name (blank = finish): " ENV
  # strip ANSI/control junk (terminal focus events like ESC[O / ESC[I land in stdin) + trim
  ENV=$(printf '%s' "$ENV" | sed $'s/\x1b\\[[0-9;?]*[A-Za-z]//g' | tr -cd '[:alnum:][:space:],._-' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
  [ -z "$ENV" ] && break
  STAMP=$(date +%Y%m%d_%H%M%S)
  NAME="${ENV// /_}_$STAMP"
  read -r -p "  Stand at your START point, then press ENTER to begin recording '$ENV'..." _
  # verify the sensor is ACTUALLY streaming right now — it can drop between clips (battery/tether),
  # and recording a dead stream silently produces a 0-message bag. Don't let that happen.
  if ! docker exec "$CTR" bash -lc "export RMW_IMPLEMENTATION=$RMW; $SRC && timeout 5 ros2 topic hz /ouster/points 2>/dev/null | grep -q 'average rate'"; then
    warn "  ⛔ /ouster/points not flowing — the sensor dropped. Skipping this clip; check the LiDAR power/cable, then pick the spot again."
    continue
  fi
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
  echo "  checking for sensor dropouts (the tether/connection blackout)..."
  if check_gaps "$FIELD_CTR/$NAME"; then GAP=clean; else GAP=DROPOUT; warn "  ^^ RE-WALK this spot — any SLAM will diverge on a blacked-out take."; fi
  read -r -p "  Did you return to the exact start point? (y/n + any notes): " NOTE
  # log INSIDE the container (it owns the root:root field dir; a host echo as 'dev' gets Permission denied)
  docker exec -i "$CTR" bash -c "cat >> $FIELD_CTR/field_log.txt" <<EOF
$NAME | size=$SZ | $DUR | gaps: $GAP | returned_to_start: $NOTE
EOF
done

say "Done. $(ls -1d "$FIELD_HOST"/*/ 2>/dev/null | wc -l) bag(s) in $FIELD_HOST"
echo "Back on WiFi, tell the agent: \"process the field bags\" — it will build maps + evaluate each."
