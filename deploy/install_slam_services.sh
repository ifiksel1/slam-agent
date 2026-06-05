#!/bin/bash
# Install + enable the SLAM boot services (run with sudo):
#   sudo deploy/install_slam_services.sh
#
# Creates:
#   slam-container.service  -> brings up the Docker container on boot
#   slam-fastlio.service    -> launches FAST-LIO + driver + MAVROS + bridge inside it
# slam-fastlio Requires slam-container, so enabling/starting fastlio pulls in the container.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

if [ "$(id -u)" -ne 0 ]; then
    echo "Run with sudo: sudo $0" >&2
    exit 1
fi

# 1. Ensure docker compose v2 is visible to root (systemd runs as root, which doesn't
#    see a user-local ~/.docker/cli-plugins). Install it system-wide if missing.
if ! docker compose version >/dev/null 2>&1; then
    PLUGIN_DIR=/usr/local/lib/docker/cli-plugins
    USER_HOME="$(getent passwd "${SUDO_USER:-revolute}" | cut -d: -f6)"
    SRC="$USER_HOME/.docker/cli-plugins/docker-compose"
    if [ -x "$SRC" ]; then
        install -d "$PLUGIN_DIR"
        install -m 0755 "$SRC" "$PLUGIN_DIR/docker-compose"
        echo "[+] Installed docker compose plugin system-wide from $SRC"
    else
        echo "[!] 'docker compose' not available to root and no plugin at $SRC" >&2
        echo "    Install Docker Compose v2 system-wide first." >&2
        exit 1
    fi
fi

# 2. Install unit files
install -m 0644 "$HERE/slam-container.service" /etc/systemd/system/slam-container.service
install -m 0644 "$HERE/slam-fastlio.service"   /etc/systemd/system/slam-fastlio.service
echo "[+] Installed unit files to /etc/systemd/system/"

# 3. Reload + enable on boot
systemctl daemon-reload
systemctl enable slam-container.service slam-fastlio.service
echo "[+] Enabled slam-container.service and slam-fastlio.service on boot"

echo
echo "Start now:   sudo systemctl start slam-fastlio.service   # pulls in the container too"
echo "Status:      systemctl status slam-container slam-fastlio"
echo "Logs:        journalctl -u slam-fastlio -f"
echo "Disable:     sudo systemctl disable --now slam-fastlio slam-container"
