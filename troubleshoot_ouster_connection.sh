#!/bin/bash
# Ouster Connectivity Troubleshooting Script
# Run this to diagnose eth0 and Ouster connection issues

echo "========================================"
echo "OUSTER CONNECTION DIAGNOSTIC"
echo "========================================"
echo ""

echo "[1] CHECKING ETH0 PHYSICAL STATE..."
ip link show eth0
echo ""

echo "[2] CHECKING ETH0 IP CONFIGURATION..."
ip addr show eth0
echo ""

echo "[3] CHECKING NETWORK ROUTES..."
ip route show | grep eth0
echo ""

echo "[4] CHECKING FOR RUNNING DHCP CLIENT ON ETH0..."
ps aux | grep -i "dhclient.*eth0\|systemd-networkd"
echo ""

echo "[5] ATTEMPTING TO PING KNOWN OUSTER IP (169.254.56.220)..."
timeout 3 ping -c 1 169.254.56.220 2>&1 || echo "(Expected to fail if not connected)"
echo ""

echo "[6] CHECKING OUSTER HOSTNAME RESOLUTION..."
nslookup os-122224003549.local 2>&1 || echo "(May fail if not on network)"
echo ""

echo "[7] CHECKING ETH0 INTERFACE CONFIG..."
cat /etc/netplan/*.yaml 2>/dev/null | head -30
echo ""

echo "[8] CHECKING SYSTEM LOGS FOR ETH0 ERRORS..."
dmesg | tail -20 | grep -i "eth0\|enp\|network\|link" || echo "(No recent network errors)"
echo ""

echo "========================================"
echo "DIAGNOSTIC SUMMARY"
echo "========================================"
echo ""
echo "eth0 Status:"
if ip link show eth0 | grep -q "NO-CARRIER"; then
    echo "  ❌ NO-CARRIER: Physical link is DOWN"
    echo "  ACTION: Check that:"
    echo "    1. Ethernet cable is plugged into Jetson eth0 port"
    echo "    2. Other end is plugged into Ouster sensor"
    echo "    3. Both devices are powered on"
    echo "    4. Cable is not damaged"
elif ip link show eth0 | grep -q "state UP"; then
    echo "  ✅ CARRIER PRESENT: Physical link is UP"
    echo "  Checking IP assignment..."
    if ip addr show eth0 | grep -q "inet "; then
        echo "  ✅ IP Address Assigned"
    else
        echo "  ⚠️  No IP assigned. Waiting for DHCP or link-local address..."
    fi
else
    echo "  ⚠️  eth0 is DOWN but has power (UP flag present)"
fi

echo ""
echo "Expected Configuration:"
echo "  Interface: eth0"
echo "  IP Type: Link-local (169.254.x.x) or DHCP"
echo "  Expected IP: 169.254.56.220 (Ouster sensor)"
echo "  Connection: Direct Ethernet to Ouster OS1-64"
echo ""
echo "========================================"
