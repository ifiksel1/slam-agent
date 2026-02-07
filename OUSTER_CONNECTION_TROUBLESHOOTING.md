# Ouster Connection Troubleshooting Guide

## Current Status
- **eth0 Interface**: NO-CARRIER (physical link DOWN)
- **eth0 IP**: 192.168.2.50/24 (configured but not connected)
- **Ouster Expected IP**: 169.254.56.220 (link-local)
- **Issue**: Physical Ethernet cable not connected or Ouster sensor not powered/reachable

---

## Quick Diagnostic Results
```
eth0 Status: <NO-CARRIER,BROADCAST,MULTICAST,UP> state DOWN
Routes: 192.168.2.0/24 proto kernel scope link src 192.168.2.50 linkdown
Ping 169.254.56.220: FAILED (Host Unreachable)
```

---

## Step-by-Step Troubleshooting

### Step 1: Physical Connection Check
**BEFORE running any software commands, verify hardware:**

1. **Check Jetson Ethernet Port:**
   - Locate the Ethernet port on the Jetson Orin NX (typically labeled eth0 or enP8p1s0)
   - Verify the port has power indicator LED (if present)
   - LED should be **AMBER/YELLOW** when connected

2. **Check Ethernet Cable:**
   - Visually inspect the cable for damage
   - Ensure it's fully plugged into the Jetson port (should click/seat firmly)
   - Try a different Ethernet cable if available to rule out cable failure

3. **Check Ouster Sensor:**
   - Verify Ouster is **POWERED ON** (look for status LEDs)
   - Verify the Ethernet cable is connected to the Ouster's Ethernet port
   - If Ouster has multiple ports, use the main Ethernet port (not redundant/backup port)

4. **Check Direct Connection:**
   - Verify it's a **direct connection** (Jetson eth0 → Ouster eth port)
   - Do NOT use a network switch between them (link-local needs direct connection)

---

### Step 2: Bring up eth0 Interface
Once physical connection is verified:

```bash
# Bring up eth0 if it's administratively down
sudo ip link set eth0 up

# Wait 2-3 seconds for link negotiation
sleep 3

# Check if carrier is now present
ip link show eth0

# Expected output should show CARRIER instead of NO-CARRIER
```

---

### Step 3: Verify Link-Local Address
```bash
# Check if link-local address is assigned
ip addr show eth0

# You should see:
# inet 169.254.x.x/16 scope link
# OR
# inet 192.168.2.50/24 scope global
```

If not present, manually assign link-local:
```bash
# Assign a link-local address in the same subnet as Ouster
sudo ip addr add 169.254.56.200/16 dev eth0
```

---

### Step 4: Test Connectivity
```bash
# Ping the Ouster sensor
ping -c 3 169.254.56.220

# Expected output:
# 64 bytes from 169.254.56.220: icmp_seq=1 ttl=64 time=0.8ms
```

---

### Step 5: Try Alternate Network Configuration
If step 4 fails, the Ouster might be on a different subnet. Try:

```bash
# Check if Ouster is on default subnet
sudo ip addr add 192.168.2.51/24 dev eth0
ping -c 3 192.168.2.1  # Try gateway

# Or try DHCP if Ouster has DHCP server
sudo dhclient eth0
sleep 2
ip addr show eth0
```

---

## Ouster Driver Launch (Once Connected)

Once Step 4 succeeds (ping returns replies):

```bash
# Terminal 1: Start ROS Master
roscore

# Terminal 2: Launch Ouster driver with sensor IP
roslaunch ouster_ros driver.launch sensor_hostname:=169.254.56.220

# Expected output:
# [INFO] [timestamp]: Loading nodelet /ouster/os_driver...
# [INFO] [timestamp]: Initializing nodelet with N worker threads
```

---

## Verification Commands

```bash
# 1. Check if eth0 is up
ip link show eth0 | grep -i "carrier\|state"

# 2. Check if IP is assigned
ip addr show eth0

# 3. Check if Ouster is reachable
ping -c 1 169.254.56.220

# 4. Check if Ouster driver is running
rosnode list | grep ouster

# 5. Check if LiDAR topics are publishing
rostopic list | grep ouster

# 6. Check LiDAR data
rostopic hz /os_cloud_node/points  # Should show ~10 Hz
```

---

## Common Issues & Solutions

### Issue 1: eth0 Still Shows NO-CARRIER After Plugging In
**Possible Causes:**
- Cable is loose or defective → try reseating or different cable
- Ouster is powered off → check power supply and LEDs
- Network interface is disabled → run `sudo ip link set eth0 up`
- Driver issue → try `sudo ethtool eth0` to check interface driver

**Solution:**
```bash
# Check driver status
ethtool eth0

# Restart network interface
sudo ip link set eth0 down
sleep 1
sudo ip link set eth0 up
sleep 2
ip link show eth0  # Check for CARRIER
```

### Issue 2: Ping Fails with "Network Unreachable"
**Possible Causes:**
- Ouster is on different subnet
- Link-local negotiation hasn't completed
- Firewall blocking

**Solution:**
```bash
# Manually set IP in correct subnet
sudo ip addr flush dev eth0
sudo ip addr add 169.254.56.1/16 dev eth0

# Retry ping
ping 169.254.56.220
```

### Issue 3: Ouster Driver Starts But No Data
**Possible Causes:**
- Ouster sensor not actually connected (verify Step 1)
- Sensor firmware issue
- Configuration mismatch

**Solution:**
```bash
# Check driver logs
cat ~/.ros/log/latest/ouster-*

# Verify sensor is accessible
ping -c 10 169.254.56.220  # Should see responses

# Check if driver can reach sensor
roslaunch ouster_ros driver.launch sensor_hostname:=169.254.56.220 &
sleep 2
rosnode list  # Should show /ouster/os_driver
```

---

## Hardware Profile Reference
Based on your known-good configuration:

```yaml
Network:
  Interface: eth0
  Connection Type: Direct Ethernet to Ouster
  IP Configuration: Link-local (169.254.56.220)

Ouster OS1-64:
  Hostname: os-122224003549.local
  Model: Ouster OS1-64 LiDAR
  Channels: 64 (vertical)
  Rate: 10 Hz
  Built-in IMU: InvenSense ICM-20948

ROS Topics (When Working):
  /os_cloud_node/points     → 64-channel point cloud @ 10 Hz
  /os_cloud_node/imu        → IMU data @ 100 Hz
  /os_cloud_node/metadata   → Sensor configuration
```

---

## Next Steps

1. **Physically verify connection** (see Step 1 above)
2. **Bring up eth0** (see Step 2)
3. **Ping Ouster** (see Step 4)
4. **Launch driver** (see driver section above)
5. **Verify topics** (use verification commands)
6. **Monitor SLAM** (watch /Odometry topic for x,y,z positions)

---

## Support

Run this diagnostic again to check status:
```bash
/home/dev/slam-agent/troubleshoot_ouster_connection.sh
```

If issues persist, check:
- `/home/dev/.ros/log/latest/ouster-*.log` for driver errors
- `/home/dev/.ros/log/latest/master.log` for ROS issues
- `dmesg` for kernel-level network errors

---

**Status**: ⏳ AWAITING PHYSICAL CONNECTION
**Last Updated**: 2026-02-07
**System Status**: SLAM ready, awaiting LiDAR data flow
