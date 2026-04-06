# Docker Network Configuration Guide for Ouster Sensor

Complete guide for configuring network connectivity between Docker containers and the Ouster OS2-64 LiDAR sensor.

## Network Architecture

```
┌─────────────────────────────────────────┐
│         Host Machine (Jetson)           │
│  IP: 169.254.56.1/16 on eth0           │
│                                         │
│  ┌───────────────────────────────────┐ │
│  │   Docker Container                 │ │
│  │   (network_mode: host)             │ │
│  │                                    │ │
│  │   Uses host network stack          │ │
│  │   Direct access to eth0            │ │
│  └───────────────────────────────────┘ │
│                                         │
│              eth0 (physical)            │
└──────────────┬──────────────────────────┘
               │
               │ Ethernet Cable
               │
┌──────────────┴──────────────────────────┐
│       Ouster OS2-64 LiDAR Sensor        │
│       IP: 169.254.56.220/16             │
│       Ports: 7502 (LiDAR), 7503 (IMU)   │
└─────────────────────────────────────────┘
```

## Why Host Networking?

The Docker setup uses `network_mode: host` for several reasons:

1. **UDP Multicast**: Ouster uses UDP multicast for discovery
2. **Low Latency**: Direct network access without NAT overhead
3. **Port Forwarding**: No need to manually forward ports 7502/7503
4. **Simplicity**: Container sees same network as host

### Trade-offs

**Pros:**
- Direct access to Ouster sensor
- No network address translation
- Minimal latency
- Simple configuration

**Cons:**
- Less isolation from host
- Port conflicts possible if services run on host
- Container can access all host network interfaces

## Host Network Configuration

### Step 1: Configure Jetson Network Interface

```bash
# Check current eth0 configuration
ip addr show eth0

# Add Ouster network to eth0
sudo ip addr add 169.254.56.1/16 dev eth0

# Bring interface up
sudo ip link set eth0 up

# Verify configuration
ip addr show eth0 | grep 169.254
# Should show: inet 169.254.56.1/16 scope link eth0
```

### Step 2: Make Configuration Persistent

Create `/etc/network/interfaces.d/eth0-ouster`:

```
# Ouster sensor network
auto eth0:0
iface eth0:0 inet static
    address 169.254.56.1
    netmask 255.255.0.0
```

Or using netplan (Ubuntu 20.04+), edit `/etc/netplan/01-netcfg.yaml`:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 169.254.56.1/16
```

Apply netplan config:
```bash
sudo netplan apply
```

### Step 3: Test Host Connectivity

```bash
# Ping Ouster sensor
ping -c 5 169.254.56.220

# Should see responses:
# 64 bytes from 169.254.56.220: icmp_seq=1 ttl=64 time=0.3 ms

# Check route
ip route get 169.254.56.220
# Should show: via eth0 src 169.254.56.1

# Test UDP ports (if nc installed)
nc -u -v 169.254.56.220 7502
```

## Docker Network Configuration

### Using Host Network (Default)

The `docker-compose.yml` uses:

```yaml
services:
  slam-system:
    network_mode: host
    environment:
      - OUSTER_IP=169.254.56.220
```

This means:
- Container uses host's network stack
- Container sees host's IP addresses
- Container can bind to any host port
- No port mapping needed

### Verification from Container

```bash
# Start container
docker compose run --rm slam-system bash

# Check network interfaces
ip addr show

# Should see same interfaces as host, including eth0 with 169.254.56.1

# Test Ouster connectivity
ping -c 3 169.254.56.220

# Test DNS resolution (if needed)
nslookup google.com
```

## Alternative: Bridge Networking

If you need network isolation, use bridge networking (not recommended for Ouster):

```yaml
services:
  slam-system:
    networks:
      - slam-net
    ports:
      - "7502:7502/udp"
      - "7503:7503/udp"
      - "11311:11311"  # ROS master

networks:
  slam-net:
    driver: bridge
```

**Limitations:**
- UDP multicast may not work
- Need to configure port forwarding
- Ouster sensor must be reconfigured to send to Docker bridge IP

## Firewall Configuration

### Check Firewall Status

```bash
# Ubuntu UFW
sudo ufw status

# iptables
sudo iptables -L -n
```

### Allow Ouster Traffic

If firewall is enabled:

```bash
# Allow UDP ports for Ouster
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp

# Or allow all from Ouster IP
sudo ufw allow from 169.254.56.220

# Reload firewall
sudo ufw reload
```

### Docker and Firewall

Docker automatically manages iptables rules. If using host networking, ensure:

```bash
# Allow forwarding
sudo iptables -P FORWARD ACCEPT

# Check Docker rules
sudo iptables -t nat -L -n | grep DOCKER
```

## Ouster Sensor Configuration

### Access Ouster Web Interface

```bash
# From host or container (if host networking)
curl http://169.254.56.220/api/v1/sensor/metadata

# Or use browser on same network:
# http://169.254.56.220
```

### Configure Ouster Network Settings

Default settings:
- **Sensor IP**: 169.254.56.220 (link-local, auto-configured)
- **Subnet**: 255.255.0.0 (/16)
- **UDP Destination**: Automatic (returns data to requester)

To change (if needed):

```bash
# Set UDP destination explicitly to host IP
curl -X POST http://169.254.56.220/api/v1/sensor/config \
  -H "Content-Type: application/json" \
  -d '{
    "udp_dest": "169.254.56.1",
    "udp_port_lidar": 7502,
    "udp_port_imu": 7503
  }'

# Reinitialize sensor
curl -X POST http://169.254.56.220/api/v1/sensor/cmd/reinitialize
```

### Verify Ouster Configuration

```bash
# Get current configuration
curl http://169.254.56.220/api/v1/sensor/config | jq .

# Check key settings
curl http://169.254.56.220/api/v1/sensor/config | jq '{
  udp_dest: .udp_dest,
  udp_port_lidar: .udp_port_lidar,
  udp_port_imu: .udp_port_imu,
  lidar_mode: .lidar_mode
}'
```

## ROS Network Configuration

### Environment Variables

The Docker container sets:

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

For multi-machine ROS:

```yaml
# docker-compose.yml
environment:
  - ROS_HOSTNAME=jetson.local
  - ROS_MASTER_URI=http://jetson.local:11311
  - ROS_IP=169.254.56.1
```

### Testing ROS Network

```bash
# Inside container 1: Start ROS master
roscore

# Inside container 2: Test connectivity
rostopic list

# Publish test message
rostopic pub /test std_msgs/String "data: 'hello'" -r 1

# Subscribe in another container
rostopic echo /test
```

## Troubleshooting Network Issues

### Issue: Cannot ping Ouster from host

```bash
# Check physical link
ip link show eth0
# Should show: state UP

# Check IP configuration
ip addr show eth0 | grep 169.254
# Should show: inet 169.254.56.1/16

# Check routing
ip route show | grep 169.254
# Should show route via eth0

# Try manual ping
ping -I eth0 169.254.56.220
```

**Solution:**
```bash
# Reset network interface
sudo ip link set eth0 down
sudo ip link set eth0 up
sudo ip addr add 169.254.56.1/16 dev eth0

# Power cycle Ouster sensor
```

### Issue: Cannot ping Ouster from container

```bash
# Check container network mode
docker inspect slam-ouster-fastlio | grep NetworkMode
# Should show: "NetworkMode": "host"

# Test from container
docker exec -it slam-ouster-fastlio ping -c 3 169.254.56.220
```

**Solution:**
```bash
# Restart container with host networking
docker compose down
docker compose up -d

# Verify network mode in docker-compose.yml
cat docker-compose.yml | grep network_mode
```

### Issue: Ouster topics not appearing in ROS

```bash
# Check if Ouster driver is running
rosnode list | grep ouster

# Check topics
rostopic list | grep ouster

# Test ROS network
rostopic hz /rosout

# Check Ouster driver logs
rosnode info /ouster/os_node
```

**Solution:**
```bash
# Restart Ouster driver with explicit IP
roslaunch ouster_ros sensor.launch \
  sensor_hostname:=169.254.56.220 \
  udp_dest:=169.254.56.1 \
  viz:=false

# Check for UDP packets
sudo tcpdump -i eth0 -n udp port 7502 or udp port 7503
```

### Issue: Intermittent data loss

**Symptoms:**
```bash
rostopic hz /ouster/points
# Shows 0.0 Hz or inconsistent rate
```

**Diagnosis:**
```bash
# Check packet loss
sudo tcpdump -i eth0 -n udp port 7502 -c 100 | wc -l

# Check system load
top
docker stats

# Check network errors
ip -s link show eth0
```

**Solution:**
```bash
# Increase socket buffer size
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# Make persistent in /etc/sysctl.conf
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_default=26214400" | sudo tee -a /etc/sysctl.conf

# Restart container
docker compose restart slam-system
```

### Issue: Port conflicts

**Symptoms:**
```bash
Error: bind: address already in use (port 11311)
```

**Solution:**
```bash
# Check what's using the port
sudo netstat -tulpn | grep 11311

# Kill existing ROS master
killall rosmaster roscore

# Or use different port
export ROS_MASTER_URI=http://localhost:11312
```

## Network Performance Monitoring

### Monitor UDP Traffic

```bash
# Monitor Ouster packets in real-time
sudo tcpdump -i eth0 -n udp port 7502 or udp port 7503

# Count packets per second
sudo tcpdump -i eth0 -n udp port 7502 2>&1 | \
  pv -l -i 1 > /dev/null
```

### Measure Bandwidth

```bash
# Install iftop
sudo apt-get install iftop

# Monitor eth0 bandwidth
sudo iftop -i eth0 -f "udp port 7502 or udp port 7503"
```

### Check Latency

```bash
# Ping statistics
ping -c 100 169.254.56.220 | tail -1

# Expected: avg < 1ms for link-local network

# ROS topic latency
rostopic delay /ouster/points
```

## Security Considerations

### Host Network Security

With `network_mode: host`, the container can:
- Bind to any host port
- Access any host network service
- See host network traffic

**Mitigation:**
1. Run container with limited privileges where possible
2. Use firewall rules to restrict container access
3. Monitor container network activity
4. Consider user namespaces for additional isolation

### Network Isolation

For production, consider:
```bash
# Run with restricted network access
docker compose run --rm \
  --cap-drop=ALL \
  --cap-add=NET_BIND_SERVICE \
  slam-system
```

## Advanced Configuration

### Multiple Ouster Sensors

To support multiple sensors:

```yaml
# docker-compose.yml
services:
  ouster-1:
    extends: slam-system
    environment:
      - OUSTER_IP=169.254.56.220
    command: roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.56.220

  ouster-2:
    extends: slam-system
    environment:
      - OUSTER_IP=169.254.56.221
    command: roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.56.221
```

### VLAN Configuration

For VLAN networks:

```bash
# Create VLAN interface on host
sudo ip link add link eth0 name eth0.100 type vlan id 100
sudo ip addr add 169.254.56.1/16 dev eth0.100
sudo ip link set eth0.100 up
```

### Static Routes

Add static routes if needed:

```bash
# Add route to Ouster subnet
sudo ip route add 169.254.56.0/24 dev eth0

# Make persistent in /etc/network/interfaces
up ip route add 169.254.56.0/24 dev eth0
```

## Network Testing Tools

Useful tools for diagnostics:

```bash
# Install tools in container
apt-get update && apt-get install -y \
  tcpdump \
  net-tools \
  iputils-ping \
  iproute2 \
  netcat \
  nmap \
  iftop \
  iperf3

# Scan Ouster ports
nmap -sU -p 7502,7503 169.254.56.220

# Test UDP connectivity
nc -u -v 169.254.56.220 7502
```

## Summary

**Key Points:**
1. Use host networking for simplest Ouster integration
2. Configure host eth0 with 169.254.56.1/16
3. Verify Ouster sensor reachable at 169.254.56.220
4. Test from container before running SLAM
5. Monitor network performance for production use

**Quick Test Checklist:**
```bash
# 1. Host connectivity
ping 169.254.56.220

# 2. Container connectivity
docker exec slam-ouster-fastlio ping 169.254.56.220

# 3. ROS topics appearing
rostopic list | grep ouster

# 4. Data flowing
rostopic hz /ouster/points

# 5. No packet loss
sudo tcpdump -i eth0 udp port 7502 -c 100
```

For issues, start with the troubleshooting section and work through each network layer systematically.
