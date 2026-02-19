#!/bin/bash
# Network connectivity check for field testing
# Checks Aurora, Jetson network configuration, and Wi-Fi status

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Network Connectivity Check"
echo "=========================================="
echo ""

# Check Aurora connectivity
echo "=== Aurora (Ethernet) ==="
if ping -c 2 -W 2 192.168.11.1 > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Aurora reachable at 192.168.11.1"
    AURORA_OK=true
else
    echo -e "${RED}✗${NC} Aurora NOT reachable at 192.168.11.1"
    echo "  Check: Ethernet cable connected, Aurora powered on"
    AURORA_OK=false
fi
echo ""

# Check Jetson network interfaces
echo "=== Jetson Network Interfaces ==="
echo "Ethernet interfaces:"
ip addr show | grep -E "^[0-9]+:.*eth|^[0-9]+:.*enp" | grep -v "lo" || echo "  No Ethernet interfaces found"

echo ""
echo "IP addresses (excluding loopback):"
ip addr show | grep "inet " | grep -v "127.0.0.1" | while read line; do
    echo "  $line"
done

echo ""
echo "Default route:"
ip route | grep default || echo "  No default route configured"

echo ""

# Check Wi-Fi status
echo "=== Wi-Fi Status ==="
if command -v nmcli > /dev/null 2>&1; then
    echo "Available Wi-Fi networks:"
    nmcli device wifi list | head -10 || echo "  Could not scan Wi-Fi"
    echo ""
    echo "Active Wi-Fi connections:"
    nmcli connection show --active | grep wifi || echo "  No active Wi-Fi connections"
else
    echo "  nmcli not available - install NetworkManager"
fi
echo ""

# Check SSH service
echo "=== SSH Service ==="
if systemctl is-active --quiet ssh || systemctl is-active --quiet sshd; then
    echo -e "${GREEN}✓${NC} SSH service is running"
    SSH_OK=true
else
    echo -e "${YELLOW}⚠${NC} SSH service may not be running"
    SSH_OK=false
fi

# Get SSH listening port
SSH_PORT=$(ss -tlnp | grep sshd | head -1 | awk '{print $4}' | cut -d: -f2 || echo "unknown")
echo "  SSH listening on port: $SSH_PORT"
echo ""

# Check ROS 2 topics (if Aurora is connected)
if [ "$AURORA_OK" = true ]; then
    echo "=== Aurora ROS 2 Topics ==="
    if command -v ros2 > /dev/null 2>&1; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        if ros2 topic list 2>/dev/null | grep -q slamware; then
            echo -e "${GREEN}✓${NC} Aurora ROS topics available:"
            ros2 topic list | grep slamware | head -5
        else
            echo -e "${YELLOW}⚠${NC} Aurora ROS topics not found"
            echo "  Aurora may not be running ROS SDK"
        fi
    else
        echo "  ROS 2 not sourced"
    fi
else
    echo "  Skipping ROS check (Aurora not reachable)"
fi
echo ""

# Summary and recommendations
echo "=========================================="
echo "Summary & Recommendations"
echo "=========================================="
echo ""

if [ "$AURORA_OK" = true ] && [ "$SSH_OK" = true ]; then
    echo -e "${GREEN}✓${NC} Network setup looks good!"
    echo ""
    echo "To connect from laptop:"
    JETSON_IP=$(ip addr show | grep "inet " | grep -v "127.0.0.1" | head -1 | awk '{print $2}' | cut -d/ -f1)
    if [ -n "$JETSON_IP" ]; then
        echo "  ssh conor@$JETSON_IP"
    else
        echo "  ssh conor@<jetson-ip>"
    fi
elif [ "$AURORA_OK" = false ]; then
    echo -e "${RED}✗${NC} Aurora not reachable"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check Ethernet cable: Jetson ↔ Aurora"
    echo "  2. Verify Aurora is powered on (green light)"
    echo "  3. Check Aurora IP: ping 192.168.11.1"
    echo "  4. Try: ip link show (check Ethernet interface)"
elif [ "$SSH_OK" = false ]; then
    echo -e "${YELLOW}⚠${NC} SSH service issue"
    echo ""
    echo "To enable SSH:"
    echo "  sudo systemctl enable ssh"
    echo "  sudo systemctl start ssh"
fi

echo ""
echo "For field testing network setup, see:"
echo "  ~/ugv_ws/src/Tyre_Inspection_Bot/docs/NETWORK_SETUP_FIELD_TESTING.md"
echo ""
