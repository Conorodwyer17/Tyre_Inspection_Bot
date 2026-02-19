#!/bin/bash
# Configure Jetson with static IP on Aurora network
# WARNING: This will change network configuration - use with caution

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

JETSON_IP="${1:-192.168.11.10}"
AURORA_IP="192.168.11.1"

echo "=========================================="
echo "Configure Jetson Static IP"
echo "=========================================="
echo ""
echo "This will configure Jetson Ethernet with:"
echo "  IP Address: $JETSON_IP"
echo "  Gateway:    $AURORA_IP"
echo "  Network:    192.168.11.0/24"
echo ""
read -p "Continue? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 1
fi

# Find Ethernet interface
ETH_INTERFACE=$(ip link show | grep -E "^[0-9]+:.*eth|^[0-9]+:.*enp" | grep -v "lo" | head -1 | cut -d: -f2 | xargs)
if [ -z "$ETH_INTERFACE" ]; then
    echo -e "${RED}✗${NC} No Ethernet interface found"
    echo "Available interfaces:"
    ip link show | grep "^[0-9]+:" | cut -d: -f2
    exit 1
fi

echo -e "${GREEN}✓${NC} Found Ethernet interface: $ETH_INTERFACE"
echo ""

# Backup current config
NETPLAN_FILE="/etc/netplan/01-netcfg.yaml"
if [ -f "$NETPLAN_FILE" ]; then
    echo "Backing up current config..."
    sudo cp "$NETPLAN_FILE" "${NETPLAN_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    echo -e "${GREEN}✓${NC} Backup created"
else
    echo -e "${YELLOW}⚠${NC} No existing netplan config found"
fi
echo ""

# Create new config
echo "Creating new network configuration..."
sudo tee "$NETPLAN_FILE" > /dev/null << EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    $ETH_INTERFACE:
      dhcp4: no
      addresses:
        - $JETSON_IP/24
      gateway4: $AURORA_IP
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
EOF

echo -e "${GREEN}✓${NC} Configuration written"
echo ""

# Apply configuration
echo "Applying network configuration..."
echo "This may temporarily disconnect network..."
sudo netplan apply

echo ""
echo "Waiting for network to stabilize..."
sleep 3

# Verify
echo ""
echo "=== Verification ==="
if ping -c 2 -W 2 "$AURORA_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Aurora reachable at $AURORA_IP"
else
    echo -e "${RED}✗${NC} Aurora NOT reachable - check configuration"
fi

CURRENT_IP=$(ip addr show "$ETH_INTERFACE" | grep "inet " | awk '{print $2}' | cut -d/ -f1)
if [ "$CURRENT_IP" = "$JETSON_IP" ]; then
    echo -e "${GREEN}✓${NC} Jetson IP configured: $CURRENT_IP"
else
    echo -e "${YELLOW}⚠${NC} IP mismatch: expected $JETSON_IP, got $CURRENT_IP"
fi

echo ""
echo "=========================================="
echo "Configuration complete!"
echo ""
echo "Jetson IP: $JETSON_IP"
echo "SSH from laptop: ssh conor@$JETSON_IP"
echo ""
echo "If network is not working, restore backup:"
echo "  sudo cp ${NETPLAN_FILE}.backup.* $NETPLAN_FILE"
echo "  sudo netplan apply"
echo ""
