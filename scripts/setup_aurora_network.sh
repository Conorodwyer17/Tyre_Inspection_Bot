#!/bin/bash
# Quick setup script for Aurora network configuration
# Sets Jetson static IP on Aurora network for reliable SSH access

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

JETSON_IP="192.168.11.10"
AURORA_IP="192.168.11.1"
ETH_INTERFACE="enP8p1s0"

echo "=========================================="
echo "Aurora Network Setup"
echo "=========================================="
echo ""
echo "Current status:"
echo "  Ethernet interface: $ETH_INTERFACE"
CURRENT_IP=$(ip addr show "$ETH_INTERFACE" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 || echo "none")
echo "  Current IP: $CURRENT_IP"
echo "  Aurora IP: $AURORA_IP"
echo ""
echo "This will set Jetson static IP to: $JETSON_IP"
echo ""

# Check if already configured
if [ "$CURRENT_IP" = "$JETSON_IP" ]; then
    echo -e "${GREEN}✓${NC} Jetson already configured with static IP: $JETSON_IP"
    echo ""
    echo "=========================================="
    echo "SSH Information"
    echo "=========================================="
    echo ""
    echo "When your laptop is connected to Aurora Wi-Fi:"
    echo "  ${BLUE}ssh conor@$JETSON_IP${NC}"
    echo ""
    echo "Aurora Wi-Fi network:"
    echo "  SSID: SLAMWARE-Aurora-xxxxxx (check Aurora label)"
    echo "  Default IP: $AURORA_IP"
    echo ""
    exit 0
fi

read -p "Set static IP to $JETSON_IP? (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled. Using current IP: $CURRENT_IP"
    echo ""
    echo "To SSH from laptop (when connected to Aurora Wi-Fi):"
    echo "  ${BLUE}ssh conor@$CURRENT_IP${NC}"
    exit 0
fi

# Backup netplan config
NETPLAN_FILE="/etc/netplan/01-netcfg.yaml"
if [ -f "$NETPLAN_FILE" ]; then
    BACKUP_FILE="${NETPLAN_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    echo "Backing up current config to: $BACKUP_FILE"
    sudo cp "$NETPLAN_FILE" "$BACKUP_FILE"
fi

# Check if netplan file exists and read it
if [ -f "$NETPLAN_FILE" ]; then
    # Try to preserve Wi-Fi config if it exists
    echo "Reading existing network configuration..."
    # We'll create a new config that preserves Wi-Fi but sets static IP for Ethernet
else
    echo "Creating new network configuration..."
fi

# Create/update netplan config
echo ""
echo "Configuring static IP for Ethernet interface..."
sudo tee "$NETPLAN_FILE" > /dev/null << 'EOF'
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enP8p1s0:
      dhcp4: no
      addresses:
        - 192.168.11.10/24
      gateway4: 192.168.11.1
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
sleep 5

# Verify
echo ""
echo "=== Verification ==="
if ping -c 2 -W 2 "$AURORA_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Aurora reachable at $AURORA_IP"
else
    echo -e "${RED}✗${NC} Aurora NOT reachable - check Ethernet cable"
fi

NEW_IP=$(ip addr show "$ETH_INTERFACE" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 || echo "none")
if [ "$NEW_IP" = "$JETSON_IP" ]; then
    echo -e "${GREEN}✓${NC} Jetson IP configured: $NEW_IP"
else
    echo -e "${YELLOW}⚠${NC} IP is $NEW_IP (expected $JETSON_IP)"
    echo "  This may be normal if NetworkManager is managing the interface"
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Jetson IP on Aurora network: ${BLUE}$JETSON_IP${NC}"
echo ""
echo "Next steps:"
echo "  1. Connect your laptop to Aurora Wi-Fi network"
echo "     SSID: SLAMWARE-Aurora-xxxxxx (check Aurora label)"
echo ""
echo "  2. SSH to Jetson:"
echo "     ${BLUE}ssh conor@$JETSON_IP${NC}"
echo ""
echo "  3. Verify Aurora connection:"
echo "     ping 192.168.11.1"
echo ""
echo "If network issues occur, restore backup:"
if [ -n "$BACKUP_FILE" ]; then
    echo "     sudo cp $BACKUP_FILE $NETPLAN_FILE"
    echo "     sudo netplan apply"
fi
echo ""
