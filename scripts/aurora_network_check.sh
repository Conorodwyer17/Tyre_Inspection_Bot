#!/bin/bash
# Priority A: Verify Jetson network and Aurora reachability.
# Run: ./scripts/aurora_network_check.sh [eth-iface]
# Saves output to logs/aurora_integration/<ts>/

set -e
UGV_WS="${UGV_WS:-$HOME/ugv_ws}"
TS=$(date +%Y%m%dT%H%M%S)
LOG_DIR="$UGV_WS/logs/aurora_integration/$TS"
mkdir -p "$LOG_DIR"

ETH="${1:-eth0}"
# Try to find an ethernet interface if eth0 missing
if ! ip link show "$ETH" &>/dev/null; then
  ETH=$(ip -o link show | awk -F': ' '{print $2}' | grep -E '^e(nx|th)' | head -1 || true)
  [ -z "$ETH" ] && ETH="eth0"
fi

echo "=== Aurora network check ==="
echo "Interface: $ETH"
echo "Log dir: $LOG_DIR"

ip addr show > "$LOG_DIR/ip_addr.txt" 2>&1
ip route show >> "$LOG_DIR/ip_addr.txt" 2>&1
ethtool "$ETH" >> "$LOG_DIR/ip_addr.txt" 2>&1 || true
arp -n >> "$LOG_DIR/ip_addr.txt" 2>&1 || true

echo "Pinging 192.168.11.1 ..."
if ping -c 5 192.168.11.1 >> "$LOG_DIR/ip_addr.txt" 2>&1; then
  echo "PING OK"
  exit 0
else
  echo "PING FAILED - check $LOG_DIR/ip_addr.txt"
  exit 1
fi
