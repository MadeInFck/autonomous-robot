#!/bin/bash
# Switch Pi 5 to Access Point mode (RobotPatroller) via NetworkManager
# Usage: sudo switch-to-ap

set -e

CON_NAME="RobotAP"
AP_IP="192.168.4.1"

echo "=== Switching to AP mode ==="

# Disconnect from any active WiFi client connection
ACTIVE_CLIENT=$(nmcli -t -f NAME,TYPE con show --active | grep wireless | grep -v "$CON_NAME" | cut -d: -f1 | head -1)
if [ -n "$ACTIVE_CLIENT" ]; then
    echo "Disconnecting from: $ACTIVE_CLIENT"
    nmcli con down "$ACTIVE_CLIENT" 2>/dev/null || true
fi

# Activate the AP
nmcli con up "$CON_NAME"

echo ""
echo "AP mode active — SSID: RobotPatroller"
echo "SSH  : ssh madeinfck@${AP_IP}"
echo "Web  : https://${AP_IP}:8085"
