#!/bin/bash
# Switch Pi 5 to WiFi client mode (home network) for rsync/updates
# Usage: sudo switch-to-client
# After rsync, reboot to return to AP mode: sudo reboot

set -e

CON_NAME="RobotAP"

echo "=== Switching to WiFi client mode ==="

# Stop the AP
nmcli con down "$CON_NAME" 2>/dev/null || true

# Reconnect to home WiFi (first known network found)
HOME_CON=$(nmcli -t -f NAME,TYPE con show | grep wireless | grep -v "$CON_NAME" | cut -d: -f1 | head -1)
if [ -z "$HOME_CON" ]; then
    echo "No home WiFi profile found. Connect manually with:"
    echo "  nmcli dev wifi connect \"YourSSID\" password \"YourPassword\""
    exit 1
fi

echo "Connecting to: $HOME_CON"
nmcli con up "$HOME_CON"

echo "Waiting for DHCP..."
sleep 5

IP=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+\.\d+\.\d+\.\d+' || echo "unknown")
echo ""
echo "Client mode active — IP: ${IP}"
echo "You can now rsync from your Mac."
echo ""
echo "When done: sudo reboot  (returns to AP mode on next boot)"
