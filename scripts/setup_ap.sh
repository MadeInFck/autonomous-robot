#!/bin/bash
# Setup RobotPatroller WiFi Access Point via NetworkManager
# Tested on Debian Trixie with NetworkManager
#
# Usage: sudo bash scripts/setup_ap.sh "YourPassword" "HomeSSID"
# Example: sudo bash scripts/setup_ap.sh "MyPassword" "Freebox-771D9F"

set -e

SSID="RobotPatroller"
# No default passphrase: a published default would let anyone in range
# connect to the AP and reach the robot.
PASSPHRASE="${1:?Usage: sudo bash scripts/setup_ap.sh \"YourPassword\" [\"HomeSSID\"] — passphrase obligatoire (15+ caracteres)}"
HOME_SSID="${2}"

# WPA2-PSK handshakes can be captured from a distance and cracked offline at
# no cost: the passphrase length is the only real protection. 15+ random
# characters puts a dictionary/brute-force attack out of practical reach.
MIN_PSK_LEN=15
PSK_LEN=${#PASSPHRASE}
if [ "$PSK_LEN" -lt "$MIN_PSK_LEN" ]; then
    echo "ERREUR: passphrase de $PSK_LEN caracteres, minimum $MIN_PSK_LEN requis." >&2
    echo "Un handshake WPA2 capture se casse hors-ligne : la longueur est la seule defense." >&2
    echo "Generer une passphrase solide :  openssl rand -base64 18" >&2
    echo "(pour forcer malgre tout : ALLOW_SHORT_PSK=1 sudo -E bash scripts/setup_ap.sh ...)" >&2
    [ "${ALLOW_SHORT_PSK}" = "1" ] || exit 1
    echo "ALLOW_SHORT_PSK=1 : passphrase courte acceptee malgre l'avertissement." >&2
fi
if [ "$PSK_LEN" -gt 63 ]; then
    echo "ERREUR: passphrase de $PSK_LEN caracteres, maximum 63 (limite WPA2-PSK)." >&2
    exit 1
fi
AP_IP="192.168.4.1"
CON_NAME="RobotAP"
COUNTRY="FR"

echo "=== Robot AP Setup (NetworkManager) ==="
echo "SSID     : $SSID"
echo "IP       : $AP_IP"
echo "Country  : $COUNTRY"
echo ""

# Remove existing RobotAP connection if present
nmcli con delete "$CON_NAME" 2>/dev/null || true

# Create hotspot connection
nmcli con add type wifi ifname wlan0 con-name "$CON_NAME" autoconnect yes ssid "$SSID"
nmcli con modify "$CON_NAME" 802-11-wireless.mode ap
nmcli con modify "$CON_NAME" 802-11-wireless.band bg
nmcli con modify "$CON_NAME" 802-11-wireless.channel 6
nmcli con modify "$CON_NAME" 802-11-wireless-security.key-mgmt wpa-psk
nmcli con modify "$CON_NAME" 802-11-wireless-security.psk "$PASSPHRASE"
nmcli con modify "$CON_NAME" 802-11-wireless-security.pmf 2
nmcli con modify "$CON_NAME" ipv4.method shared
nmcli con modify "$CON_NAME" ipv4.addresses "${AP_IP}/24"
nmcli con modify "$CON_NAME" connection.autoconnect yes
nmcli con modify "$CON_NAME" connection.autoconnect-priority 100

# Disable autoconnect on home WiFi so AP takes priority at boot
if [ -n "$HOME_SSID" ]; then
    nmcli con modify "$HOME_SSID" connection.autoconnect no
    echo "Home WiFi '$HOME_SSID' autoconnect disabled (use switch-to-client to reconnect)"
fi

# Install switch scripts
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cp "$SCRIPT_DIR/switch-to-ap.sh" /usr/local/bin/switch-to-ap
cp "$SCRIPT_DIR/switch-to-client.sh" /usr/local/bin/switch-to-client
chmod +x /usr/local/bin/switch-to-ap /usr/local/bin/switch-to-client

echo ""
echo "=== Setup complete ==="
echo "Run 'sudo switch-to-ap' to activate now, or reboot."
echo ""
echo "Robot will be reachable at:"
echo "  SSH  : ssh madeinfck@${AP_IP}"
echo "  Web  : https://${AP_IP}:8085"
