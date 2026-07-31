#!/bin/bash
# Installation script for Raspberry Pi 5
# Autonomous Mecanum Robot
# Deployment in ~/AutonomRobot

set -e

# Installation directory
INSTALL_DIR="$HOME/AutonomRobot"

echo "=================================="
echo "  Setup Robot Mecanum - Pi 5"
echo "  Dossier: $INSTALL_DIR"
echo "=================================="

# Check that we are on a Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "ATTENTION: Ce script est conçu pour Raspberry Pi"
fi

# 1. Enable UART3
echo ""
echo "[1/6] Configuration UART3..."
CONFIG_FILE="/boot/firmware/config.txt"
if [ -f "$CONFIG_FILE" ]; then
    if ! grep -q "dtoverlay=uart3-pi5" "$CONFIG_FILE"; then
        echo "dtoverlay=uart3-pi5" | sudo tee -a "$CONFIG_FILE"
        echo "  UART3 ajouté à config.txt"
        NEED_REBOOT=true
    else
        echo "  UART3 déjà configuré"
    fi
else
    echo "  ATTENTION: $CONFIG_FILE non trouvé"
fi

# 2. Install system dependencies
echo ""
echo "[2/6] Installation des dépendances système..."
sudo apt-get update
sudo apt-get install -y \
    python3-venv \
    python3-pip \
    python3-lgpio \
    libgpiod-dev

# 3. Create the virtual environment
echo ""
echo "[3/6] Création de l'environnement virtuel..."

# Navigate to the pi5 directory
PI5_DIR="$INSTALL_DIR/pi5"
if [ ! -d "$PI5_DIR" ]; then
    echo "  ERREUR: Dossier $PI5_DIR non trouvé"
    echo "  Copiez d'abord le projet dans $INSTALL_DIR"
    exit 1
fi

cd "$PI5_DIR"

if [ ! -d "venv" ]; then
    python3 -m venv venv --system-site-packages
    echo "  Environnement virtuel créé"
else
    echo "  Environnement virtuel existant"
fi

# 4. Install Python dependencies
echo ""
echo "[4/6] Installation des dépendances Python..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# 5. Serial permissions
echo ""
echo "[5/6] Configuration des permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER

# 6. Sudoers rule for the web UI shutdown/restart buttons.
# Restricted to exactly these two commands — do NOT grant a broad NOPASSWD,
# the web server would otherwise become a full root escalation.
echo ""
echo "[6/6] Regle sudoers (boutons Eteindre/Redemarrer de l'interface web)..."
SUDOERS_FILE="/etc/sudoers.d/robot-power"
sudo tee "$SUDOERS_FILE" > /dev/null <<EOF
# Robot Mecanum: web UI power buttons (pi5/telemetry/web_server.py)
$USER ALL=(root) NOPASSWD: /usr/sbin/shutdown now, /usr/sbin/reboot ""
EOF
sudo chmod 0440 "$SUDOERS_FILE"
sudo visudo -c -f "$SUDOERS_FILE" > /dev/null && echo "  Regle installee: $SUDOERS_FILE"

echo ""
echo "=================================="
echo "  Installation terminée!"
echo "=================================="
echo ""
echo "Prochaines étapes:"
echo "  1. Redémarrer si UART3 a été configuré"
echo "  2. Câbler le Pico et les moteurs"
echo "  3. Lancer:"
echo "     cd $PI5_DIR"
echo "     source venv/bin/activate"
echo "     python main.py"
echo ""

if [ "$NEED_REBOOT" = true ]; then
    echo "ATTENTION: Un redémarrage est nécessaire pour activer UART3"
    read -p "Redémarrer maintenant? (o/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Oo]$ ]]; then
        sudo reboot
    fi
fi
