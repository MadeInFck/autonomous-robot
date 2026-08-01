#!/bin/bash
# Installation script for Raspberry Pi 5
# Autonomous Mecanum Robot
# Deployment in ~/AutonomRobot

set -e

# Run as the normal robot user: the script calls sudo where it needs to, and
# $USER/$HOME decide who owns the venv, the sudoers rule and the systemd unit.
# Running the whole thing as root would install all three for root.
if [ "$(id -u)" -eq 0 ]; then
    echo "ERREUR: lancez ce script en tant qu'utilisateur normal (il appelle sudo lui-meme):" >&2
    echo "  bash scripts/setup_pi5_env.sh" >&2
    exit 1
fi

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
echo "[1/7] Configuration UART3..."
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
echo "[2/7] Installation des dépendances système..."
sudo apt-get update
sudo apt-get install -y \
    python3-venv \
    python3-pip \
    python3-lgpio \
    libgpiod-dev

# 3. Create the virtual environment
echo ""
echo "[3/7] Création de l'environnement virtuel..."

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
echo "[4/7] Installation des dépendances Python..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# 5. Serial permissions
echo ""
echo "[5/7] Configuration des permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER

# 6. Sudoers rule for the web UI shutdown/restart buttons.
# Restricted to exactly these two commands — do NOT grant a broad NOPASSWD,
# the web server would otherwise become a full root escalation.
echo ""
echo "[6/7] Regle sudoers (boutons Eteindre/Redemarrer de l'interface web)..."
SUDOERS_FILE="/etc/sudoers.d/robot-power"
sudo tee "$SUDOERS_FILE" > /dev/null <<EOF
# Robot Mecanum: web UI power buttons (pi5/telemetry/web_server.py)
$USER ALL=(root) NOPASSWD: /usr/sbin/shutdown now, /usr/sbin/reboot ""
EOF
sudo chmod 0440 "$SUDOERS_FILE"
sudo visudo -c -f "$SUDOERS_FILE" > /dev/null && echo "  Regle installee: $SUDOERS_FILE"

# 7. systemd unit: start the robot at boot (motors idle, patrol not started).
echo ""
echo "[7/7] Service systemd (demarrage automatique au boot)..."
SERVICE_SRC="$INSTALL_DIR/scripts/robot.service"
SERVICE_DST="/etc/systemd/system/robot.service"
if [ -f "$SERVICE_SRC" ]; then
    sed -e "s|__USER__|$USER|g" -e "s|__INSTALL_DIR__|$INSTALL_DIR|g" \
        "$SERVICE_SRC" | sudo tee "$SERVICE_DST" > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable robot.service > /dev/null
    echo "  Service installe et active: $SERVICE_DST"
else
    echo "  ATTENTION: $SERVICE_SRC introuvable, service non installe"
fi

echo ""
echo "=================================="
echo "  Installation terminée!"
echo "=================================="
echo ""
echo "Prochaines étapes:"
echo "  1. Créer config/secrets.yaml (sinon le serveur refuse de démarrer)"
echo "  2. Redémarrer si UART3 a été configuré"
echo "  3. Câbler le Pico et les moteurs"
echo ""
echo "Le robot démarre désormais tout seul au boot. Commandes utiles:"
echo "  sudo systemctl start robot      # démarrer maintenant"
echo "  sudo systemctl stop robot       # arrêter (coupe les moteurs)"
echo "  systemctl status robot          # état"
echo "  journalctl -u robot -f          # journal en direct"
echo "  sudo systemctl disable robot    # ne plus démarrer au boot"
echo ""
echo "Lancement manuel (démarrage auto désactivé, pour déboguer):"
echo "     cd $PI5_DIR && source venv/bin/activate && python main.py"
echo ""

if [ "$NEED_REBOOT" = true ]; then
    echo "ATTENTION: Un redémarrage est nécessaire pour activer UART3"
    read -p "Redémarrer maintenant? (o/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Oo]$ ]]; then
        sudo reboot
    fi
fi
