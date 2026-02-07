#!/bin/bash
# Script d'installation pour Raspberry Pi 5
# Robot Mecanum Autonome
# Déploiement dans ~/AutonomRobot

set -e

# Dossier d'installation
INSTALL_DIR="$HOME/AutonomRobot"

echo "=================================="
echo "  Setup Robot Mecanum - Pi 5"
echo "  Dossier: $INSTALL_DIR"
echo "=================================="

# Vérifier qu'on est sur une Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "ATTENTION: Ce script est conçu pour Raspberry Pi"
fi

# 1. Activer UART3
echo ""
echo "[1/5] Configuration UART3..."
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

# 2. Installer les dépendances système
echo ""
echo "[2/5] Installation des dépendances système..."
sudo apt-get update
sudo apt-get install -y \
    python3-venv \
    python3-pip \
    python3-lgpio \
    libgpiod-dev

# 3. Créer l'environnement virtuel
echo ""
echo "[3/5] Création de l'environnement virtuel..."

# Se placer dans le dossier pi5
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

# 4. Installer les dépendances Python
echo ""
echo "[4/5] Installation des dépendances Python..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# 5. Permissions série
echo ""
echo "[5/5] Configuration des permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER

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
