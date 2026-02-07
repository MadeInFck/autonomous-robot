# Robot Autonome Mecanum - GPS + IMU

Robot autonome à roues Mecanum avec architecture distribuée Raspberry Pi 5 + Pico.

## Architecture

```
Raspberry Pi Pico (MicroPython)          Raspberry Pi 5 (Python)
┌─────────────────────────────┐         ┌──────────────────────────────┐
│ • NEO-6M GPS (UART0)        │         │ • 4 moteurs Mecanum (HW-249) │
│ • BMI160 IMU (I2C1)         │◄─UART3─►│ • Contrôle omnidirectionnel  │
│ • Acquisition capteurs      │         │ • Serveur Flask (WiFi)       │
│ • TX: GP0, RX: GP1          │         │ • TX: GPIO8, RX: GPIO9       │
└─────────────────────────────┘         └──────────────────────────────┘
                                                     │
                                                     ▼
                                        ┌──────────────────────────────┐
                                        │     Smartphone (WiFi)        │
                                        │  • Interface joystick web    │
                                        │  • Télémétrie temps réel     │
                                        └──────────────────────────────┘
```

## Fonctionnalités

- **Contrôle Mecanum** : Déplacement omnidirectionnel (avant/arrière, strafe, rotation)
- **Interface smartphone** : Joystick tactile via navigateur web
- **Télémétrie temps réel** : Position GPS, données IMU
- **Navigation autonome** : Suivi de waypoints GPS (future)

## Câblage

### UART3 (Pi5 ↔ Pico)

| Pi 5 (broche)        | Pico (broche) | Description    |
|----------------------|---------------|----------------|
| GPIO8 / TXD3 (24)    | GP1 (2)       | Pi5 TX → Pico RX |
| GPIO9 / RXD3 (21)    | GP0 (1)       | Pi5 RX ← Pico TX |
| GND (20)             | GND (3)       | Masse commune  |

### Moteurs Mecanum (HW-249)

```
Vue de dessus du robot:
    FL ──────── FR
    │    ↑     │
    │  AVANT   │
    │          │
    RL ──────── RR
```

| Moteur | GPIO IN1 | GPIO IN2 |
|--------|----------|----------|
| FL     | 17       | 27       |
| FR     | 22       | 23       |
| RL     | 24       | 25       |
| RR     | 5        | 6        |

## Installation

### Raspberry Pi 5

```bash
# Copier le projet dans ~/AutonomRobot
cp -r autonomous-robot ~/AutonomRobot

# Lancer le script d'installation
cd ~/AutonomRobot
bash scripts/setup_pi5_env.sh
```

Le script configure automatiquement:
- UART3 (nécessite un reboot)
- Environnement virtuel Python
- Dépendances système et Python
- Permissions GPIO et série

### Raspberry Pi Pico

Copier les fichiers `pico/` sur le Pico via Thonny ou `mpremote`.

## Utilisation

### Démarrer le robot

```bash
cd ~/AutonomRobot/pi5
source venv/bin/activate
python main.py
```

Options:
- `--no-motors` : Désactiver les moteurs (test)
- `--no-sensors` : Désactiver la réception UART
- `--port 8080` : Port du serveur web

### Contrôle via smartphone

1. Connecter le smartphone au même réseau WiFi que la Pi5
2. Ouvrir `http://<ip-pi5>:8080` dans le navigateur
3. Utiliser le joystick tactile pour piloter

## Structure du projet

```
AutonomRobot/
├── config/
│   └── robot_config.yaml       # Configuration GPIO, UART, moteurs
├── pico/                       # Code MicroPython (Pico)
│   ├── main.py
│   └── lib/
│       ├── bmi160.py           # Driver IMU
│       ├── micropyGPS.py       # Parser GPS
│       ├── sensor_manager.py   # Gestionnaire capteurs
│       └── uart_transmitter.py # Transmission vers Pi5
├── pi5/                        # Code Python (Pi5)
│   ├── main.py                 # Application principale
│   ├── requirements.txt
│   ├── motors/
│   │   └── motor_controller.py # Contrôleur Mecanum
│   ├── sensors/
│   │   └── uart_receiver.py    # Réception UART
│   └── telemetry/
│       └── web_server.py       # Serveur Flask + joystick
└── scripts/
    └── setup_pi5_env.sh        # Script d'installation
```

## Protocole UART

Trames de 32 bytes avec Magic Header (0xAA55) et CRC-16:
- Header: Magic (2), Type (1), Sequence (1)
- GPS: Latitude/Longitude (8), Vitesse (2), Cap (2), Satellites (1), Fix (1)
- IMU: Pitch/Roll/Yaw (6)
- Timestamp (4), Status (1), Réservé (1)
- CRC-16 (2)

## TODO

- [ ] Valider câblage moteurs
- [ ] Test interface joystick sur smartphone
- [ ] Intégrer navigation autonome par waypoints
- [ ] Ajouter détection d'obstacles (lidar)

## Licence

Projet personnel - Tous droits réservés
