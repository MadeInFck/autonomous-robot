# Robot Autonome Mecanum - GPS + IMU

Robot autonome à roues Mecanum avec architecture distribuée Raspberry Pi 5 + Pico.

## Architecture

```
Raspberry Pi Pico (MicroPython)          Raspberry Pi 5 (Python)
┌─────────────────────────────┐         ┌──────────────────────────────┐
│ • NEO-6M GPS (UART1)        │         │ • 4 moteurs Mecanum (HW-249) │
│ • BMI160 IMU (I2C1)         │◄─UART3─►│ • Contrôle omnidirectionnel  │
│ • Acquisition capteurs 10Hz │         │ • Serveur Flask (WiFi)       │
│ • TX: GP0, RX: GP1          │         │ • TX: GPIO8, RX: GPIO9       │
└─────────────────────────────┘         └──────────────────────────────┘
                                                     │
                                                     ▼
                                        ┌──────────────────────────────┐
                                        │     Smartphone (WiFi)        │
                                        │  • Double joystick tactile   │
                                        │  • Télémétrie temps réel     │
                                        └──────────────────────────────┘
```

## Fonctionnalités

- **Contrôle Mecanum** : Déplacement omnidirectionnel (avant/arrière, strafe, rotation)
- **Interface smartphone** : Double joystick tactile (mouvement + rotation) via navigateur web
- **Télémétrie temps réel** : Position GPS, accéléromètre, gyroscope, vitesse, cap
- **Contrôle de vitesse** : Réglable de 30% à 100% depuis l'interface
- **Navigation autonome** : Suivi de waypoints GPS (en développement)

## Câblage

### UART Pi5 ↔ Pico (115200 baud)

| Pi 5 (broche)        | Pico (broche) | Description       |
|----------------------|---------------|-------------------|
| GPIO8 / TXD3 (24)    | GP1 (2)       | Pi5 TX → Pico RX  |
| GPIO9 / RXD3 (21)    | GP0 (1)       | Pi5 RX ← Pico TX  |
| GND (20)             | GND (3)       | Masse commune     |

### GPS NEO-6M → Pico (9600 baud)

| NEO-6M | Pico (broche) | Description |
|--------|---------------|-------------|
| TX     | GP5 (7)       | GPS TX → Pico RX (UART1) |
| RX     | GP4 (6)       | GPS RX ← Pico TX (UART1) |
| VCC    | 3V3           | Alimentation |
| GND    | GND           | Masse |

### IMU BMI160 → Pico (I2C1, 400 kHz)

| BMI160 | Pico (broche) | Description |
|--------|---------------|-------------|
| SDA    | GP14 (19)     | I2C1 Data   |
| SCL    | GP15 (20)     | I2C1 Clock  |
| VCC    | 3V3           | Alimentation |
| GND    | GND           | Masse |

Adresse I2C : `0x68` (SDO=LOW) ou `0x69` (SDO=HIGH)

### Moteurs Mecanum (HW-249 / L9110S)

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

PWM logiciel 100 Hz via `gpiod`. Alimentation moteurs : 5V.

## Installation

### Raspberry Pi 5

```bash
# Cloner le dépôt
git clone https://github.com/<user>/autonomous-robot.git ~/AutonomRobot

# Lancer le script d'installation
cd ~/AutonomRobot
bash scripts/setup_pi5_env.sh
```

Le script configure automatiquement :
- UART3 dans `/boot/firmware/config.txt` (nécessite un reboot)
- Environnement virtuel Python (`pi5/venv/`)
- Dépendances système (`python3-lgpio`, `libgpiod-dev`, etc.)
- Dépendances Python (`flask`, `pyserial`, `gpiod`, etc.)
- Permissions GPIO et série (groupes `gpio` et `dialout`)

### Raspberry Pi Pico

Copier le contenu du dossier `pico/` sur le Pico via Thonny ou `mpremote` :

```bash
mpremote cp pico/main.py :main.py
mpremote cp -r pico/lib/ :lib/
```

## Utilisation

### Démarrer le robot

```bash
cd ~/AutonomRobot/pi5
source venv/bin/activate
python main.py
```

Options :
- `--no-motors` : Désactiver les moteurs (test capteurs uniquement)
- `--no-sensors` : Désactiver la réception UART (test moteurs uniquement)
- `--port 8085` : Port du serveur web (défaut : 8085)

### Contrôle via smartphone

1. Connecter le smartphone au même réseau WiFi que la Pi5
2. Ouvrir `http://<ip-pi5>:8085` dans le navigateur
3. Utiliser le joystick gauche pour le déplacement, le droit pour la rotation
4. Régler la vitesse avec les boutons +/- (30-100%)

### Test UART sans capteurs

Le script `pico/test_uart.py` simule des données IMU et GPS pour tester la communication Pi5 ↔ Pico sans matériel capteur :

```bash
mpremote run pico/test_uart.py
```

## Structure du projet

```
autonomous-robot/
├── config/
│   ├── robot_config.yaml          # Configuration GPIO, UART, moteurs, PID
│   └── sensor_calibration.json    # Offsets calibration IMU et GPS
├── pico/                          # Code MicroPython (Pico)
│   ├── main.py                    # Application capteurs principale
│   ├── test_uart.py               # Test UART avec données simulées
│   └── lib/
│       ├── bmi160_raw.py          # Driver IMU BMI160
│       ├── gps_reader.py          # Lecteur GPS NEO-6M
│       ├── micropyGPS.py          # Parser NMEA (lib externe)
│       └── protocole.py           # Encodage/décodage trames UART
├── pi5/                           # Code Python (Pi5)
│   ├── main.py                    # Application principale
│   ├── requirements.txt           # Dépendances Python
│   ├── motors/
│   │   └── motor_controller.py    # Contrôleur Mecanum (gpiod + PWM)
│   ├── sensors/
│   │   └── uart_receiver.py       # Réception et décodage trames UART
│   ├── telemetry/
│   │   └── web_server.py          # Serveur Flask + interface joystick
│   └── tests/                     # Tests unitaires (à compléter)
└── scripts/
    └── setup_pi5_env.sh           # Script d'installation Pi5
```

## Protocole UART

Trames de 34 bytes avec encapsulation STX/ETX et CRC-8 :

```
Octet  0     : STX (0x02)
Octet  1     : LEN (30 = longueur payload)
Octets 2-31  : PAYLOAD (30 bytes)
  2-3        : Séquence          (uint16, 0-65535)
  4-5        : Accéléromètre X   (int16, en mg)
  6-7        : Accéléromètre Y   (int16, en mg)
  8-9        : Accéléromètre Z   (int16, en mg)
  10-11      : Gyroscope X       (int16, en 0.1°/s)
  12-13      : Gyroscope Y       (int16, en 0.1°/s)
  14-15      : Gyroscope Z       (int16, en 0.1°/s)
  16-19      : Latitude          (int32, en microdegrés)
  20-23      : Longitude         (int32, en microdegrés)
  24-25      : Altitude          (int16, en décimètres)
  26-27      : Vitesse           (uint16, en cm/s)
  28-29      : Cap               (uint16, en 0.01°)
  30         : Satellites        (uint8, nombre de satellites)
  31         : Fix qualité       (uint8, 0=no fix, 1=GPS, 2=DGPS)
Octet  32    : CRC-8 (polynôme 0x07, sur le payload)
Octet  33    : ETX (0x03)
```

Fréquence de transmission : **10 Hz** (100 ms).

## API Web

Le serveur Flask expose les endpoints suivants :

| Méthode | Endpoint       | Description                        |
|---------|----------------|------------------------------------|
| POST    | `/api/move`    | Commande de mouvement              |
| GET     | `/api/sensors` | Dernières données capteurs         |
| GET     | `/api/status`  | État des composants                |
| POST    | `/api/stop`    | Arrêt d'urgence des moteurs       |

### POST `/api/move`

```json
{
  "vx": -1.0,       // Vélocité latérale (-1 à 1)
  "vy": 0.5,        // Vélocité longitudinale (-1 à 1)
  "omega": 0.0,     // Rotation (-1 à 1)
  "speed": 70       // Vitesse max en % (0-100)
}
```

### GET `/api/sensors`

```json
{
  "sequence": 1234,
  "acc_x": 0.012, "acc_y": -0.005, "acc_z": 1.001,
  "gyr_x": 0.1, "gyr_y": -0.2, "gyr_z": 0.05,
  "latitude": 48.8566, "longitude": 2.3522,
  "altitude": 35.2, "speed": 1.5, "speed_kmh": 5.4,
  "heading": 127.5, "satellites": 9, "has_fix": true,
  "timestamp": 1700000000.0
}
```

## Cinématique Mecanum

```
FL = vy + vx + omega
FR = vy - vx - omega
RL = vy - vx + omega
RR = vy + vx - omega
```

Les vitesses sont normalisées par la valeur maximale pour éviter la saturation.

## TODO

- [x] Valider câblage moteurs et sens de rotation
- [x] Tester interface joystick sur smartphone
- [x] Calibrer IMU BMI160 au repos
- [x] Valider réception GPS en extérieur (premier fix)
- [ ] Écrire tests unitaires (`pi5/tests/`)
- [ ] Intégrer navigation autonome par waypoints GPS
- [ ] Ajouter détection d'obstacles (lidar/ultrason)

## Licence

Projet personnel - Tous droits réservés
