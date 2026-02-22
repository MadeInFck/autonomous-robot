*[English version](README.md)*

# Robot Autonome Mecanum - GPS + IMU

Robot autonome à roues Mecanum avec architecture distribuée Raspberry Pi 5 + Pico.

## Architecture

```
Raspberry Pi Pico (MicroPython)          Raspberry Pi 5 (Python)
┌─────────────────────────────┐         ┌──────────────────────────────┐
│ • NEO-6M GPS (UART1)        │         │ • 4 moteurs Mecanum (HW-249) │
│ • IMU I2C1 (BMI160/BNO085)  │◄─UART3─►│ • Contrôle omnidirectionnel  │
│ • Acquisition capteurs 10Hz │         │ • Serveur Flask (WiFi)       │
│ • TX: GP0, RX: GP1          │         │ • TX: GPIO8, RX: GPIO9       │
└─────────────────────────────┘         │ • Raspberry Pi AI Camera     │
                                         │   (IMX500, inférence on-chip)│
                                         └──────────────────────────────┘
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
- **Télémétrie temps réel** : Position GPS, données IMU (accéléromètre + cap), vitesse, satellites
- **Options IMU** : BMI160 (accel + gyro) ou BNO085 9-DOF (accel + gyro + mag, cap absolu à toute vitesse)
- **Contrôle de vitesse** : Réglable de 30% à 100% depuis l'interface
- **Navigation autonome** : Patrouille par waypoints GPS avec évitement d'obstacles (LiDAR)
- **LiDAR RPLidar C1** : Scan 360°, détection d'obstacles, grille d'occupation
- **Caméra IA (IMX500)** : Inférence EfficientDet Lite0 on-chip — détection personnes/animaux avec alertes toast, flux MJPEG en direct
- **Sécurité** : Authentification HTTP Basic Auth (scrypt), TLS/HTTPS optionnel

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

### IMU → Pico (I2C1, 400 kHz)

Deux options IMU sont supportées sur les mêmes broches. Choisir selon le matériel disponible :

**Option A — BMI160** (accéléromètre + gyroscope, courant/économique)

> Attention : beaucoup de modules "BMI160" bon marché (AliExpress) sont des clones
> (chip ID ≠ 0xD1) avec le gyroscope non fonctionnel. Lancer `pico/diag_bmi160.py`
> dans Thonny pour vérifier.

| BMI160 | Pico (broche) | Description  |
|--------|---------------|--------------|
| SDA    | GP14 (19)     | I2C1 Data    |
| SCL    | GP15 (20)     | I2C1 Clock   |
| VCC    | 3V3           | Alimentation |
| GND    | GND           | Masse        |

Adresse I2C : `0x68` (SDO=LOW) ou `0x69` (SDO=HIGH)
Driver : `pico/lib/bmi160_raw.py`

**Option B — BNO085** (9-DOF fusion : accel + gyro + mag, recommandé)

Fournit le cap absolu (yaw) référencé au nord magnétique, même à l'arrêt ou à basse vitesse.
Utiliser le [breakout Adafruit BNO085](https://www.adafruit.com/product/4754) (mode I2C, par défaut).

| BNO085 | Pico (broche) | Description  |
|--------|---------------|--------------|
| SDA    | GP14 (19)     | I2C1 Data    |
| SCL    | GP15 (20)     | I2C1 Clock   |
| VCC    | 3V3           | Alimentation |
| GND    | GND           | Masse        |

Adresse I2C : `0x4A` (PS0=LOW, PS1=LOW) ou `0x4B` (PS0=HIGH)
Driver : `pico/lib/bno085.py`

> Les deux capteurs partagent le même bus I2C (adresses différentes) et peuvent coexister si besoin.

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
- `--no-lidar` : Désactiver le LiDAR
- `--no-camera` : Désactiver la caméra IA
- `--camera-conf 0.50` : Seuil de confiance de détection (défaut : 0.50)
- `--lidar-port /dev/ttyUSB0` : Port série du LiDAR (défaut : `/dev/ttyUSB0`)
- `--port 8085` : Port du serveur web (défaut : 8085)
- `--no-auth` : Désactiver l'authentification (dev/debug uniquement)

### Contrôle via smartphone

1. Connecter le smartphone au même réseau WiFi que la Pi5
2. Ouvrir `https://<ip-pi5>:8085` dans le navigateur (accepter le certificat auto-signé)
3. Se connecter avec les identifiants configurés dans `config/secrets.yaml`
4. Utiliser le joystick gauche pour le déplacement, le droit pour la rotation
5. Régler la vitesse avec les boutons +/- (30-100%)

## Sécurité

Les secrets (auth, TLS) sont dans `config/secrets.yaml` (ignoré par git). Pour configurer :

```bash
cp config/secrets.yaml.example config/secrets.yaml
# Editer secrets.yaml avec vos identifiants
```

### Authentification

L'interface web est protégée par HTTP Basic Auth. Les identifiants (`username` et `password_hash`) sont configurés dans `config/secrets.yaml`.

Pour générer un hash de mot de passe :

```bash
python -c "from werkzeug.security import generate_password_hash; print(generate_password_hash('MON_MOT_DE_PASSE'))"
```

Pour désactiver l'authentification (dev uniquement) : `python main.py --no-auth`

### TLS / HTTPS

Le serveur supporte HTTPS via un certificat auto-signé. Les fichiers de certificat sont stockés dans `config/ssl/` (ignoré par git).

**Générer le certificat sur la Pi5 :**

```bash
mkdir -p ~/AutonomRobot/config/ssl
openssl req -x509 -newkey rsa:2048 \
  -keyout ~/AutonomRobot/config/ssl/key.pem \
  -out ~/AutonomRobot/config/ssl/cert.pem \
  -days 3650 -nodes \
  -subj '/CN=robot-mecanum'
```

La configuration TLS dans `config/secrets.yaml` :

```yaml
tls:
  cert: config/ssl/cert.pem
  key: config/ssl/key.pem
```

Si les fichiers de certificat n'existent pas, le serveur démarre automatiquement en HTTP.

## Structure du projet

```
autonomous-robot/
├── config/
│   ├── robot_config.yaml          # Configuration GPIO, UART, moteurs, LiDAR, nav
│   ├── secrets.yaml               # Auth + TLS (ignoré par git)
│   ├── secrets.yaml.example       # Template pour secrets.yaml
│   ├── patrol_route.json          # Waypoints de patrouille (créé automatiquement)
│   ├── sensor_calibration.json    # Offsets calibration IMU et GPS
│   └── ssl/                       # Certificats TLS (ignoré par git)
│       ├── cert.pem
│       └── key.pem
├── pico/                          # Code MicroPython (Pico)
│   ├── main.py                    # Application capteurs principale
│   ├── diag_bmi160.py             # Script diagnostic BMI160 (à lancer dans Thonny)
│   └── lib/
│       ├── bmi160_raw.py          # Driver IMU BMI160 (option A)
│       ├── bno085.py              # Driver IMU BNO085 9-DOF (option B, recommandé)
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
│   ├── lidar/                     # RPLidar C1 (scans + détection)
│   │   ├── scanner.py             # Acquisition scans 360°
│   │   ├── data.py                # Modèles ScanPoint, Scan
│   │   └── detection/             # DBSCAN, tracking, grille occupation
│   ├── navigation/                # Navigation autonome
│   │   ├── patrol_manager.py      # Gestion waypoints GPS
│   │   ├── obstacle_avoider.py    # Évitement obstacles lidar
│   │   └── pilot.py               # Pilote autonome (GPS + lidar)
│   ├── camera/
│   │   └── ai_camera.py           # Caméra IA IMX500 — thread EfficientDet Lite0 en arrière-plan
│   ├── telemetry/
│   │   └── web_server.py          # Serveur Flask + joystick + lidar + patrouille + caméra
│   └── tests/                     # Tests unitaires
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
  10-11      : Gyroscope X       (int16, en 0.1°/s — BMI160 uniquement, 0 avec BNO085)
  12-13      : Gyroscope Y       (int16, en 0.1°/s — BMI160 uniquement, 0 avec BNO085)
  14-15      : Gyroscope Z       (int16, en 0.1°/s — BMI160 uniquement, 0 avec BNO085)
  16-19      : Latitude          (int32, en microdegrés)
  20-23      : Longitude         (int32, en microdegrés)
  24-25      : Altitude          (int16, en décimètres)
  26-27      : Vitesse           (uint16, en cm/s)
  28-29      : Cap               (uint16, en 0.01° — GPS COG avec BMI160, yaw BNO085 avec BNO085)
  30         : Satellites        (uint8, nombre de satellites)
  31         : Fix qualité       (uint8, 0=no fix, 1=GPS, 2=DGPS)
Octet  32    : CRC-8 (polynôme 0x07, sur le payload)
Octet  33    : ETX (0x03)
```

Fréquence de transmission : **10 Hz** (100 ms).

## API Web

Le serveur Flask expose les endpoints suivants :

| Méthode | Endpoint              | Description                          |
|---------|-----------------------|--------------------------------------|
| POST    | `/api/move`           | Commande de mouvement                |
| GET     | `/api/sensors`        | Dernières données capteurs           |
| GET     | `/api/status`         | État des composants                  |
| POST    | `/api/stop`           | Arrêt d'urgence des moteurs         |
| GET     | `/api/lidar`          | Derniers points du scan LiDAR        |
| GET     | `/api/waypoints`      | Liste des waypoints + index courant  |
| POST    | `/api/waypoints/record` | Enregistre la position GPS actuelle |
| DELETE  | `/api/waypoints/<idx>` | Supprime un waypoint                |
| POST    | `/api/patrol/start`   | Lance la patrouille autonome         |
| POST    | `/api/patrol/stop`    | Arrête la patrouille                 |
| GET     | `/api/patrol/status`  | État de la patrouille                |
| GET     | `/api/detections`     | Dernières détections caméra IA (60 s)|
| GET     | `/api/stream`         | Flux vidéo MJPEG en direct           |

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
  "gyr_x": 0.1, "gyr_y": -0.2, "gyr_z": 0.05,  // BMI160 uniquement ; toujours 0.0 avec BNO085
  "latitude": 48.8566, "longitude": 2.3522,
  "altitude": 35.2, "speed": 1.5, "speed_kmh": 5.4,
  "heading": 127.5,  // GPS COG (BMI160) ou yaw absolu BNO085 (BNO085)
  "satellites": 9, "has_fix": true,
  "timestamp": 1700000000.0
}
```

### GET `/api/detections`

```json
{
  "camera_ok": true,
  "detections": [
    {"label": "person", "confidence": 0.87, "timestamp": 1700000000.0, "age_s": 1.2, "is_alert": true},
    {"label": "dog",    "confidence": 0.63, "timestamp": 1700000000.0, "age_s": 1.2, "is_alert": false}
  ]
}
```

`is_alert: true` lors de la première occurrence d'un label d'alerte dans la fenêtre de cooldown de 5 secondes.
Labels d'alerte : `person`, `cat`, `dog`, `horse`, `sheep`, `cow`, `bear`, `zebra`, `giraffe`.

### GET `/api/stream`

Flux MJPEG multipart (`multipart/x-mixed-replace`). Utilisable directement comme src d'une balise `<img>`.

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
- [x] Calibrer IMU BMI160 au repos (accéléromètre OK ; gyroscope HS sur les clones — voir diag_bmi160.py)
- [x] Valider réception GPS en extérieur (premier fix)
- [x] Écrire tests unitaires (`pi5/tests/`)
- [x] Intégrer navigation autonome par waypoints GPS
- [x] Ajouter détection d'obstacles (RPLidar C1)
- [x] Sécuriser l'interface web (auth + TLS)
- [x] Écrire driver BNO085 (`pico/lib/bno085.py`) — cap absolu à toute vitesse
- [ ] Intégrer BNO085 dans `pico/main.py` (module en commande) — remplace GPS COG pour le cap
- [ ] Tester patrouille autonome (suivi waypoints GPS, arrêt, changement de mode)
- [ ] Tester l'évitement d'obstacles lidar (scénarios réels : obstacles statiques, distance de passage, récupération après évitement)
- [x] Intégrer Raspberry Pi AI Camera (IMX500 / EfficientDet Lite0 — détection personnes/animaux, alertes toast, flux MJPEG)
- [ ] Notifications push + sirène en cas de détection d'intrusion
- [ ] Créer une app iOS native à l'image de l'interface web existante

## Licence

Usage non commercial uniquement. Voir [LICENSE](LICENSE) pour les details.
