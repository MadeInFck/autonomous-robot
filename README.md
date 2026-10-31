*[Version française](README_fr.md)*

# Autonomous Mecanum Robot - GPS + IMU

Autonomous Mecanum wheeled robot with distributed architecture Raspberry Pi 5 + Pico.

## Architecture

```
Raspberry Pi Pico (MicroPython)          Raspberry Pi 5 (Python)
┌─────────────────────────────┐         ┌──────────────────────────────┐
│ • NEO-6M GPS (UART1)        │         │ • 4 Mecanum motors (HW-249)  │
│ • IMU I2C1 (BMI160/BNO085)  │◄─UART3─►│ • Omnidirectional control    │
│ • Sensor acquisition 10Hz   │         │ • Flask web server (WiFi)    │
│ • TX: GP0, RX: GP1          │         │ • TX: GPIO8, RX: GPIO9       │
└─────────────────────────────┘         │ • Raspberry Pi AI Camera     │
                                         │   (IMX500, on-chip inference)│
                                         └──────────────────────────────┘
                                                     │
                                                     ▼
                                        ┌──────────────────────────────┐
                                        │     Smartphone (WiFi)        │
                                        │  • Dual touch joystick       │
                                        │  • Real-time telemetry       │
                                        └──────────────────────────────┘
```

## Features

- **Mecanum Control**: Omnidirectional movement (forward/backward, strafe, rotation)
- **Smartphone Interface**: Dual touch joystick (movement + rotation) via web browser
- **Real-time Telemetry**: GPS position, IMU data (accelerometer + heading), speed, satellites
- **IMU Options**: BMI160 (accel + gyro) or BNO085 9-DOF (accel + gyro + mag, absolute heading at any speed)
- **Speed Control**: Adjustable from 30% to 100% from the interface
- **Autonomous Navigation**: GPS waypoint patrol with obstacle avoidance (LiDAR)
- **Failsafes**: motors cut when the joystick goes silent (> 0.5 s), when UART/GPS frames (> 2 s)
  or LiDAR scans (> 1 s) freeze during a patrol, or when the pilot thread stops beating (> 3 s)
- **LiDAR RPLidar C1**: 360° scan, obstacle detection (numpy DBSCAN) with temporal tracking
- **AI Camera (IMX500)**: EfficientDet Lite0 on-chip inference — person/animal detection with toast alerts, live MJPEG stream
- **Security**: HTTP Basic Auth (scrypt), optional TLS/HTTPS

## Wiring

### UART Pi5 ↔ Pico (115200 baud)

| Pi 5 (pin)           | Pico (pin)    | Description       |
|----------------------|---------------|-------------------|
| GPIO8 / TXD3 (24)    | GP1 (2)       | Pi5 TX → Pico RX  |
| GPIO9 / RXD3 (21)    | GP0 (1)       | Pi5 RX ← Pico TX  |
| GND (20)             | GND (3)       | Common ground      |

### GPS NEO-6M → Pico (115200 baud)

| NEO-6M | Pico (pin)    | Description |
|--------|---------------|-------------|
| TX     | GP5 (7)       | GPS TX → Pico RX (UART1) |
| RX     | GP4 (6)       | GPS RX ← Pico TX (UART1) |
| VCC    | 3V3           | Power supply |
| GND    | GND           | Ground |

### IMU → Pico (I2C1, 100 kHz)

Two IMU options are supported on the same pins. `pico/main.py` **auto-detects** whichever is connected: BNO085 at `0x4A` takes priority; BMI160 at `0x68` is the fallback. No configuration needed.

**Option A — BMI160** (accelerometer + gyroscope, cheap/common)

> Note: many cheap "BMI160" modules on AliExpress are clones (chip ID ≠ 0xD1) with a
> non-functional gyroscope. The PMU status printed when `pico/main.py` starts
> (`Gyr=OK` / `SUSPEND`) tells you which one you have.

| BMI160 | Pico (pin) | Description  |
|--------|------------|--------------|
| SDA    | GP14 (19)  | I2C1 Data    |
| SCL    | GP15 (20)  | I2C1 Clock   |
| VCC    | 3V3        | Power supply |
| GND    | GND        | Ground       |

I2C address: `0x68` (SDO=LOW) or `0x69` (SDO=HIGH)
Driver: `pico/lib/bmi160_raw.py`

**Option B — BNO085** (9-DOF fusion: accel + gyro + mag, recommended)

Provides absolute heading (yaw) referenced to magnetic north, even at rest or low speed.
Use the [Adafruit BNO085 breakout](https://www.adafruit.com/product/4754) (I2C mode, default).

| BNO085 | Pico (pin) | Description  |
|--------|------------|--------------|
| SDA    | GP14 (19)  | I2C1 Data    |
| SCL    | GP15 (20)  | I2C1 Clock   |
| VCC    | 3V3        | Power supply |
| GND    | GND        | Ground       |

I2C address: `0x4A` (PS0=LOW, PS1=LOW) or `0x4B` (PS0=HIGH)
Driver: `pico/lib/bno085.py`

> To verify the BNO085 before full integration, run `pico/lib/bno085.py` directly in Thonny
> (the `__main__` block at the bottom prints yaw/pitch/roll in a live loop).

> Both sensors share the same I2C bus (different addresses) and can coexist if needed.

**BNO085 calibration**: the sensor requires a 3D magnetic calibration (figure-8 motions while tilting the robot in all directions). Once `heading_accuracy_deg < 10°`, calibration is automatically saved to the sensor's internal flash (DCD) and reloaded on the next boot. The calibrated state is also persisted on the Pi5 at `~/.config/robot/imu_cal_state.json` so the UI shows green immediately after reboot.

**Heading convention**: N=0°, E=90°, S=180°, W=270° (clockwise = increasing). Mount the sensor flat with the X arrow pointing toward the robot's front.

### Mecanum Motors (HW-249 / L9110S)

```
Top view of the robot:
    FL ──────── FR
    │    ↑     │
    │  FRONT   │
    │          │
    RL ──────── RR
```

| Motor  | GPIO IN1 | GPIO IN2 |
|--------|----------|----------|
| FL     | 17       | 27       |
| FR     | 22       | 23       |
| RL     | 24       | 25       |
| RR     | 5        | 6        |

Software PWM 100 Hz via `gpiod`. Motor power supply: 5V, **5A minimum** (4 mecanum motors under load draw more than a 4A supply can provide — insufficient amperage causes brownouts/resets that can look like software bugs).

## Installation

### Raspberry Pi 5

```bash
# Clone the repository
git clone https://github.com/<user>/autonomous-robot.git ~/AutonomRobot

# Run the setup script
cd ~/AutonomRobot
bash scripts/setup_pi5_env.sh
```

The script automatically configures:
- UART3 in `/boot/firmware/config.txt` (requires reboot)
- Python virtual environment (`pi5/venv/`)
- System dependencies (`python3-libgpiod` — the libgpiod v2 bindings behind
  `import gpiod`, visible in the venv through `--system-site-packages` —, `libgpiod-dev`, etc.)
- Python dependencies (`flask`, `cheroot`, `pyserial`, `numpy`, `rplidarc1`, etc.;
  no more scikit-learn/scipy: lidar clustering is plain numpy)
- GPIO and serial permissions (`gpio` and `dialout` groups)
- Sudoers rule `/etc/sudoers.d/robot-power` restricted to `shutdown now` and
  `reboot` only, used by the web UI power buttons (never grant a broad
  NOPASSWD to the user running the server)

- systemd service `robot.service` (generated from `scripts/robot.service`),
  **enabled at boot**

Authentication is required by default: the server refuses to start if
`config/secrets.yaml` is missing or invalid (copy `config/secrets.yaml.example`),
unless `--no-auth` is passed explicitly.

### Starting at boot

The service is installed and enabled by `setup_pi5_env.sh`. At boot the robot
comes up with **motors idle and no patrol running**: only the web interface
becomes available, the patrol still has to be started manually.

```bash
sudo systemctl start robot      # start now
sudo systemctl stop robot       # stop (motors cut cleanly)
systemctl status robot          # status
journalctl -u robot -f          # live log
sudo systemctl disable robot    # stop starting at boot
```

`systemctl stop/restart` sends SIGTERM, which `main.py` traps to run
`RobotApp.shutdown()` (pilot, motors, lidar, UART and camera all stopped) — the
robot cannot keep moving across a service restart. On a crash,
`Restart=on-failure` brings it back in the safe state (IDLE, motors stopped);
an invalid `secrets.yaml` fails 5 times and systemd then gives up instead of
looping.

To debug, stop the service and run it by hand:

```bash
sudo systemctl stop robot
cd ~/AutonomRobot/pi5 && source venv/bin/activate && python main.py
```

### Raspberry Pi Pico

Copy the `pico/` folder contents to the Pico via Thonny or `mpremote`:

```bash
mpremote cp pico/main.py :main.py
mpremote cp -r pico/lib/ :lib/
```

## Usage

### Start the robot

```bash
cd ~/AutonomRobot/pi5
source venv/bin/activate
python main.py
```

Options:
- `--no-motors`: Disable motors (sensor testing only)
- `--no-sensors`: Disable UART reception (motor testing only)
- `--no-lidar`: Disable LiDAR
- `--no-camera`: Disable the AI camera
- `--camera-conf 0.50`: Detection confidence threshold (default: 0.50)
- `--lidar-port /dev/ttyUSB0`: LiDAR serial port (default: `/dev/ttyUSB0`)
- `--port 8085`: Web server port (default: 8085)
- `--no-auth`: Disable authentication (dev/debug only)

### Smartphone control

1. Connect your smartphone to the same WiFi network as the Pi5
2. Open `https://<pi5-ip>:8085` in your browser (accept the self-signed certificate)
3. Log in with the credentials configured in `config/secrets.yaml`
4. Use the left joystick for movement, the right one for rotation
5. Adjust speed with +/- buttons (30-100%)

## Security

Secrets (auth, TLS) are stored in `config/secrets.yaml` (git-ignored). To configure:

```bash
cp config/secrets.yaml.example config/secrets.yaml
# Edit secrets.yaml with your credentials
```

### Authentication

The web interface is protected by HTTP Basic Auth. Credentials (`username` and `password_hash`) are configured in `config/secrets.yaml`.

To generate a password hash:

```bash
python -c "from werkzeug.security import generate_password_hash; print(generate_password_hash('MY_PASSWORD'))"
```

To disable authentication (dev only): `python main.py --no-auth`

### TLS / HTTPS

The server supports HTTPS via a self-signed certificate. Certificate files are stored in `config/ssl/` (git-ignored).

**Generate the certificate on the Pi5:**

```bash
mkdir -p ~/AutonomRobot/config/ssl
openssl req -x509 -newkey rsa:2048 \
  -keyout ~/AutonomRobot/config/ssl/key.pem \
  -out ~/AutonomRobot/config/ssl/cert.pem \
  -days 3650 -nodes \
  -subj '/CN=robot-mecanum'
```

TLS configuration in `config/secrets.yaml`:

```yaml
tls:
  cert: config/ssl/cert.pem
  key: config/ssl/key.pem
```

If certificate files don't exist, the server automatically starts in HTTP mode.

## Project Structure

```
autonomous-robot/
├── config/
│   ├── robot_config.yaml          # GPIO, UART, motors, LiDAR, navigation config
│   ├── secrets.yaml               # Auth + TLS (git-ignored)
│   ├── secrets.yaml.example       # Template for secrets.yaml
│   ├── patrol_route.json          # Patrol waypoints (auto-created)
│   └── ssl/                       # TLS certificates (git-ignored)
│       ├── cert.pem
│       └── key.pem
├── pico/                          # MicroPython code (Pico)
│   ├── main.py                    # Main sensor application
│   └── lib/
│       ├── bmi160_raw.py          # BMI160 IMU driver (option A)
│       ├── bno085.py              # BNO085 9-DOF IMU driver (option B, recommended)
│       ├── gps_reader.py          # NEO-6M GPS reader
│       ├── micropyGPS.py          # NMEA parser (external lib)
│       └── protocole.py           # UART frame encoding/decoding
├── pi5/                           # Python code (Pi5)
│   ├── main.py                    # Main application
│   ├── requirements.txt           # Python dependencies
│   ├── motors/
│   │   └── motor_controller.py    # Mecanum controller (gpiod + PWM)
│   ├── sensors/
│   │   └── uart_receiver.py       # UART frame reception and decoding
│   ├── lidar/                     # RPLidar C1 (scans + detection)
│   │   ├── scanner.py             # 360° scan acquisition
│   │   ├── data.py                # ScanPoint, Scan models
│   │   └── detection/             # DBSCAN (numpy), wall/object classification, tracking
│   ├── navigation/                # Autonomous navigation
│   │   ├── patrol_manager.py      # GPS waypoint management
│   │   ├── obstacle_avoider.py    # LiDAR obstacle avoidance
│   │   └── pilot.py               # Autonomous pilot (GPS + LiDAR)
│   ├── camera/
│   │   └── ai_camera.py           # IMX500 AI camera — dedicated subprocess (no GIL), 4 fps detection / 15 fps stream
│   ├── telemetry/
│   │   ├── web_server.py          # Flask server + joystick + LiDAR + patrol + camera
│   │   └── templates/index.html   # Web UI (HTML/JS)
│   └── tests/                     # Unit tests (pytest, no hardware needed)
└── scripts/
    ├── setup_pi5_env.sh           # Pi5 setup script
    ├── robot.service              # systemd unit template (installed by setup_pi5_env.sh)
    ├── setup_ap.sh                # RobotPatroller WiFi access point (NetworkManager)
    └── switch-to-ap.sh / switch-to-client.sh   # Toggle AP ↔ home WiFi
```

## UART Protocol

34-byte frames with STX/ETX encapsulation and CRC-8:

```
Byte   0     : STX (0x02)
Byte   1     : LEN (30 = payload length)
Bytes  2-31  : PAYLOAD (30 bytes)
  2-3        : Sequence          (uint16, 0-65535)
  4-5        : Accelerometer X   (int16, in mg — with BNO085: pitch in 0.01°)
  6-7        : Accelerometer Y   (int16, in mg — with BNO085: roll in 0.01°)
  8-9        : Accelerometer Z   (int16, in mg)
  10-11      : Gyroscope X       (int16, in 0.1°/s — BMI160 ; always 0 with BNO085)
  12-13      : Gyroscope Y       (int16, in 0.1°/s — BMI160 ; always 0 with BNO085)
  14-15      : Gyroscope Z       (int16, in 0.1°/s — BMI160 ; with BNO085 = heading_accuracy_deg × 10,
                                  max 1800 for 180°)
  16-19      : Latitude          (int32, in microdegrees)
  20-23      : Longitude         (int32, in microdegrees)
  24-25      : Altitude          (int16, in decimeters)
  26-27      : Speed             (uint16, in cm/s)
  28-29      : Heading           (uint16, in 0.01° — GPS COG with BMI160, BNO085 yaw with BNO085)
  30         : Satellites        (uint8, satellite count)
  31         : Fix quality/flags (uint8 — bits 0-3: 0=no fix, 1=GPS, 2=DGPS;
                                  bit 7: IMU is BNO085. Both sides must be
                                  deployed together when this byte changes)
Byte   32    : CRC-8 (polynomial 0x07, over payload)
Byte   33    : ETX (0x03)
```

Transmission rate: **10 Hz** (100 ms).

## Web API

The Flask server exposes the following endpoints:

| Method  | Endpoint              | Description                          |
|---------|-----------------------|--------------------------------------|
| POST    | `/api/move`           | Movement command                     |
| GET     | `/api/sensors`        | Latest sensor data                   |
| GET     | `/api/status`         | Component status                     |
| POST    | `/api/stop`           | Emergency motor stop                 |
| GET     | `/api/lidar`          | Latest LiDAR scan points             |
| GET     | `/api/waypoints`      | Waypoint list + current index        |
| POST    | `/api/waypoints/record` | Record current GPS position        |
| DELETE  | `/api/waypoints/<idx>` | Delete a waypoint                   |
| POST    | `/api/patrol/start`   | Start autonomous patrol              |
| POST    | `/api/patrol/stop`    | Stop patrol                          |
| GET     | `/api/patrol/status`  | Patrol status                        |
| GET     | `/api/detections`     | Recent AI camera detections (60 s)   |
| GET     | `/api/stream`         | Live MJPEG video stream              |

### POST `/api/move`

```json
{
  "vx": -1.0,       // Lateral velocity (-1 to 1)
  "vy": 0.5,        // Longitudinal velocity (-1 to 1)
  "omega": 0.0,     // Rotation (-1 to 1)
  "speed": 70       // Max speed in % (0-100)
}
```

### GET `/api/sensors`

```json
{
  "sequence": 1234,
  "acc_x": 0.012, "acc_y": -0.005, "acc_z": 1.001,
  "gyr_x": 0.1, "gyr_y": -0.2, "gyr_z": 0.05,  // BMI160 only; always 0.0 with BNO085
  "latitude": 48.8566, "longitude": 2.3522,
  "altitude": 35.2, "speed": 1.5, "speed_kmh": 5.4,
  "heading": 127.5,  // GPS COG (BMI160) or BNO085 absolute yaw (BNO085)
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

`is_alert: true` on the first occurrence of an alert label within the 5-second cooldown window.
Alert labels: `person`, `cat`, `dog`, `horse`, `sheep`, `cow`, `bear`, `zebra`, `giraffe`.

### GET `/api/stream`

Multipart MJPEG stream (`multipart/x-mixed-replace`). Can be used as an `<img>` src directly.

## Mecanum Kinematics

```
FL = vy + vx + omega
FR = vy - vx - omega
RL = vy - vx + omega
RR = vy + vx - omega
```

Speeds are normalized by the maximum value to prevent saturation.

## TODO

- [x] Validate motor wiring and rotation direction
- [x] Test smartphone joystick interface
- [x] Calibrate BMI160 IMU at rest (accelerometer OK; gyroscope dead on clone modules)
- [x] Validate outdoor GPS reception (first fix)
- [x] Write unit tests (`pi5/tests/`)
- [x] Integrate autonomous GPS waypoint navigation
- [x] Add obstacle detection (RPLidar C1)
- [x] Secure web interface (auth + TLS)
- [x] Write BNO085 driver (`pico/lib/bno085.py`) — absolute heading at any speed
- [x] Integrate BNO085 into `pico/main.py` — auto-detected at 0x4A, replaces GPS COG for heading
- [ ] Test autonomous patrol (GPS waypoint following, stop, mode switching)
- [ ] Test LiDAR obstacle avoidance (real-world scenarios: static obstacles, passing distance, avoidance recovery)
- [x] Integrate Raspberry Pi AI Camera (IMX500 / EfficientDet Lite0 — person/animal detection, toast alerts, MJPEG stream)
- [ ] Push notifications + siren on intrusion detection
- [ ] Create a native iOS app mirroring the existing web interface

## License

Non-commercial use only. See [LICENSE](LICENSE) for details.
