#!/usr/bin/env python3
"""
Main application for Raspberry Pi 5
Autonomous Mecanum Robot - Smartphone control + autonomous patrol

Deployment: ~/AutonomRobot/pi5/
"""

import os
import sys
import time
import signal
import logging
import argparse

logger = logging.getLogger('robot')

# Relative path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensors.uart_receiver import UARTReceiver, SensorData
from motors.motor_controller import MecanumController, MotorPins
from telemetry.web_server import WebServer
from navigation.patrol_manager import PatrolManager

# Default configuration — overridden by config/robot_config.yaml when present,
# and (for lidar port / web port) by CLI flags on top of that.
UART_PORT = '/dev/ttyAMA3'
UART_BAUDRATE = 115200
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 460800
LIDAR_MIN_QUALITY = 10
WEB_PORT = 8085
WAYPOINT_RADIUS = 3.0
PATROL_SPEED = 40
OBSTACLE_DISTANCE_MM = 800
LOOP_PATROL = True
HEADING_OFFSET_DEG = 45.0
PID_KP, PID_KI, PID_KD = 1.0, 0.05, 0.1
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATROL_FILE = os.path.join(PROJECT_ROOT, 'config', 'patrol_route.json')
ROBOT_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'config', 'robot_config.yaml')


def load_robot_config() -> dict:
    """Loads config/robot_config.yaml. Returns {} if missing/invalid (hardcoded defaults apply)."""
    try:
        import yaml
        with open(ROBOT_CONFIG_PATH) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.warning(f"robot_config.yaml non charge ({e}), valeurs par defaut utilisees")
        return {}


def _build_motor_pins(config: dict):
    """Builds a motor pin config dict from robot_config.yaml, or None to use the driver's defaults."""
    pins_cfg = config.get('motors', {}).get('pins')
    if not pins_cfg:
        return None
    try:
        return {
            'FL': MotorPins(ia=pins_cfg['FL_in1'], ib=pins_cfg['FL_in2']),
            'FR': MotorPins(ia=pins_cfg['FR_in1'], ib=pins_cfg['FR_in2']),
            'RL': MotorPins(ia=pins_cfg['RL_in1'], ib=pins_cfg['RL_in2']),
            'RR': MotorPins(ia=pins_cfg['RR_in1'], ib=pins_cfg['RR_in2']),
        }
    except KeyError as e:
        logger.warning(f"Pins moteurs incomplets dans robot_config.yaml ({e}), defauts utilises")
        return None


class RobotApp:
    """Main robot application"""

    def __init__(self, enable_motors=True, enable_sensors=True,
                 enable_lidar=True, lidar_port=LIDAR_PORT,
                 enable_camera=True, camera_conf=0.35,
                 web_port=WEB_PORT,
                 auth_username=None, auth_password_hash=None,
                 ssl_cert=None, ssl_key=None,
                 config=None):
        self.motor_controller = None
        self.sensor_receiver = None
        self.lidar_scanner = None
        self.patrol_manager = None
        self.pilot = None
        self.camera = None
        self.web_server = None
        self._running = False
        self._shutdown_done = False

        config = config or {}
        uart_cfg = config.get('uart', {})
        lidar_cfg = config.get('lidar', {})
        nav_cfg = config.get('navigation', {})
        pid_cfg = nav_cfg.get('pid', {})
        min_quality = lidar_cfg.get('min_quality', LIDAR_MIN_QUALITY)
        patrol_file_rel = nav_cfg.get('patrol_file')
        self.patrol_file = (
            os.path.join(PROJECT_ROOT, patrol_file_rel) if patrol_file_rel else PATROL_FILE
        )

        print("\n" + "=" * 50)
        print("  ROBOT MECANUM - Pi 5 Controller")
        print("=" * 50)

        # Initialize motors
        if enable_motors:
            logger.info("[Motors] Initialisation...")
            try:
                self.motor_controller = MecanumController(
                    motor_config=_build_motor_pins(config)
                )
                logger.info("[Motors] OK")
            except Exception as e:
                logger.error(f"[Motors] Erreur: {e}")

        # Initialize the UART receiver
        uart_port = uart_cfg.get('port', UART_PORT)
        uart_baudrate = uart_cfg.get('baudrate', UART_BAUDRATE)
        if enable_sensors:
            logger.info(f"[UART] Initialisation {uart_port}...")
            try:
                self.sensor_receiver = UARTReceiver(
                    port=uart_port,
                    baudrate=uart_baudrate
                )
                if self.sensor_receiver.open():
                    self.sensor_receiver.start_async(callback=self._on_sensor_data)
                    logger.info("[UART] OK")
                else:
                    logger.error("[UART] Impossible d'ouvrir le port")
                    self.sensor_receiver = None
            except Exception as e:
                logger.error(f"[UART] Erreur: {e}")
                self.sensor_receiver = None

        # Initialize the LiDAR
        if enable_lidar:
            logger.info(f"[LiDAR] Initialisation {lidar_port}...")
            try:
                from lidar.scanner import LidarScanner
                self.lidar_scanner = LidarScanner(
                    port=lidar_port,
                    baudrate=lidar_cfg.get('baudrate', LIDAR_BAUDRATE),
                    min_quality=min_quality,
                )
                self.lidar_scanner.start()
                logger.info("[LiDAR] OK")
            except Exception as e:
                logger.error(f"[LiDAR] Erreur: {e}")
                self.lidar_scanner = None

        # Initialize the AI camera
        if enable_camera:
            logger.info("[Camera] Initialisation...")
            try:
                from camera.ai_camera import AiCamera
                self.camera = AiCamera(conf_threshold=camera_conf)
            except Exception as e:
                logger.error(f"[Camera] Erreur: {e}")

        # Initialize the patrol manager
        logger.info("[Patrol] Initialisation...")
        self.patrol_manager = PatrolManager(
            waypoint_radius=nav_cfg.get('waypoint_radius', WAYPOINT_RADIUS)
        )
        if os.path.exists(self.patrol_file):
            try:
                if self.patrol_manager.load(self.patrol_file):
                    logger.info(f"[Patrol] {self.patrol_manager.count} waypoints charges")
                else:
                    logger.error("[Patrol] Erreur chargement waypoints")
            except Exception as e:
                # Malformed JSON must not prevent startup (patrol simply empty)
                logger.error(f"[Patrol] Fichier waypoints invalide ({e}), patrouille vide")
        else:
            logger.info("[Patrol] Aucun parcours sauvegarde")

        # Initialize the autonomous pilot
        if self.motor_controller and self.sensor_receiver:
            try:
                from navigation.pilot import AutonomousPilot
                self.pilot = AutonomousPilot(
                    motor_controller=self.motor_controller,
                    sensor_receiver=self.sensor_receiver,
                    lidar_scanner=self.lidar_scanner,
                    patrol_manager=self.patrol_manager,
                    patrol_speed=nav_cfg.get('patrol_speed', PATROL_SPEED),
                    obstacle_distance_mm=nav_cfg.get('obstacle_distance', OBSTACLE_DISTANCE_MM),
                    loop_patrol=nav_cfg.get('loop', LOOP_PATROL),
                    heading_offset_deg=nav_cfg.get('heading_offset_deg', HEADING_OFFSET_DEG),
                    min_quality=min_quality,
                    pid_kp=pid_cfg.get('kp', PID_KP),
                    pid_ki=pid_cfg.get('ki', PID_KI),
                    pid_kd=pid_cfg.get('kd', PID_KD),
                )
                logger.info("[Pilot] OK")
            except Exception as e:
                logger.error(f"[Pilot] Erreur: {e}")

        # Initialize the web server
        logger.info(f"[Web] Initialisation serveur sur port {web_port}...")
        self.web_server = WebServer(
            motor_controller=self.motor_controller,
            sensor_receiver=self.sensor_receiver,
            lidar_scanner=self.lidar_scanner,
            patrol_manager=self.patrol_manager,
            pilot=self.pilot,
            camera=self.camera,
            port=web_port,
            auth_username=auth_username,
            auth_password_hash=auth_password_hash,
            ssl_cert=ssl_cert,
            ssl_key=ssl_key,
            patrol_file=self.patrol_file,
        )
        if auth_username:
            logger.info("[Web] Authentification activee")
        if ssl_cert:
            logger.info("[Web] TLS actif")

    def _on_sensor_data(self, data: SensorData):
        """Callback for received sensor data"""
        pass

    def run(self):
        """Starts the application"""
        self._running = True

        # Start the web server
        if self.web_server:
            url = self.web_server.start(threaded=True)
        else:
            url = "non disponible"

        print("\n" + "-" * 50)
        print("Robot pret!")
        print(f"Interface web: {url}")
        print("Ctrl+C pour arreter")
        print("-" * 50 + "\n")

        if self.camera:
            self.camera.start()

        try:
            while self._running:
                # Process incoming lidar points
                if self.lidar_scanner:
                    self.lidar_scanner.process_incoming()
                self._print_status()
                time.sleep(5)
        except KeyboardInterrupt:
            print("\n\nArret demande...")

        self.shutdown()

    def _print_status(self):
        """Displays periodic status"""
        status_parts = []

        if self.sensor_receiver:
            stats = self.sensor_receiver.get_stats()
            data = self.sensor_receiver.get_last_data()
            if data:
                status_parts.append(f"GPS: {data.latitude:.4f},{data.longitude:.4f}")
                status_parts.append(f"IMU: {data.acc_z:.2f}g")
            status_parts.append(f"Rx: {stats['packets_received']}")

        if self.motor_controller:
            motors = self.motor_controller.get_status()
            active = sum(1 for m in motors.values() if m['speed'] > 0)
            status_parts.append(f"Motors: {active}/4")

        if self.lidar_scanner:
            scan = self.lidar_scanner.get_last_scan()
            if scan:
                status_parts.append(f"Lidar: {scan.num_valid}pts")

        if self.pilot:
            status_parts.append(f"Mode: {self.pilot.state.name}")

        if status_parts:
            logger.info(f"[Status] {' | '.join(status_parts)}")

    def shutdown(self):
        """Gracefully shuts down the application (idempotent)"""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._running = False
        logger.info("Arret des composants...")

        if self.camera:
            logger.info("  - Arret caméra")
            self.camera.stop()

        if self.pilot:
            logger.info("  - Arret pilote")
            self.pilot.stop()

        if self.motor_controller:
            logger.info("  - Arret moteurs")
            self.motor_controller.stop()
            self.motor_controller.close()

        if self.lidar_scanner:
            logger.info("  - Arret LiDAR")
            self.lidar_scanner.stop()

        if self.sensor_receiver:
            logger.info("  - Fermeture UART")
            self.sensor_receiver.close()

        if self.web_server:
            logger.info("  - Arret serveur web")
            self.web_server.stop()

        logger.info("Programme termine")


def load_web_config(secrets_path=None):
    """Loads auth + TLS config from config/secrets.yaml.

    Fail-closed: raises RuntimeError if the file is missing, unreadable or
    incomplete — starting silently WITHOUT auth would expose motor control
    to the whole network. Use --no-auth to opt out explicitly.
    """
    if secrets_path is None:
        secrets_path = os.path.join(PROJECT_ROOT, 'config', 'secrets.yaml')
    try:
        import yaml
        with open(secrets_path) as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        raise RuntimeError(
            f"{secrets_path} introuvable. Copiez config/secrets.yaml.example "
            "vers config/secrets.yaml, ou lancez avec --no-auth (interface NON protegee)."
        )
    except Exception as e:
        raise RuntimeError(
            f"{secrets_path} illisible ({e}). Corrigez le fichier, "
            "ou lancez avec --no-auth (interface NON protegee)."
        )

    auth = cfg.get('auth') or {}
    username = auth.get('username')
    password_hash = auth.get('password_hash')
    if not username or not password_hash:
        raise RuntimeError(
            f"{secrets_path}: section auth incomplete (username/password_hash requis), "
            "ou lancez avec --no-auth (interface NON protegee)."
        )

    tls = cfg.get('tls') or {}
    # Resolve cert/key paths relative to project root
    ssl_cert = tls.get('cert')
    ssl_key = tls.get('key')
    if ssl_cert and not os.path.isabs(ssl_cert):
        ssl_cert = os.path.join(PROJECT_ROOT, ssl_cert)
    if ssl_key and not os.path.isabs(ssl_key):
        ssl_key = os.path.join(PROJECT_ROOT, ssl_key)
    # Only use TLS if both files exist
    if not (ssl_cert and ssl_key and os.path.exists(ssl_cert) and os.path.exists(ssl_key)):
        ssl_cert, ssl_key = None, None
    return username, password_hash, ssl_cert, ssl_key


def main():
    # Timestamped logs: essential to diagnose a patrol after the fact
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)-7s %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )
    config = load_robot_config()
    default_lidar_port = config.get('lidar', {}).get('port', LIDAR_PORT)
    default_web_port = config.get('telemetry', {}).get('web_server_port', WEB_PORT)

    parser = argparse.ArgumentParser(description='Robot Mecanum Controller')
    parser.add_argument('--no-motors', action='store_true',
                        help='Desactiver le controle moteurs')
    parser.add_argument('--no-sensors', action='store_true',
                        help='Desactiver la reception UART')
    parser.add_argument('--no-lidar', action='store_true',
                        help='Desactiver le LiDAR')
    parser.add_argument('--no-camera', action='store_true',
                        help='Désactiver la caméra IA')
    parser.add_argument('--camera-conf', type=float, default=0.50,
                        help='Seuil de confiance caméra (défaut: 0.50)')
    parser.add_argument('--lidar-port', type=str, default=default_lidar_port,
                        help=f'Port LiDAR (defaut: {default_lidar_port})')
    parser.add_argument('--port', type=int, default=default_web_port,
                        help=f'Port serveur web (defaut: {default_web_port})')
    parser.add_argument('--no-auth', action='store_true',
                        help='Desactiver l\'authentification')

    args = parser.parse_args()

    if args.no_auth:
        auth_user, auth_hash, ssl_cert, ssl_key = None, None, None, None
        try:
            # --no-auth disables authentication but keeps TLS if configured
            _, _, ssl_cert, ssl_key = load_web_config()
        except RuntimeError:
            pass
        logger.warning("[Web] ATTENTION: authentification DESACTIVEE (--no-auth), "
                       "toute personne sur le reseau peut controler le robot")
    else:
        try:
            auth_user, auth_hash, ssl_cert, ssl_key = load_web_config()
        except RuntimeError as e:
            logger.error(f"[Config] {e}")
            sys.exit(1)
        if not ssl_cert:
            logger.warning("[Web] ATTENTION: TLS inactif (certificats absents), le "
                           "mot de passe Basic Auth circule EN CLAIR sur le reseau. "
                           "Generez un certificat: voir config/secrets.yaml (section tls).")

    app = RobotApp(
        enable_motors=not args.no_motors,
        enable_sensors=not args.no_sensors,
        enable_lidar=not args.no_lidar,
        enable_camera=not args.no_camera,
        camera_conf=args.camera_conf,
        lidar_port=args.lidar_port,
        web_port=args.port,
        config=config,
        auth_username=auth_user,
        auth_password_hash=auth_hash,
        ssl_cert=ssl_cert,
        ssl_key=ssl_key,
    )

    def signal_handler(sig, frame):
        app.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    app.run()


if __name__ == "__main__":
    main()
