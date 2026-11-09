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
from sensors.uart_receiver import UARTReceiver
from motors.motor_controller import MecanumController, MotorPins, MOTORS
from telemetry.web_server import WebServer
from navigation.patrol_manager import PatrolManager
from log_setup import configure_logging

# Hardware defaults — overridden by config/robot_config.yaml when present,
# and (for lidar port / web port) by CLI flags on top of that. Tuning values
# (speeds, PID gains, radii...) default in the classes that use them.
UART_PORT = '/dev/ttyAMA3'
UART_BAUDRATE = 115200
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 460800
LIDAR_MIN_QUALITY = 10
WEB_PORT = 8085
STATUS_PERIOD_S = 5
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATROL_FILE = os.path.join(PROJECT_ROOT, 'config', 'patrol_route.json')
ROBOT_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'config', 'robot_config.yaml')

# robot_config.yaml `navigation:` keys -> AutonomousPilot keyword arguments
_PILOT_OPTIONS = {
    'patrol_speed': 'patrol_speed',
    'obstacle_distance': 'obstacle_distance_mm',
    'loop': 'loop_patrol',
}
_PID_OPTIONS = {'kp': 'pid_kp', 'ki': 'pid_ki', 'kd': 'pid_kd'}


def _pick(section: dict, names: dict) -> dict:
    """Keyword arguments for the config keys present (absent ones keep the class default)."""
    return {kwarg: section[key] for key, kwarg in names.items() if key in section}


def load_robot_config() -> dict:
    """Loads config/robot_config.yaml. Returns {} if missing/invalid (hardcoded defaults apply)."""
    try:
        import yaml
        with open(ROBOT_CONFIG_PATH) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.warning(f"robot_config.yaml non charge ({e}), valeurs par defaut utilisees")
        return {}


def _build_motor_trim(config: dict) -> dict:
    """Per-wheel speed factors from robot_config.yaml (`motors: trim:`).

    Only slowing a wheel down is accepted (0.5-1.0): a factor above 1 would
    be clipped at full speed and the correction would vanish there.
    """
    trim = {}
    for motor, value in (config.get('motors', {}).get('trim') or {}).items():
        if motor in MOTORS and isinstance(value, (int, float)) and 0.5 <= value <= 1.0:
            trim[motor] = float(value)
        else:
            logger.warning(f"motors.trim.{motor}={value!r} ignore "
                           "(attendu: FL/FR/RL/RR entre 0.5 et 1.0)")
    if any(v != 1.0 for v in trim.values()):
        logger.info("Correction moteurs: " + ", ".join(f"{m} x{v:.2f}" for m, v in trim.items()))
    return trim


def _heading_offset(config: dict) -> float:
    """BNO085 mounting correction from robot_config.yaml (`imu: heading_offset_deg:`).

    Real heading = sensor yaw + offset. Only for a sensor mounted rotated on
    the robot: a magnetic error depends on the direction the robot faces and
    is the calibration's job, a constant offset cannot fix it.
    """
    value = config.get('imu', {}).get('heading_offset_deg', 0.0)
    if 'heading_offset_deg' in config.get('navigation', {}):
        logger.warning("navigation.heading_offset_deg n'est plus lu: "
                       "deplacez-le sous imu: heading_offset_deg")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not -180 <= value <= 360:
        logger.warning(f"imu.heading_offset_deg={value!r} ignore (attendu: -180 a 360)")
        return 0.0
    if value:
        logger.info(f"Correction de cap BNO085: {value:+.1f} deg")
    return float(value)


def _build_motor_pins(config: dict):
    """Builds a motor pin config dict from robot_config.yaml, or None to use the driver's defaults."""
    pins_cfg = config.get('motors', {}).get('pins')
    if not pins_cfg:
        return None
    try:
        return {m: MotorPins(ia=pins_cfg[f'{m}_in1'], ib=pins_cfg[f'{m}_in2']) for m in MOTORS}
    except KeyError as e:
        logger.warning(f"Pins moteurs incomplets dans robot_config.yaml ({e}), defauts utilises")
        return None


def _init_component(name: str, factory, optional: bool = False):
    """Builds one subsystem. A failure is logged and the robot runs without it.

    An optional one (lidar, camera) only degrades the robot: its failure is a
    warning. Without motors or UART the robot can't do its job: error.
    """
    logger.info(f"[{name}] Initialisation...")
    try:
        component = factory()
        logger.info(f"[{name}] OK")
        return component
    except Exception as e:
        logger.log(logging.WARNING if optional else logging.ERROR, f"[{name}] Erreur: {e}")
        return None


def _open_uart(port: str, baudrate: int, heading_offset_deg: float = 0.0) -> UARTReceiver:
    receiver = UARTReceiver(port=port, baudrate=baudrate, heading_offset_deg=heading_offset_deg)
    if not receiver.open():
        raise RuntimeError(f"impossible d'ouvrir le port {port}")
    receiver.start_async()
    return receiver


def _start_lidar(port: str, baudrate: int, min_quality: int):
    from lidar.scanner import LidarScanner
    scanner = LidarScanner(port=port, baudrate=baudrate, min_quality=min_quality)
    scanner.start()
    return scanner


def _make_camera(conf_threshold):
    from camera.ai_camera import AiCamera
    return AiCamera() if conf_threshold is None else AiCamera(conf_threshold=conf_threshold)


class RobotApp:
    """Main robot application"""

    def __init__(self, enable_motors=True, enable_sensors=True,
                 enable_lidar=True, lidar_port=LIDAR_PORT,
                 enable_camera=True, camera_conf=None,
                 web_port=WEB_PORT,
                 auth_username=None, auth_password_hash=None,
                 ssl_cert=None, ssl_key=None,
                 config=None):
        self._running = False
        self._shutdown_done = False

        config = config or {}
        uart_cfg = config.get('uart', {})
        lidar_cfg = config.get('lidar', {})
        nav_cfg = config.get('navigation', {})
        min_quality = lidar_cfg.get('min_quality', LIDAR_MIN_QUALITY)
        patrol_file_rel = nav_cfg.get('patrol_file')
        self.patrol_file = (
            os.path.join(PROJECT_ROOT, patrol_file_rel) if patrol_file_rel else PATROL_FILE
        )

        print("\n" + "=" * 50)
        print("  ROBOT MECANUM - Pi 5 Controller")
        print("=" * 50)

        self.motor_controller = _init_component(
            "Motors", lambda: MecanumController(motor_config=_build_motor_pins(config),
                                               trim=_build_motor_trim(config))
        ) if enable_motors else None
        self.sensor_receiver = _init_component(
            "UART", lambda: _open_uart(uart_cfg.get('port', UART_PORT),
                                       uart_cfg.get('baudrate', UART_BAUDRATE),
                                       _heading_offset(config))
        ) if enable_sensors else None
        self.lidar_scanner = _init_component(
            "LiDAR", lambda: _start_lidar(lidar_port,
                                          lidar_cfg.get('baudrate', LIDAR_BAUDRATE),
                                          min_quality),
            optional=True,
        ) if enable_lidar else None
        self.camera = _init_component(
            "Camera", lambda: _make_camera(camera_conf), optional=True,
        ) if enable_camera else None

        self.patrol_manager = PatrolManager(**_pick(nav_cfg, {'waypoint_radius': 'waypoint_radius'}))
        self._load_patrol()

        # The pilot needs both the motors and the GPS; the lidar is optional
        self.pilot = None
        if self.motor_controller and self.sensor_receiver:
            def make_pilot():
                from navigation.pilot import AutonomousPilot
                return AutonomousPilot(
                    motor_controller=self.motor_controller,
                    sensor_receiver=self.sensor_receiver,
                    lidar_scanner=self.lidar_scanner,
                    patrol_manager=self.patrol_manager,
                    min_quality=min_quality,
                    **_pick(nav_cfg, _PILOT_OPTIONS),
                    **_pick(nav_cfg.get('pid', {}), _PID_OPTIONS),
                )
            self.pilot = _init_component("Pilot", make_pilot)

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

    def _load_patrol(self):
        if not os.path.exists(self.patrol_file):
            logger.info("[Patrol] Aucun parcours sauvegarde")
            return
        try:
            if self.patrol_manager.load(self.patrol_file):
                logger.info(f"[Patrol] {self.patrol_manager.count} waypoints charges")
            else:
                logger.error("[Patrol] Erreur chargement waypoints")
        except Exception as e:
            # Malformed JSON must not prevent startup (patrol simply empty)
            logger.error(f"[Patrol] Fichier waypoints invalide ({e}), patrouille vide")

    def run(self):
        """Starts the application; returns after shutdown (SIGINT/SIGTERM exit through here)."""
        self._running = True
        url = self.web_server.start(threaded=True)

        print("\n" + "-" * 50)
        print("Robot pret!")
        print(f"Interface web: {url}")
        print("Ctrl+C pour arreter")
        print("-" * 50 + "\n")

        try:
            if self.camera:
                self.camera.start()   # blocks while the IMX500 loads its model
            while self._running:
                self._print_status()
                time.sleep(STATUS_PERIOD_S)
        except KeyboardInterrupt:
            print("\n\nArret demande...")
        finally:
            self.shutdown()

    def _print_status(self):
        """Displays periodic status"""
        status_parts = []

        if self.sensor_receiver:
            stats = self.sensor_receiver.get_stats()
            data = self.sensor_receiver.get_last_data()
            if data:
                status_parts.append(f"GPS: {data.latitude:.4f},{data.longitude:.4f}")
                status_parts.append(f"Cap: {data.heading:.0f}deg ({data.imu_type})")
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
            self.motor_controller.close()

        if self.lidar_scanner:
            logger.info("  - Arret LiDAR")
            self.lidar_scanner.stop()

        if self.sensor_receiver:
            logger.info("  - Fermeture UART")
            self.sensor_receiver.close()

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
    # Timestamped, severity-coloured logs (see log_setup)
    configure_logging()
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
    parser.add_argument('--camera-conf', type=float, default=None,
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

    # SIGINT/SIGTERM (Ctrl+C, systemctl stop) unwind run(), whose `finally`
    # stops the motors and closes the hardware.
    def signal_handler(sig, frame):
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    app.run()


if __name__ == "__main__":
    main()
