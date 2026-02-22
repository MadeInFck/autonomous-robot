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
import argparse

# Relative path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensors.uart_receiver import UARTReceiver, SensorData
from motors.motor_controller import MecanumController
from telemetry.web_server import WebServer
from navigation.patrol_manager import PatrolManager

# Default configuration
UART_PORT = '/dev/ttyAMA3'
UART_BAUDRATE = 115200
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUDRATE = 460800
WEB_PORT = 8085
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATROL_FILE = os.path.join(PROJECT_ROOT, 'config', 'patrol_route.json')


class RobotApp:
    """Main robot application"""

    def __init__(self, enable_motors=True, enable_sensors=True,
                 enable_lidar=True, lidar_port=LIDAR_PORT,
                 enable_camera=True, camera_conf=0.35,
                 web_port=WEB_PORT,
                 auth_username=None, auth_password_hash=None,
                 ssl_cert=None, ssl_key=None):
        self.motor_controller = None
        self.sensor_receiver = None
        self.lidar_scanner = None
        self.patrol_manager = None
        self.pilot = None
        self.camera = None
        self.web_server = None
        self._running = False

        print("\n" + "=" * 50)
        print("  ROBOT MECANUM - Pi 5 Controller")
        print("=" * 50)

        # Initialize motors
        if enable_motors:
            print("\n[Motors] Initialisation...")
            try:
                self.motor_controller = MecanumController()
                print("[Motors] OK")
            except Exception as e:
                print(f"[Motors] Erreur: {e}")

        # Initialize the UART receiver
        if enable_sensors:
            print(f"\n[UART] Initialisation {UART_PORT}...")
            try:
                self.sensor_receiver = UARTReceiver(
                    port=UART_PORT,
                    baudrate=UART_BAUDRATE
                )
                if self.sensor_receiver.open():
                    self.sensor_receiver.start_async(callback=self._on_sensor_data)
                    print("[UART] OK")
                else:
                    print("[UART] Impossible d'ouvrir le port")
                    self.sensor_receiver = None
            except Exception as e:
                print(f"[UART] Erreur: {e}")
                self.sensor_receiver = None

        # Initialize the LiDAR
        if enable_lidar:
            print(f"\n[LiDAR] Initialisation {lidar_port}...")
            try:
                from lidar.scanner import LidarScanner
                self.lidar_scanner = LidarScanner(
                    port=lidar_port,
                    baudrate=LIDAR_BAUDRATE,
                    min_quality=10,
                )
                self.lidar_scanner.start()
                print("[LiDAR] OK")
            except Exception as e:
                print(f"[LiDAR] Erreur: {e}")
                self.lidar_scanner = None

        # Initialize the AI camera
        if enable_camera:
            print("\n[Camera] Initialisation...")
            try:
                from camera.ai_camera import AiCamera
                self.camera = AiCamera(conf_threshold=camera_conf)
            except Exception as e:
                print(f"[Camera] Erreur: {e}")

        # Initialize the patrol manager
        print("\n[Patrol] Initialisation...")
        self.patrol_manager = PatrolManager(waypoint_radius=3.0)
        if os.path.exists(PATROL_FILE):
            if self.patrol_manager.load(PATROL_FILE):
                print(f"[Patrol] {self.patrol_manager.count} waypoints charges")
            else:
                print("[Patrol] Erreur chargement waypoints")
        else:
            print("[Patrol] Aucun parcours sauvegarde")

        # Initialize the autonomous pilot
        if self.motor_controller and self.sensor_receiver:
            try:
                from navigation.pilot import AutonomousPilot
                self.pilot = AutonomousPilot(
                    motor_controller=self.motor_controller,
                    sensor_receiver=self.sensor_receiver,
                    lidar_scanner=self.lidar_scanner,
                    patrol_manager=self.patrol_manager,
                    patrol_speed=40,
                    obstacle_distance_mm=800,
                    loop_patrol=True,
                )
                print("[Pilot] OK")
            except Exception as e:
                print(f"[Pilot] Erreur: {e}")

        # Initialize the web server
        print(f"\n[Web] Initialisation serveur sur port {web_port}...")
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
            patrol_file=PATROL_FILE,
        )
        if auth_username:
            print("[Web] Authentification activee")
        if ssl_cert:
            print("[Web] TLS actif")

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
            print(f"[Status] {' | '.join(status_parts)}")

    def shutdown(self):
        """Gracefully shuts down the application"""
        self._running = False
        print("\nArret des composants...")

        if self.camera:
            print("  - Arret caméra")
            self.camera.stop()

        if self.pilot:
            print("  - Arret pilote")
            self.pilot.stop()

        if self.motor_controller:
            print("  - Arret moteurs")
            self.motor_controller.stop()
            self.motor_controller.close()

        if self.lidar_scanner:
            print("  - Arret LiDAR")
            self.lidar_scanner.stop()

        if self.sensor_receiver:
            print("  - Fermeture UART")
            self.sensor_receiver.close()

        if self.web_server:
            print("  - Arret serveur web")
            self.web_server.stop()

        print("\nProgramme termine")


def load_web_config():
    """Loads auth + TLS config from secrets.yaml"""
    secrets_path = os.path.join(PROJECT_ROOT, 'config', 'secrets.yaml')
    try:
        import yaml
        with open(secrets_path) as f:
            cfg = yaml.safe_load(f)
        auth = cfg.get('auth', {})
        tls = cfg.get('tls', {})
        # Resolve cert/key paths relative to project root
        ssl_cert = tls.get('cert')
        ssl_key = tls.get('key')
        if ssl_cert and not os.path.isabs(ssl_cert):
            ssl_cert = os.path.join(PROJECT_ROOT, ssl_cert)
        if ssl_key and not os.path.isabs(ssl_key):
            ssl_key = os.path.join(PROJECT_ROOT, ssl_key)
        # Only use TLS if both files exist
        if ssl_cert and ssl_key and os.path.exists(ssl_cert) and os.path.exists(ssl_key):
            pass
        else:
            ssl_cert, ssl_key = None, None
        return auth.get('username'), auth.get('password_hash'), ssl_cert, ssl_key
    except Exception:
        return None, None, None, None


def main():
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
    parser.add_argument('--lidar-port', type=str, default=LIDAR_PORT,
                        help=f'Port LiDAR (defaut: {LIDAR_PORT})')
    parser.add_argument('--port', type=int, default=WEB_PORT,
                        help=f'Port serveur web (defaut: {WEB_PORT})')
    parser.add_argument('--no-auth', action='store_true',
                        help='Desactiver l\'authentification')

    args = parser.parse_args()

    auth_user, auth_hash, ssl_cert, ssl_key = load_web_config()
    if args.no_auth:
        auth_user, auth_hash = None, None

    app = RobotApp(
        enable_motors=not args.no_motors,
        enable_sensors=not args.no_sensors,
        enable_lidar=not args.no_lidar,
        enable_camera=not args.no_camera,
        camera_conf=args.camera_conf,
        lidar_port=args.lidar_port,
        web_port=args.port,
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
