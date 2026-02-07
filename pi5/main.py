#!/usr/bin/env python3
"""
Application principale Raspberry Pi 5
Robot Autonome Mecanum - Contrôle via smartphone

Déploiement: ~/AutonomRobot/pi5/
"""

import os
import sys
import time
import signal
import argparse

# Chemin relatif pour les imports locaux
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensors.uart_receiver import UARTReceiver, SensorData
from motors.motor_controller import MecanumController
from telemetry.web_server import WebServer

# Configuration
UART_PORT = '/dev/ttyAMA3'
UART_BAUDRATE = 115200
WEB_PORT = 8085


class RobotApp:
    """Application principale du robot"""

    def __init__(self, enable_motors=True, enable_sensors=True, web_port=WEB_PORT):
        self.motor_controller = None
        self.sensor_receiver = None
        self.web_server = None
        self._running = False

        print("\n" + "=" * 50)
        print("  ROBOT MECANUM - Pi 5 Controller")
        print("=" * 50)

        # Initialiser les moteurs
        if enable_motors:
            print("\n[Motors] Initialisation...")
            try:
                self.motor_controller = MecanumController()
                print("[Motors] OK")
            except Exception as e:
                print(f"[Motors] Erreur: {e}")

        # Initialiser le récepteur UART
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

        # Initialiser le serveur web
        print(f"\n[Web] Initialisation serveur sur port {web_port}...")
        self.web_server = WebServer(
            motor_controller=self.motor_controller,
            sensor_receiver=self.sensor_receiver,
            port=web_port
        )

    def _on_sensor_data(self, data: SensorData):
        """Callback pour données capteurs reçues"""
        pass

    def run(self):
        """Lance l'application"""
        self._running = True

        # Démarrer le serveur web
        if self.web_server:
            url = self.web_server.start(threaded=True)
        else:
            url = "non disponible"

        print("\n" + "-" * 50)
        print("Robot prêt!")
        print(f"Interface web: {url}")
        print("Ctrl+C pour arrêter")
        print("-" * 50 + "\n")

        try:
            while self._running:
                self._print_status()
                time.sleep(5)
        except KeyboardInterrupt:
            print("\n\nArrêt demandé...")

        self.shutdown()

    def _print_status(self):
        """Affiche le status périodique"""
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

        if status_parts:
            print(f"[Status] {' | '.join(status_parts)}")

    def shutdown(self):
        """Arrête proprement l'application"""
        self._running = False
        print("\nArrêt des composants...")

        if self.motor_controller:
            print("  - Arrêt moteurs")
            self.motor_controller.stop()
            self.motor_controller.close()

        if self.sensor_receiver:
            print("  - Fermeture UART")
            self.sensor_receiver.close()

        if self.web_server:
            print("  - Arrêt serveur web")
            self.web_server.stop()

        print("\nProgramme terminé")


def main():
    parser = argparse.ArgumentParser(description='Robot Mecanum Controller')
    parser.add_argument('--no-motors', action='store_true',
                        help='Désactiver le contrôle moteurs')
    parser.add_argument('--no-sensors', action='store_true',
                        help='Désactiver la réception UART')
    parser.add_argument('--port', type=int, default=WEB_PORT,
                        help=f'Port serveur web (défaut: {WEB_PORT})')

    args = parser.parse_args()

    app = RobotApp(
        enable_motors=not args.no_motors,
        enable_sensors=not args.no_sensors,
        web_port=args.port
    )

    def signal_handler(sig, frame):
        app.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    app.run()


if __name__ == "__main__":
    main()
