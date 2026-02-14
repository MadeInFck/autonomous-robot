"""Pilote autonome pour patrouille GPS avec evitement d'obstacles lidar."""

import math
import time
import threading
import logging
from enum import Enum, auto
from typing import Optional

from simple_pid import PID

from motors.motor_controller import MecanumController
from sensors.uart_receiver import UARTReceiver
from lidar.scanner import LidarScanner
from lidar.detection.obstacles import ObstacleDetector
from lidar.detection.tracker import ObstacleTracker
from navigation.patrol_manager import PatrolManager
from navigation.obstacle_avoider import ObstacleAvoider

logger = logging.getLogger(__name__)


class PilotState(Enum):
    """Etats du pilote autonome."""
    IDLE = auto()
    NAVIGATING = auto()
    AVOIDING = auto()
    WAYPOINT_REACHED = auto()
    PATROL_COMPLETE = auto()


class AutonomousPilot:
    """Pilote autonome combinant navigation GPS et evitement lidar.

    Machine a etats :
      IDLE → NAVIGATING → AVOIDING → NAVIGATING → ... → WAYPOINT_REACHED
        → NAVIGATING → ... → PATROL_COMPLETE → (boucle ou IDLE)

    Boucle de controle a ~5 Hz dans un thread dedie.
    """

    def __init__(
        self,
        motor_controller: MecanumController,
        sensor_receiver: UARTReceiver,
        lidar_scanner: Optional[LidarScanner],
        patrol_manager: PatrolManager,
        patrol_speed: int = 40,
        obstacle_distance_mm: float = 800,
        loop_patrol: bool = True,
    ):
        self.motors = motor_controller
        self.sensors = sensor_receiver
        self.lidar = lidar_scanner
        self.patrol = patrol_manager
        self.patrol_speed = patrol_speed
        self.loop_patrol = loop_patrol

        # Sous-modules
        self.detector = ObstacleDetector(min_quality=10)
        self.tracker = ObstacleTracker()
        self.avoider = ObstacleAvoider(obstacle_distance_mm=obstacle_distance_mm)

        # PID pour le controle de cap (output = omega pour moteurs)
        # Setpoint sera le bearing vers le waypoint
        self._heading_pid = PID(
            Kp=1.0, Ki=0.05, Kd=0.1,
            setpoint=0,
            output_limits=(-1.0, 1.0),
        )

        # Etat
        self._state = PilotState.IDLE
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        # Derniere detection lidar (pour l'API web)
        self._last_detection = None

    @property
    def state(self) -> PilotState:
        with self._lock:
            return self._state

    def get_status(self) -> dict:
        """Retourne l'etat courant pour l'API web."""
        with self._lock:
            state = self._state

        target = self.patrol.get_current_target()
        gps = self.sensors.get_last_data() if self.sensors else None

        status = {
            "state": state.name,
            "waypoint_index": self.patrol.current_index,
            "waypoint_count": self.patrol.count,
            "loop": self.loop_patrol,
        }

        if target and gps and gps.has_fix:
            dist = self.patrol.distance_to_target(gps.latitude, gps.longitude)
            bearing = self.patrol.bearing_to_target(gps.latitude, gps.longitude)
            status["distance_m"] = round(dist, 1) if dist else None
            status["bearing"] = round(bearing, 1) if bearing else None
            status["target_lat"] = target.latitude
            status["target_lon"] = target.longitude

        return status

    def get_last_detection(self):
        """Retourne le dernier DetectionResult (pour l'API web)."""
        return self._last_detection

    def start(self):
        """Lance la patrouille autonome."""
        if self._running:
            return
        if self.patrol.count == 0:
            logger.warning("Aucun waypoint defini, patrouille non lancee")
            return

        self.patrol.reset()
        self._running = True
        with self._lock:
            self._state = PilotState.NAVIGATING

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        logger.info(f"Patrouille lancee avec {self.patrol.count} waypoints")

    def stop(self):
        """Arrete la patrouille."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self.motors:
            self.motors.stop()
        with self._lock:
            self._state = PilotState.IDLE
        logger.info("Patrouille arretee")

    def _control_loop(self):
        """Boucle de controle principale (~5 Hz)."""
        while self._running:
            try:
                self._control_step()
            except Exception as e:
                logger.error(f"Erreur pilote: {e}")
            time.sleep(0.2)  # 5 Hz

    def _control_step(self):
        """Un pas de la boucle de controle."""
        # 1. Lire les capteurs
        gps = self.sensors.get_last_data() if self.sensors else None
        if gps is None or not gps.has_fix:
            if self.motors:
                self.motors.stop()
            return

        # 2. Detection lidar
        detection = None
        if self.lidar:
            self.lidar.process_incoming()
            scan = self.lidar.get_last_scan()
            if scan:
                raw_detection = self.detector.detect(scan)
                detection = self.tracker.update(raw_detection)
                self._last_detection = detection

        # 3. Verifier si waypoint atteint
        if self.patrol.is_target_reached(gps.latitude, gps.longitude):
            self._on_waypoint_reached()
            return

        # 4. Calculer le cap souhaite
        bearing = self.patrol.bearing_to_target(gps.latitude, gps.longitude)
        if bearing is None:
            if self.motors:
                self.motors.stop()
            return

        distance = self.patrol.distance_to_target(gps.latitude, gps.longitude)

        # 5. Convertir bearing GPS en cap relatif robot
        # bearing = cap absolu vers la cible (0=Nord)
        # gps.heading = cap absolu du robot (0=Nord)
        # heading_error = difference → combien tourner
        heading_error = self._angle_diff(bearing, gps.heading)

        # 6. Evitement d'obstacles
        # L'angle du lidar est dans le repere robot (0=avant)
        # On veut verifier si le chemin "droit devant ajuste" est libre
        # heading_error > 0 = tourner a droite, < 0 = tourner a gauche
        # Dans le repere lidar : 0 = droit devant
        desired_lidar_heading = 0  # On veut toujours aller "devant"

        if detection:
            avoidance = self.avoider.evaluate(detection, desired_lidar_heading)
            if avoidance.status == "blocked":
                with self._lock:
                    self._state = PilotState.AVOIDING
                if self.motors:
                    self.motors.stop()
                return
            elif avoidance.status == "avoiding":
                with self._lock:
                    self._state = PilotState.AVOIDING
                # Ajuster heading_error pour eviter l'obstacle
                # avoidance.suggested_heading est dans le repere lidar
                heading_error = avoidance.suggested_heading
            else:
                with self._lock:
                    self._state = PilotState.NAVIGATING
        else:
            with self._lock:
                self._state = PilotState.NAVIGATING

        # 7. PID pour omega
        self._heading_pid.setpoint = 0
        omega = -self._heading_pid(heading_error)

        # 8. Vitesse proportionnelle a la distance (deceleration a l'approche)
        if distance is not None:
            speed_factor = min(1.0, distance / 5.0)  # Decelere sous 5m
        else:
            speed_factor = 1.0
        speed = int(self.patrol_speed * speed_factor)
        speed = max(20, speed)  # Vitesse minimale

        # 9. Commander les moteurs
        # vy = avancer, omega = tourner
        if self.motors:
            self.motors.move(vx=0, vy=1, omega=omega, speed=speed)

    def _on_waypoint_reached(self):
        """Appele quand un waypoint est atteint."""
        logger.info(f"Waypoint {self.patrol.current_index} atteint!")

        if not self.patrol.advance():
            # Patrouille terminee
            if self.loop_patrol:
                logger.info("Patrouille terminee, recommence en boucle")
                self.patrol.reset()
                with self._lock:
                    self._state = PilotState.NAVIGATING
            else:
                logger.info("Patrouille terminee")
                if self.motors:
                    self.motors.stop()
                with self._lock:
                    self._state = PilotState.PATROL_COMPLETE
                self._running = False
        else:
            with self._lock:
                self._state = PilotState.NAVIGATING

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Difference d'angle signee, resultat dans [-180, 180]."""
        diff = (a - b + 180) % 360 - 180
        return diff
