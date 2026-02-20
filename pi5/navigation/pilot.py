"""Autonomous pilot for GPS patrol with lidar obstacle avoidance."""

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
    """Autonomous pilot states."""
    IDLE = auto()
    NAVIGATING = auto()
    AVOIDING = auto()
    WAYPOINT_REACHED = auto()
    PATROL_COMPLETE = auto()


class AutonomousPilot:
    """Autonomous pilot combining GPS navigation and lidar avoidance.

    State machine:
      IDLE → NAVIGATING → AVOIDING → NAVIGATING → ... → WAYPOINT_REACHED
        → NAVIGATING → ... → PATROL_COMPLETE → (loop or IDLE)

    Control loop at ~5 Hz in a dedicated thread.
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

        # Sub-modules
        self.detector = ObstacleDetector(min_quality=10)
        self.tracker = ObstacleTracker()
        self.avoider = ObstacleAvoider(obstacle_distance_mm=obstacle_distance_mm)

        # PID for heading control (output = omega for motors)
        # Setpoint will be the bearing to the waypoint
        self._heading_pid = PID(
            Kp=1.0, Ki=0.05, Kd=0.1,
            setpoint=0,
            output_limits=(-1.0, 1.0),
        )

        # State
        self._state = PilotState.IDLE
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

        # Last lidar detection (for the web API)
        self._last_detection = None

    @property
    def state(self) -> PilotState:
        with self._lock:
            return self._state

    def get_status(self) -> dict:
        """Returns the current state for the web API."""
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
        """Returns the last DetectionResult (for the web API)."""
        return self._last_detection

    def start(self):
        """Starts the autonomous patrol."""
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
        """Stops the patrol."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self.motors:
            self.motors.stop()
        with self._lock:
            self._state = PilotState.IDLE
        logger.info("Patrouille arretee")

    def _control_loop(self):
        """Main control loop (~5 Hz)."""
        while self._running:
            try:
                self._control_step()
            except Exception as e:
                logger.error(f"Erreur pilote: {e}")
            time.sleep(0.2)  # 5 Hz

    def _control_step(self):
        """One step of the control loop."""
        # 1. Read sensors
        gps = self.sensors.get_last_data() if self.sensors else None
        if gps is None or not gps.has_fix:
            if self.motors:
                self.motors.stop()
            return

        # 2. Lidar detection
        detection = None
        if self.lidar:
            self.lidar.process_incoming()
            scan = self.lidar.get_last_scan()
            if scan:
                raw_detection = self.detector.detect(scan)
                detection = self.tracker.update(raw_detection)
                self._last_detection = detection

        # 3. Check if waypoint reached
        if self.patrol.is_target_reached(gps.latitude, gps.longitude):
            self._on_waypoint_reached()
            return

        # 4. Compute desired heading
        bearing = self.patrol.bearing_to_target(gps.latitude, gps.longitude)
        if bearing is None:
            if self.motors:
                self.motors.stop()
            return

        distance = self.patrol.distance_to_target(gps.latitude, gps.longitude)

        # 5. Convert GPS bearing to robot-relative heading
        # bearing = absolute heading to target (0=North)
        # gps.heading = absolute heading of robot (0=North)
        # heading_error = difference → how much to turn
        # NOTE: GPS heading (NEO-6M) is only reliable above ~0.5 m/s.
        # Below that, we go straight (omega=0) to gain speed.
        MIN_SPEED_FOR_HEADING = 0.5  # m/s
        if gps.speed < MIN_SPEED_FOR_HEADING:
            heading_error = 0.0
        else:
            heading_error = self._angle_diff(bearing, gps.heading)

        # 6. Obstacle avoidance
        # Lidar angle is in robot frame (0=forward)
        # We want to check if the "adjusted straight ahead" path is clear
        # heading_error > 0 = turn right, < 0 = turn left
        # In lidar frame: 0 = straight ahead
        desired_lidar_heading = 0  # We always want to go "forward"

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
                # Adjust heading_error to avoid the obstacle
                # avoidance.suggested_heading is in lidar frame
                heading_error = avoidance.suggested_heading
            else:
                with self._lock:
                    self._state = PilotState.NAVIGATING
        else:
            with self._lock:
                self._state = PilotState.NAVIGATING

        # 7. PID for omega
        self._heading_pid.setpoint = 0
        omega = -self._heading_pid(heading_error)

        # 8. Speed proportional to distance (deceleration on approach)
        if distance is not None:
            speed_factor = min(1.0, distance / 5.0)  # Decelerate under 5m
        else:
            speed_factor = 1.0
        speed = int(self.patrol_speed * speed_factor)
        speed = max(20, speed)  # Minimum speed

        # 9. Command the motors
        # vy = move forward, omega = turn
        if self.motors:
            self.motors.move(vx=0, vy=1, omega=omega, speed=speed)

    def _on_waypoint_reached(self):
        """Called when a waypoint is reached."""
        logger.info(f"Waypoint {self.patrol.current_index} atteint!")

        if not self.patrol.advance():
            # Patrol complete
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
        """Signed angle difference, result in [-180, 180]."""
        diff = (a - b + 180) % 360 - 180
        return diff
