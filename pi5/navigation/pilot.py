"""Autonomous pilot for GPS patrol with lidar obstacle avoidance."""

import math
import time
import threading
import logging
from collections import deque
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


# --- Navigation constants ---
_HEADING_DEAD_ZONE = 10.0  # deg  — ignore small errors (GPS noise)


class AutonomousPilot:
    """Autonomous pilot combining GPS navigation and lidar avoidance.

    State machine:
      IDLE → NAVIGATING ──(obstacle blocked)──► AVOIDING
                │              (avoiding)──► heading corrected, resume
                │
                └──(waypoint reached)──► WAYPOINT_REACHED
                        └── advance ──► NAVIGATING
                        └── last WP ──► PATROL_COMPLETE or loop

    Heading correction uses PID arc navigation (omega while moving forward).
    Control loop at ~5 Hz in a dedicated thread.
    Waypoint detection uses raw GPS; bearing/heading use smoothed average.
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

        # GPS smoothing buffers (unique readings only, filtered by timestamp)
        # Position: 3 readings at 1 Hz → ~0.75 m lag at 0.5 m/s, √3 jitter reduction
        # Heading:  5 readings for smoother arc correction
        self._gps_pos_buffer: deque = deque(maxlen=3)   # (lat, lon)
        self._gps_hdg_buffer: deque = deque(maxlen=5)   # heading degrees
        self._last_gps_timestamp: float = 0.0

        # Cooldown after reaching a waypoint: skip is_target_reached for N seconds
        # Prevents cascading through all waypoints if spacing < waypoint_radius
        self._waypoint_cooldown_until: float = 0.0
        self._WAYPOINT_COOLDOWN = 5.0  # seconds

        # Last lidar detection (for the web API)
        self._last_detection = None

        # Live navigation debug values (exposed via get_status for the web UI)
        self._dbg_bearing: float = 0.0        # bearing to active WP (deg)
        self._dbg_avg_heading: float = 0.0    # smoothed GPS COG (deg)
        self._dbg_heading_error: float = 0.0  # bearing - heading (deg)
        self._dbg_omega: float = 0.0          # last omega commanded
        self._dbg_speed: float = 0.0          # gps.speed (m/s) seen by pilot

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

        # Live debug telemetry for UI
        status["dbg_bearing"]       = round(self._dbg_bearing, 1)
        status["dbg_avg_heading"]   = round(self._dbg_avg_heading, 1)
        status["dbg_heading_error"] = round(self._dbg_heading_error, 1)
        status["dbg_omega"]         = round(self._dbg_omega, 2)
        status["dbg_speed"]         = round(self._dbg_speed, 2)

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
        self._waypoint_cooldown_until = 0.0
        self._gps_pos_buffer.clear()
        self._gps_hdg_buffer.clear()
        self._last_gps_timestamp = 0.0
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
        # 1. Read sensors and feed GPS smoothing buffers (unique readings only)
        gps = self.sensors.get_last_data() if self.sensors else None
        if gps is None or not gps.has_fix:
            if self.motors:
                self.motors.stop()
            return
        if gps.timestamp != self._last_gps_timestamp:
            self._gps_pos_buffer.append((gps.latitude, gps.longitude))
            if gps.heading is not None:
                self._gps_hdg_buffer.append(gps.heading)
            self._last_gps_timestamp = gps.timestamp

        if not self._gps_pos_buffer:
            return

        # Averaged position — used only for bearing/heading (stability)
        # Raw GPS (gps.latitude/longitude) is used for waypoint detection (no lag)
        avg_lat = sum(p[0] for p in self._gps_pos_buffer) / len(self._gps_pos_buffer)
        avg_lon = sum(p[1] for p in self._gps_pos_buffer) / len(self._gps_pos_buffer)

        # Smoothed heading (circular mean handles 0/360 wraparound)
        if self._gps_hdg_buffer:
            sin_sum = sum(math.sin(math.radians(h)) for h in self._gps_hdg_buffer)
            cos_sum = sum(math.cos(math.radians(h)) for h in self._gps_hdg_buffer)
            avg_heading = math.degrees(math.atan2(sin_sum, cos_sum)) % 360
        else:
            avg_heading = gps.heading or 0.0

        # 2. Lidar detection — single evaluation, result reused below
        detection = None
        avoidance = None
        if self.lidar:
            self.lidar.process_incoming()
            scan = self.lidar.get_last_scan()
            if scan:
                raw_detection = self.detector.detect(scan)
                detection = self.tracker.update(raw_detection)
                self._last_detection = detection
                avoidance = self.avoider.evaluate(detection, 0)

        # 3. Check if waypoint reached — raw GPS avoids averaging lag
        #    (skip during cooldown to prevent cascading through close waypoints)
        if time.time() >= self._waypoint_cooldown_until:
            if self.patrol.is_target_reached(gps.latitude, gps.longitude):
                self._on_waypoint_reached()
                return

        # 4. Bearing and distance to current target (averaged position for stability)
        bearing = self.patrol.bearing_to_target(avg_lat, avg_lon)
        if bearing is None:
            if self.motors:
                self.motors.stop()
            return
        distance = self.patrol.distance_to_target(avg_lat, avg_lon)
        self._dbg_bearing = bearing
        self._dbg_avg_heading = avg_heading
        self._dbg_speed = gps.speed

        # 5. Hard obstacle: stop immediately
        if avoidance and avoidance.status == "blocked":
            with self._lock:
                self._state = PilotState.AVOIDING
            if self.motors:
                self.motors.stop()
            return

        # 6. Heading error (dead zone filters GPS noise)
        heading_error = self._angle_diff(bearing, avg_heading)
        if abs(heading_error) < _HEADING_DEAD_ZONE:
            heading_error = 0.0
        self._dbg_heading_error = heading_error

        # 7. Soft obstacle: redirect heading; otherwise navigate normally
        if avoidance and avoidance.status == "avoiding":
            with self._lock:
                self._state = PilotState.AVOIDING
            heading_error = avoidance.suggested_heading
        else:
            with self._lock:
                self._state = PilotState.NAVIGATING

        # 8. PID → omega
        self._heading_pid.setpoint = 0
        omega = -self._heading_pid(heading_error)

        # 9. Speed proportional to distance (decelerate under 5 m)
        speed_factor = min(1.0, distance / 5.0) if distance is not None else 1.0
        speed = max(20, int(self.patrol_speed * speed_factor))

        # Reduce omega near waypoint so forward motion dominates over rotation
        if distance is not None and distance < 5.0:
            omega *= distance / 5.0

        self._dbg_omega = omega
        # 10. Command motors: forward + heading correction
        if self.motors:
            self.motors.move(vx=0, vy=1, omega=omega, speed=speed)

    def _on_waypoint_reached(self):
        """Called when a waypoint is reached."""
        logger.info(f"Waypoint {self.patrol.current_index} atteint!")
        # Stop motors and reset PID integral to avoid windup on the new leg
        if self.motors:
            self.motors.stop()
        self._heading_pid._integral = 0
        # Clear stale heading buffer: heading was computed while moving toward
        # the old waypoint; it's no longer valid for the next leg.
        self._gps_hdg_buffer.clear()
        # Arm cooldown: next waypoint won't be checked for N seconds
        self._waypoint_cooldown_until = time.time() + self._WAYPOINT_COOLDOWN

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
