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
from navigation.obstacle_avoider import ObstacleAvoider, angle_diff

logger = logging.getLogger(__name__)


class PilotState(Enum):
    """Autonomous pilot states."""
    IDLE = auto()
    NAVIGATING = auto()
    AVOIDING = auto()
    PATROL_COMPLETE = auto()


# --- Navigation constants ---
_CONTROL_PERIOD_S = 0.2       # s   — control loop at 5 Hz
_HEADING_DEAD_ZONE = 10.0     # deg — ignore small errors (GPS noise)
_GPS_STALE_S = 2.0            # s   — max age of the last UART frame before failsafe stop
_LIDAR_STALE_S = 1.0          # s   — max age of the last lidar scan (~10/s) before failsafe stop
_WAYPOINT_COOLDOWN_S = 5.0    # s   — skip arrival checks right after reaching a waypoint
_SLOWDOWN_DISTANCE_M = 5.0    # m   — decelerate (and damp rotation) under this distance
_MIN_PATROL_SPEED = 20        # %   — below this the motors stall


def _log_transition(was_bad: bool, is_bad: bool, went_bad: str, recovered: str) -> bool:
    """Logs a failsafe flag change once per transition and returns the new value."""
    if is_bad != was_bad:
        if is_bad:
            logger.warning(went_bad)
        else:
            logger.info(recovered)
    return is_bad


class AutonomousPilot:
    """Autonomous pilot combining GPS navigation and lidar avoidance.

    State machine:
      IDLE → NAVIGATING ⇄ AVOIDING   (obstacle in the frontal corridor)
                │
                └──(waypoint reached)──► next waypoint, NAVIGATING
                                    └── last WP ──► PATROL_COMPLETE, or loop

    Heading correction uses PID arc navigation (omega while moving forward).
    Control loop at 5 Hz in a dedicated thread.
    Waypoint detection uses raw GPS; bearing/heading use smoothed average.
    Failsafe: motors stop while the GPS fix, the UART link or the lidar
    (when one is configured) is missing or stale.
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
        heading_offset_deg: float = 45.0,
        min_quality: int = 10,
        pid_kp: float = 1.0,
        pid_ki: float = 0.05,
        pid_kd: float = 0.1,
    ):
        self.motors = motor_controller
        self.sensors = sensor_receiver
        self.lidar = lidar_scanner
        self.patrol = patrol_manager
        self.patrol_speed = patrol_speed
        self.loop_patrol = loop_patrol
        self.heading_offset_deg = heading_offset_deg  # added to BNO085 heading (real = sensor + offset)

        # Sub-modules
        self.detector = ObstacleDetector(min_quality=min_quality)
        self.tracker = ObstacleTracker()
        self.avoider = ObstacleAvoider(obstacle_distance_mm=obstacle_distance_mm)

        # Heading PID: input = heading error, setpoint 0, output = -omega
        self._heading_pid = PID(
            Kp=pid_kp, Ki=pid_ki, Kd=pid_kd,
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

        # Cooldown after reaching a waypoint: prevents cascading through all
        # waypoints if their spacing is below waypoint_radius
        self._waypoint_cooldown_until: float = 0.0

        # Last scan run through the detector, and its result (also read by
        # the web API). A scan is only processed once: the tracker would
        # otherwise count the same frame as several confirmations.
        self._last_scan = None
        self._last_detection = None

        # Liveness heartbeat, refreshed at every control cycle. The web
        # watchdog reads it to detect a dead control thread — in that state it
        # is the only thing standing between a crash and motors stuck on their
        # last command (see control_heartbeat_age).
        self._last_step_time: float = 0.0

        # Failsafes: True while the last UART frame / lidar scan is too old
        # (Pico dead, cable unplugged, lidar USB lost). Without these checks
        # the pilot would keep driving on a frozen position or a frozen scan.
        self._gps_stale = False
        self._lidar_stale = False

        # Live navigation debug values (exposed via get_status for the web UI)
        self._dbg_bearing: float = 0.0        # bearing to active WP (deg)
        self._dbg_avg_heading: float = 0.0    # smoothed heading (deg)
        self._dbg_heading_error: float = 0.0  # bearing - heading (deg)
        self._dbg_omega: float = 0.0          # last omega commanded
        self._dbg_speed: float = 0.0          # gps.speed (m/s) seen by pilot

    @property
    def state(self) -> PilotState:
        with self._lock:
            return self._state

    def _set_state(self, state: PilotState):
        with self._lock:
            self._state = state

    def is_active(self) -> bool:
        """True while the control loop owns the motors.

        Cheap (no GPS maths): the web server asks on every manual command and
        on every watchdog tick. A finished patrol (PATROL_COMPLETE) is not
        active, so manual control comes back without pressing stop first.
        """
        return self._running

    def get_status(self) -> dict:
        """Returns the current state for the web API."""
        target = self.patrol.get_current_target()
        gps = self.sensors.get_last_data() if self.sensors else None

        status = {
            "state": self.state.name,
            "waypoint_index": self.patrol.current_index,
            "waypoint_count": self.patrol.count,
            "loop": self.loop_patrol,
            "gps_stale": self._gps_stale,
            "lidar_stale": self._lidar_stale,
        }

        if target and gps and gps.has_fix:
            status["distance_m"] = round(self.patrol.distance_to_target(gps.latitude, gps.longitude), 1)
            status["bearing"] = round(self.patrol.bearing_to_target(gps.latitude, gps.longitude), 1)
            status["target_lat"] = target.latitude
            status["target_lon"] = target.longitude

        # Live debug telemetry for UI
        status["dbg_bearing"] = round(self._dbg_bearing, 1)
        status["dbg_avg_heading"] = round(self._dbg_avg_heading, 1)
        status["dbg_heading_error"] = round(self._dbg_heading_error, 1)
        status["dbg_omega"] = round(self._dbg_omega, 2)
        status["dbg_speed"] = round(self._dbg_speed, 2)

        return status

    def get_last_detection(self):
        """Returns the last DetectionResult (for the web API)."""
        return self._last_detection

    def control_heartbeat_age(self) -> float:
        """Seconds since the control loop last completed a cycle.

        Returns 0.0 when no loop is expected to run (idle, patrol finished),
        so a watchdog polling this can treat any large value as a fault.
        """
        if not self._running or self._last_step_time == 0.0:
            return 0.0
        return time.time() - self._last_step_time

    def start(self):
        """Starts the autonomous patrol."""
        if self._running:
            return
        if self.patrol.count == 0:
            logger.warning("Aucun waypoint defini, patrouille non lancee")
            return

        self.patrol.reset()
        self._heading_pid.reset()
        self._waypoint_cooldown_until = 0.0
        self._gps_pos_buffer.clear()
        self._gps_hdg_buffer.clear()
        self._last_gps_timestamp = 0.0
        # Prime the heartbeat before the thread exists: otherwise the watchdog
        # would see a stale value during the first cycles.
        self._last_step_time = time.time()
        self._running = True
        self._set_state(PilotState.NAVIGATING)

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        logger.info(f"Patrouille lancee avec {self.patrol.count} waypoints")

    def stop(self):
        """Stops the patrol."""
        self._running = False
        self._last_step_time = 0.0
        if self._thread:
            self._thread.join(timeout=2)
        if self.motors:
            self.motors.stop()
        self._set_state(PilotState.IDLE)
        logger.info("Patrouille arretee")

    def _control_loop(self):
        """Main control loop (5 Hz)."""
        while self._running:
            self._last_step_time = time.time()
            try:
                self._control_step()
            except Exception as e:
                logger.error(f"Erreur pilote: {e}")
            time.sleep(_CONTROL_PERIOD_S)

    def _halt(self):
        if self.motors:
            self.motors.stop()

    def _control_step(self):
        """One step of the control loop."""
        now = time.time()

        # 1. Read sensors and feed GPS smoothing buffers (unique readings only)
        gps = self.sensors.get_last_data() if self.sensors else None
        self._gps_stale = _log_transition(
            self._gps_stale,
            gps is not None and (now - gps.timestamp) > _GPS_STALE_S,
            "Donnees UART/GPS perimees (Pico muet ?) — arret failsafe",
            "Donnees UART/GPS de nouveau fraiches")
        if gps is None or self._gps_stale or not gps.has_fix:
            self._halt()
            return
        if gps.timestamp != self._last_gps_timestamp:
            self._gps_pos_buffer.append((gps.latitude, gps.longitude))
            # GPS COG only: buffer when moving (COG is 0.0 and undefined at rest)
            # BNO085 heading is handled separately below — no buffering needed
            if not gps.imu_calibrated and gps.heading is not None and gps.heading > 0.1:
                self._gps_hdg_buffer.append(gps.heading)
            self._last_gps_timestamp = gps.timestamp
        if not self._gps_pos_buffer:
            return

        # Averaged position — used only for bearing/heading (stability)
        # Raw GPS (gps.latitude/longitude) is used for waypoint detection (no lag)
        avg_lat = sum(p[0] for p in self._gps_pos_buffer) / len(self._gps_pos_buffer)
        avg_lon = sum(p[1] for p in self._gps_pos_buffer) / len(self._gps_pos_buffer)

        # Heading source: BNO085 (calibrated, instant, valid at rest including 0°=North)
        # or GPS COG (smoothed circular mean, only valid while moving)
        if gps.imu_calibrated:
            avg_heading = (gps.heading + self.heading_offset_deg) % 360
        elif self._gps_hdg_buffer:
            sin_sum = sum(math.sin(math.radians(h)) for h in self._gps_hdg_buffer)
            cos_sum = sum(math.cos(math.radians(h)) for h in self._gps_hdg_buffer)
            avg_heading = math.degrees(math.atan2(sin_sum, cos_sum)) % 360
        else:
            avg_heading = 0.0

        # 2. Lidar: a configured lidar that stopped producing scans is treated
        #    like stale GPS — the robot must never drive blind.
        avoidance = None
        if self.lidar:
            scan = self.lidar.get_last_scan()
            self._lidar_stale = _log_transition(
                self._lidar_stale,
                scan is None or (now - scan.timestamp) > _LIDAR_STALE_S,
                "Scans lidar perimes (lidar deconnecte ?) — arret failsafe",
                "Scans lidar de nouveau frais")
            if self._lidar_stale:
                self._halt()
                return
            if scan is not self._last_scan:
                self._last_scan = scan
                self._last_detection = self.tracker.update(self.detector.detect(scan))
            avoidance = self.avoider.evaluate(self._last_detection, 0)

        # 3. Check if waypoint reached — raw GPS avoids averaging lag
        #    (skip during cooldown to prevent cascading through close waypoints)
        if now >= self._waypoint_cooldown_until and \
           self.patrol.is_target_reached(gps.latitude, gps.longitude):
            self._on_waypoint_reached()
            return

        # 4. Bearing and distance to current target (averaged position for stability)
        bearing = self.patrol.bearing_to_target(avg_lat, avg_lon)
        if bearing is None:
            self._halt()
            return
        distance = self.patrol.distance_to_target(avg_lat, avg_lon)
        self._dbg_bearing = bearing
        self._dbg_avg_heading = avg_heading
        self._dbg_speed = gps.speed

        # 5. Hard obstacle: stop immediately
        if avoidance and avoidance.status == "blocked":
            self._set_state(PilotState.AVOIDING)
            self._halt()
            return

        # 6. Heading error (dead zone filters noise)
        # With BNO085: heading always valid. With GPS COG: invalid until robot moves.
        if gps.imu_calibrated or self._gps_hdg_buffer:
            heading_error = angle_diff(bearing, avg_heading)
            if abs(heading_error) < _HEADING_DEAD_ZONE:
                heading_error = 0.0
        else:
            heading_error = 0.0  # no heading yet → go straight
        self._dbg_heading_error = heading_error

        # 7. Soft obstacle: steer toward the free sector; otherwise navigate.
        #    The sector is a robot-frame direction (0..360, clockwise from the
        #    front): it must become a signed error, or a free sector on the
        #    left (e.g. 345°) would be read as a hard right turn.
        if avoidance and avoidance.status == "avoiding":
            self._set_state(PilotState.AVOIDING)
            heading_error = angle_diff(avoidance.suggested_heading, 0)
        else:
            self._set_state(PilotState.NAVIGATING)

        # 8. PID → omega
        omega = -self._heading_pid(heading_error)

        # 9. Speed proportional to distance; damp rotation near the waypoint
        #    so forward motion dominates
        if distance < _SLOWDOWN_DISTANCE_M:
            omega *= distance / _SLOWDOWN_DISTANCE_M
        speed = max(_MIN_PATROL_SPEED,
                    int(self.patrol_speed * min(1.0, distance / _SLOWDOWN_DISTANCE_M)))

        self._dbg_omega = omega
        # 10. Command motors: forward + heading correction
        if self.motors:
            self.motors.move(vx=0, vy=1, omega=omega, speed=speed)

    def _on_waypoint_reached(self):
        """Called when a waypoint is reached."""
        logger.info(f"Waypoint {self.patrol.current_index} atteint!")
        # Stop motors and reset PID state to avoid windup / derivative kick on the new leg
        self._halt()
        self._heading_pid.reset()
        # Clear stale heading buffer: heading was computed while moving toward
        # the old waypoint; it's no longer valid for the next leg.
        self._gps_hdg_buffer.clear()
        self._waypoint_cooldown_until = time.time() + _WAYPOINT_COOLDOWN_S

        if self.patrol.advance():
            self._set_state(PilotState.NAVIGATING)
        elif self.loop_patrol:
            logger.info("Patrouille terminee, recommence en boucle")
            self.patrol.reset()
            self._set_state(PilotState.NAVIGATING)
        else:
            logger.info("Patrouille terminee")
            self._set_state(PilotState.PATROL_COMPLETE)
            self._running = False
