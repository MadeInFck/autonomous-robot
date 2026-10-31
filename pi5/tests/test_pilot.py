"""Tests for the autonomous pilot (state transitions)."""

import sys
import os
import time
import pytest
from unittest.mock import MagicMock, PropertyMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Mock gpiod and rplidarc1 before import
import unittest.mock
sys.modules['gpiod'] = unittest.mock.MagicMock()
sys.modules['gpiod.line'] = unittest.mock.MagicMock()
sys.modules['rplidarc1'] = unittest.mock.MagicMock()
sys.modules['simple_pid'] = unittest.mock.MagicMock()

# Mock PID
mock_pid_class = MagicMock()
mock_pid_instance = MagicMock()
mock_pid_instance.return_value = 0.0
mock_pid_class.return_value = mock_pid_instance
sys.modules['simple_pid'].PID = mock_pid_class

import numpy as np

from navigation.pilot import AutonomousPilot, PilotState
from navigation.patrol_manager import PatrolManager
from sensors.uart_receiver import SensorData
from lidar.data import Scan
from lidar.detection.types import BoundingBox, DetectionResult, Obstacle, ObstacleType


def make_sensor_data(lat=48.8566, lon=2.3522, heading=0, has_fix=True,
                     timestamp=None):
    return SensorData(
        latitude=lat, longitude=lon, heading=heading,
        has_fix=has_fix, satellites=8,
        timestamp=time.time() if timestamp is None else timestamp,
    )


@pytest.fixture
def mock_components():
    motors = MagicMock()
    sensors = MagicMock()
    lidar = None  # No lidar to simplify

    pm = PatrolManager(waypoint_radius=3.0)
    pm.record_waypoint(48.8570, 2.3522, "WP0")  # ~44m north
    pm.record_waypoint(48.8575, 2.3522, "WP1")  # ~100m north

    return motors, sensors, lidar, pm


class TestPilotStates:
    def test_initial_state_idle(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        assert pilot.state == PilotState.IDLE

    def test_start_sets_navigating(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        time.sleep(0.1)
        assert pilot.state == PilotState.NAVIGATING
        pilot.stop()

    def test_stop_sets_idle(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        time.sleep(0.1)
        pilot.stop()
        assert pilot.state == PilotState.IDLE

    def test_no_fix_stops_motors(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data(has_fix=False)
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        time.sleep(0.3)
        motors.stop.assert_called()
        pilot.stop()

    def test_no_waypoints_no_start(self, mock_components):
        motors, sensors, lidar, _ = mock_components
        pm = PatrolManager()  # Empty
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        assert pilot.state == PilotState.IDLE

    def test_stale_gps_stops_motors(self, mock_components):
        """UART dead: last frame frozen — failsafe must stop, never navigate."""
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data(
            timestamp=time.time() - 10)
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        time.sleep(0.3)
        motors.stop.assert_called()
        motors.move.assert_not_called()
        assert pilot.get_status()["gps_stale"] is True
        pilot.stop()

    def test_fresh_gps_not_stale(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        time.sleep(0.3)
        assert pilot.get_status()["gps_stale"] is False
        pilot.stop()


class TestPilotStatus:
    def test_get_status(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)

        status = pilot.get_status()
        assert status["state"] == "IDLE"
        assert status["waypoint_count"] == 2
        assert status["waypoint_index"] == 0


class TestControlHeartbeat:
    """The web watchdog uses this to detect a dead control thread."""

    def test_idle_pilot_reports_no_age(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        assert pilot.control_heartbeat_age() == 0.0

    def test_running_pilot_stays_fresh(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        try:
            time.sleep(0.3)  # a few cycles at 5 Hz
            assert pilot.control_heartbeat_age() < 1.0
        finally:
            pilot.stop()

    def test_frozen_loop_ages(self, mock_components):
        """Simulates a control thread that stopped cycling."""
        motors, sensors, lidar, pm = mock_components
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot._running = True
        pilot._last_step_time = time.time() - 30
        assert pilot.control_heartbeat_age() > 29

    def test_stop_clears_heartbeat(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        pilot.stop()
        assert pilot.control_heartbeat_age() == 0.0


def make_obstacle(angle_deg, distance_mm):
    a = np.radians(angle_deg)
    c = np.array([distance_mm * np.sin(a), distance_mm * np.cos(a)])
    return Obstacle(
        id=0, type=ObstacleType.OBJECT, points_xy=c[None, :], centroid=c,
        nearest_distance_mm=distance_mm, nearest_point=c.copy(),
        bbox=BoundingBox(c[0] - 50, c[0] + 50, c[1] - 50, c[1] + 50),
        line_segment=None, num_points=5, angular_span_deg=10.0,
    )


@pytest.fixture
def lidar_pilot(mock_components):
    """Pilot with a lidar whose detections the test controls."""
    motors, sensors, _, pm = mock_components
    sensors.get_last_data.return_value = make_sensor_data()
    lidar = MagicMock()
    lidar.get_last_scan.return_value = Scan(timestamp=time.time())
    pilot = AutonomousPilot(motors, sensors, lidar, pm)
    pilot.tracker.update = lambda result: result     # no confirmation delay
    pilot.detector.detect = MagicMock(return_value=DetectionResult())
    return pilot, motors, lidar


class TestLidarIntegration:
    def test_free_sector_on_the_left_steers_left(self, lidar_pilot):
        """Front and front-right blocked: the free sector is at 345deg (robot
        frame). It used to reach the PID as +345, i.e. a hard RIGHT turn into
        the obstacle."""
        pilot, motors, _ = lidar_pilot
        pilot.detector.detect.return_value = DetectionResult(obstacles=[
            make_obstacle(15, 500), make_obstacle(45, 500),
        ])
        mock_pid_instance.reset_mock()
        pilot._control_step()
        assert pilot.state == PilotState.AVOIDING
        heading_error = mock_pid_instance.call_args[0][0]
        assert heading_error == -15

    def test_blocked_stops(self, lidar_pilot):
        pilot, motors, _ = lidar_pilot
        pilot.detector.detect.return_value = DetectionResult(obstacles=[
            make_obstacle(a, 400) for a in range(15, 360, 30)
        ])
        pilot._control_step()
        motors.stop.assert_called()
        motors.move.assert_not_called()

    def test_frozen_lidar_stops(self, lidar_pilot):
        """Lidar unplugged mid-patrol: never drive on the last scan."""
        pilot, motors, lidar = lidar_pilot
        lidar.get_last_scan.return_value = Scan(timestamp=time.time() - 5)
        pilot._control_step()
        motors.stop.assert_called()
        motors.move.assert_not_called()
        assert pilot.get_status()["lidar_stale"] is True

    def test_no_scan_yet_stops(self, lidar_pilot):
        pilot, motors, lidar = lidar_pilot
        lidar.get_last_scan.return_value = None
        pilot._control_step()
        motors.move.assert_not_called()

    def test_clear_path_drives(self, lidar_pilot):
        pilot, motors, _ = lidar_pilot
        pilot._control_step()
        motors.move.assert_called_once()
        assert pilot.state == PilotState.NAVIGATING
        assert pilot.get_status()["lidar_stale"] is False

    def test_each_scan_detected_once(self, lidar_pilot):
        """The tracker would count a re-processed scan as a new confirmation."""
        pilot, _, lidar = lidar_pilot
        for _ in range(3):
            pilot._control_step()
        assert pilot.detector.detect.call_count == 1
        lidar.get_last_scan.return_value = Scan(timestamp=time.time())
        pilot._control_step()
        assert pilot.detector.detect.call_count == 2


class TestActivity:
    def test_active_only_while_running(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        assert pilot.is_active() is False
        pilot.start()
        try:
            assert pilot.is_active() is True
        finally:
            pilot.stop()
        assert pilot.is_active() is False

    def test_patrol_complete_releases_motors(self, mock_components):
        """Last waypoint reached (no loop): manual control must come back."""
        motors, sensors, lidar, _ = mock_components
        pm = PatrolManager(waypoint_radius=3.0)
        pm.record_waypoint(48.8566, 2.3522)          # where the robot stands
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm, loop_patrol=False)
        pilot.start()
        time.sleep(0.3)
        assert pilot.state == PilotState.PATROL_COMPLETE
        assert pilot.is_active() is False
        pilot.stop()

    def test_bearing_north_is_reported(self, mock_components):
        """bearing 0.0 (due north) used to be dropped as falsy."""
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        assert pilot.get_status()["bearing"] == 0.0
