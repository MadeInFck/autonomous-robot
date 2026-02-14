"""Tests du pilote autonome (transitions d'etats)."""

import sys
import os
import time
import pytest
from unittest.mock import MagicMock, PropertyMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Mock gpiod et rplidarc1 avant import
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

from navigation.pilot import AutonomousPilot, PilotState
from navigation.patrol_manager import PatrolManager
from sensors.uart_receiver import SensorData


def make_sensor_data(lat=48.8566, lon=2.3522, heading=0, has_fix=True):
    return SensorData(
        latitude=lat, longitude=lon, heading=heading,
        has_fix=has_fix, satellites=8,
    )


@pytest.fixture
def mock_components():
    motors = MagicMock()
    sensors = MagicMock()
    lidar = None  # Pas de lidar pour simplifier

    pm = PatrolManager(waypoint_radius=3.0)
    pm.record_waypoint(48.8570, 2.3522, "WP0")  # ~44m au nord
    pm.record_waypoint(48.8575, 2.3522, "WP1")  # ~100m au nord

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
        pm = PatrolManager()  # Vide
        pilot = AutonomousPilot(motors, sensors, lidar, pm)
        pilot.start()
        assert pilot.state == PilotState.IDLE


class TestPilotStatus:
    def test_get_status(self, mock_components):
        motors, sensors, lidar, pm = mock_components
        sensors.get_last_data.return_value = make_sensor_data()
        pilot = AutonomousPilot(motors, sensors, lidar, pm)

        status = pilot.get_status()
        assert status["state"] == "IDLE"
        assert status["waypoint_count"] == 2
        assert status["waypoint_index"] == 0


class TestAngleDiff:
    def test_no_diff(self):
        assert AutonomousPilot._angle_diff(90, 90) == 0

    def test_positive(self):
        assert abs(AutonomousPilot._angle_diff(100, 90) - 10) < 0.01

    def test_wrap(self):
        assert abs(AutonomousPilot._angle_diff(10, 350) - 20) < 0.01
