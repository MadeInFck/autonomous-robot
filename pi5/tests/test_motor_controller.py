"""
Tests for Mecanum kinematics (motor speed computation)
Tests pure logic without GPIO (no hardware needed)
"""

import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motors'))

# Patch gpiod before import to avoid error on machines without GPIO
import unittest.mock
sys.modules['gpiod'] = unittest.mock.MagicMock()
sys.modules['gpiod.line'] = unittest.mock.MagicMock()

from motor_controller import MecanumController


@pytest.fixture
def controller():
    """Create a controller in simulation mode (no GPIO)"""
    ctrl = MecanumController()
    return ctrl


class TestMecanumKinematics:
    """
    Mecanum kinematics:
    FL = vy + vx + omega
    FR = vy - vx - omega
    RL = vy - vx + omega
    RR = vy + vx - omega
    """

    def test_forward(self, controller):
        """Forward: all wheels forward, same speed"""
        controller.move(vx=0, vy=1, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100
            assert status[motor]['forward'] is True

    def test_backward(self, controller):
        """Backward: all wheels in reverse"""
        controller.move(vx=0, vy=-1, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100
            assert status[motor]['forward'] is False

    def test_strafe_right(self, controller):
        """Right lateral movement: FL and RR forward, FR and RL backward"""
        controller.move(vx=1, vy=0, omega=0, speed=100)
        status = controller.get_status()
        # FL = 0 + 1 + 0 = 1 (forward)
        assert status['FL']['forward'] is True
        # FR = 0 - 1 - 0 = -1 (backward)
        assert status['FR']['forward'] is False
        # RL = 0 - 1 + 0 = -1 (backward)
        assert status['RL']['forward'] is False
        # RR = 0 + 1 - 0 = 1 (forward)
        assert status['RR']['forward'] is True

    def test_strafe_left(self, controller):
        """Left lateral movement: opposite of right"""
        controller.move(vx=-1, vy=0, omega=0, speed=100)
        status = controller.get_status()
        assert status['FL']['forward'] is False
        assert status['FR']['forward'] is True
        assert status['RL']['forward'] is True
        assert status['RR']['forward'] is False

    def test_rotate_right(self, controller):
        """Right rotation: FL and RL forward, FR and RR backward"""
        controller.move(vx=0, vy=0, omega=1, speed=100)
        status = controller.get_status()
        # FL = 0 + 0 + 1 = 1 (forward)
        assert status['FL']['forward'] is True
        # FR = 0 - 0 - 1 = -1 (backward)
        assert status['FR']['forward'] is False
        # RL = 0 - 0 + 1 = 1 (forward)
        assert status['RL']['forward'] is True
        # RR = 0 + 0 - 1 = -1 (backward)
        assert status['RR']['forward'] is False

    def test_rotate_left(self, controller):
        """Left rotation: opposite"""
        controller.move(vx=0, vy=0, omega=-1, speed=100)
        status = controller.get_status()
        assert status['FL']['forward'] is False
        assert status['FR']['forward'] is True
        assert status['RL']['forward'] is False
        assert status['RR']['forward'] is True

    def test_diagonal_front_right(self, controller):
        """Front-right diagonal: FL and RR fast, FR and RL slow"""
        controller.move(vx=1, vy=1, omega=0, speed=100)
        status = controller.get_status()
        # FL = 1 + 1 = 2 -> normalized to 1 -> 100%
        # FR = 1 - 1 = 0 -> 0%
        assert status['FL']['speed'] == 100
        assert status['FR']['speed'] == 0
        assert status['RL']['speed'] == 0
        assert status['RR']['speed'] == 100

    def test_stop(self, controller):
        """Stop: all speeds at 0"""
        controller.move(vx=1, vy=1, omega=0.5, speed=80)
        controller.stop()
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 0


class TestNormalization:
    def test_values_never_exceed_speed(self, controller):
        """Motor speeds must never exceed the max speed"""
        test_cases = [
            (1, 1, 1),
            (-1, -1, -1),
            (1, 1, -1),
            (0.5, 0.8, 0.3),
        ]
        for vx, vy, omega in test_cases:
            controller.move(vx, vy, omega, speed=100)
            status = controller.get_status()
            for motor in ['FL', 'FR', 'RL', 'RR']:
                assert status[motor]['speed'] <= 100, \
                    f"Motor {motor} > 100% avec vx={vx} vy={vy} omega={omega}"

    def test_speed_parameter_scales(self, controller):
        """The speed parameter must scale proportionally"""
        controller.move(vx=0, vy=1, omega=0, speed=50)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 50


class TestInputClamping:
    def test_clamp_vx(self, controller):
        """vx > 1 must be clamped to 1"""
        controller.move(vx=5, vy=0, omega=0, speed=100)
        status = controller.get_status()
        # FL = 0 + 1 + 0 = 1 (clamped)
        assert status['FL']['speed'] == 100

    def test_clamp_vy(self, controller):
        controller.move(vx=0, vy=5, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100

    def test_clamp_negative(self, controller):
        controller.move(vx=0, vy=-5, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100
            assert status[motor]['forward'] is False

    def test_zero_input(self, controller):
        """Zero input = stop"""
        controller.move(vx=0, vy=0, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 0


class TestHelperMethods:
    def test_forward_helper(self, controller):
        controller.forward(60)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 60
            assert status[motor]['forward'] is True

    def test_backward_helper(self, controller):
        controller.backward(40)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 40
            assert status[motor]['forward'] is False

    def test_strafe_left_helper(self, controller):
        controller.strafe_left(50)
        status = controller.get_status()
        assert status['FL']['forward'] is False
        assert status['FR']['forward'] is True

    def test_strafe_right_helper(self, controller):
        controller.strafe_right(50)
        status = controller.get_status()
        assert status['FL']['forward'] is True
        assert status['FR']['forward'] is False
