"""
Tests for Mecanum kinematics (motor speed computation)
Tests pure logic without GPIO (no hardware needed)
"""

import sys
import os
import time
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motors'))

# Patch gpiod before import to avoid error on machines without GPIO
import unittest.mock
sys.modules['gpiod'] = unittest.mock.MagicMock()
sys.modules['gpiod.line'] = unittest.mock.MagicMock()

import motor_controller
from motor_controller import MecanumController, MOTORS


@pytest.fixture
def controller():
    """Create a controller on a mocked GPIO chip"""
    ctrl = MecanumController()
    yield ctrl
    ctrl.close()


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


# ===== Software PWM (timing, idle cost, shutdown) =====

class _PinRecorder:
    """Stands in for the gpiod line request: timestamps every pin write."""

    def __init__(self):
        self.events = []

    def set_value(self, pin, value):
        self.events.append((time.perf_counter(), pin, value))

    def release(self):
        pass

    def writes(self, pin):
        return [(t, v) for t, p, v in self.events if p == pin]


@pytest.fixture
def pwm(monkeypatch):
    recorder = _PinRecorder()
    monkeypatch.setattr(motor_controller.gpiod, 'request_lines',
                        lambda *args, **kwargs: recorder)

    class SlowPWM(MecanumController):
        PWM_HZ = 10  # 100 ms cycles: duty measurable despite scheduler jitter

    ctrl = SlowPWM()
    yield ctrl, recorder
    ctrl.close()


ACTIVE = motor_controller.Value.ACTIVE
INACTIVE = motor_controller.Value.INACTIVE


def _on_durations(writes):
    """Durations of the ACTIVE pulses in a pin's write history."""
    pulses, rose = [], None
    for t, value in writes:
        if value is ACTIVE:
            rose = t
        elif rose is not None:
            pulses.append(t - rose)
            rose = None
    return pulses


class TestSoftwarePWM:
    def test_duty_cycle_matches_speed(self, pwm):
        ctrl, rec = pwm
        ctrl.forward(30)               # 30 % of a 100 ms cycle = 30 ms pulses
        time.sleep(0.65)
        ctrl.stop()
        pulses = _on_durations(rec.writes(ctrl.config['FL'].ia))
        assert len(pulses) >= 4, 'PWM never cycled'
        mean = sum(pulses) / len(pulses)
        assert 0.022 < mean < 0.040, f'pulse {mean * 1000:.1f} ms, expected ~30 ms'

    def test_reverse_drives_ib_only(self, pwm):
        ctrl, rec = pwm
        ctrl.backward(50)
        time.sleep(0.35)
        ctrl.stop()
        fl = ctrl.config['FL']
        assert any(v is ACTIVE for _, v in rec.writes(fl.ib))
        assert not any(v is ACTIVE for _, v in rec.writes(fl.ia))

    def test_full_speed_stays_on_without_toggling(self, pwm):
        ctrl, rec = pwm
        ctrl.forward(100)
        time.sleep(0.35)
        writes = rec.writes(ctrl.config['FL'].ia)
        assert [v for _, v in writes].count(ACTIVE) == 1, '100 % must not chop'

    def test_stopped_wheel_never_driven(self, pwm):
        ctrl, rec = pwm
        ctrl.move(vx=1, vy=1, omega=0, speed=60)   # FR = RL = 0
        time.sleep(0.35)
        for motor in ('FR', 'RL'):
            pins = ctrl.config[motor]
            assert not any(v is ACTIVE for _, v in rec.writes(pins.ia) + rec.writes(pins.ib))

    def test_idle_costs_no_pin_writes(self, pwm):
        """Robot at rest: the PWM thread must block, not spin."""
        ctrl, rec = pwm
        ctrl.forward(50)
        time.sleep(0.25)
        ctrl.stop()
        time.sleep(0.25)               # let the cycle in progress finish
        before = len(rec.events)
        time.sleep(0.4)
        assert len(rec.events) == before

    def test_close_leaves_every_pin_low(self, pwm):
        ctrl, rec = pwm
        ctrl.forward(80)
        time.sleep(0.15)
        ctrl.close()
        assert not ctrl._pwm_thread.is_alive()
        last = {}
        for _, pin, value in rec.events:
            last[pin] = value
        for motor in MOTORS:
            pins = ctrl.config[motor]
            assert last[pins.ia] is INACTIVE and last[pins.ib] is INACTIVE
