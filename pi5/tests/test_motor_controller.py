"""
Tests de la cinematique Mecanum (calcul des vitesses moteur)
Teste la logique pure sans GPIO (pas besoin de hardware)
"""

import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motors'))

# Patch gpiod avant import pour eviter l'erreur sur machine sans GPIO
import unittest.mock
sys.modules['gpiod'] = unittest.mock.MagicMock()
sys.modules['gpiod.line'] = unittest.mock.MagicMock()

from motor_controller import MecanumController


@pytest.fixture
def controller():
    """Cree un controleur en mode simulation (pas de GPIO)"""
    ctrl = MecanumController()
    return ctrl


class TestMecanumKinematics:
    """
    Cinematique Mecanum:
    FL = vy + vx + omega
    FR = vy - vx - omega
    RL = vy - vx + omega
    RR = vy + vx - omega
    """

    def test_forward(self, controller):
        """Avant: toutes les roues en avant, meme vitesse"""
        controller.move(vx=0, vy=1, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100
            assert status[motor]['forward'] is True

    def test_backward(self, controller):
        """Arriere: toutes les roues en arriere"""
        controller.move(vx=0, vy=-1, omega=0, speed=100)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 100
            assert status[motor]['forward'] is False

    def test_strafe_right(self, controller):
        """Deplacement lateral droit: FL et RR avant, FR et RL arriere"""
        controller.move(vx=1, vy=0, omega=0, speed=100)
        status = controller.get_status()
        # FL = 0 + 1 + 0 = 1 (avant)
        assert status['FL']['forward'] is True
        # FR = 0 - 1 - 0 = -1 (arriere)
        assert status['FR']['forward'] is False
        # RL = 0 - 1 + 0 = -1 (arriere)
        assert status['RL']['forward'] is False
        # RR = 0 + 1 - 0 = 1 (avant)
        assert status['RR']['forward'] is True

    def test_strafe_left(self, controller):
        """Deplacement lateral gauche: inverse du droit"""
        controller.move(vx=-1, vy=0, omega=0, speed=100)
        status = controller.get_status()
        assert status['FL']['forward'] is False
        assert status['FR']['forward'] is True
        assert status['RL']['forward'] is True
        assert status['RR']['forward'] is False

    def test_rotate_right(self, controller):
        """Rotation droite: FL et RL avant, FR et RR arriere"""
        controller.move(vx=0, vy=0, omega=1, speed=100)
        status = controller.get_status()
        # FL = 0 + 0 + 1 = 1 (avant)
        assert status['FL']['forward'] is True
        # FR = 0 - 0 - 1 = -1 (arriere)
        assert status['FR']['forward'] is False
        # RL = 0 - 0 + 1 = 1 (avant)
        assert status['RL']['forward'] is True
        # RR = 0 + 0 - 1 = -1 (arriere)
        assert status['RR']['forward'] is False

    def test_rotate_left(self, controller):
        """Rotation gauche: inverse"""
        controller.move(vx=0, vy=0, omega=-1, speed=100)
        status = controller.get_status()
        assert status['FL']['forward'] is False
        assert status['FR']['forward'] is True
        assert status['RL']['forward'] is False
        assert status['RR']['forward'] is True

    def test_diagonal_front_right(self, controller):
        """Diagonale avant-droite: FL et RR rapides, FR et RL lents"""
        controller.move(vx=1, vy=1, omega=0, speed=100)
        status = controller.get_status()
        # FL = 1 + 1 = 2 → normalise a 1 → 100%
        # FR = 1 - 1 = 0 → 0%
        assert status['FL']['speed'] == 100
        assert status['FR']['speed'] == 0
        assert status['RL']['speed'] == 0
        assert status['RR']['speed'] == 100

    def test_stop(self, controller):
        """Stop: toutes les vitesses a 0"""
        controller.move(vx=1, vy=1, omega=0.5, speed=80)
        controller.stop()
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 0


class TestNormalization:
    def test_values_never_exceed_speed(self, controller):
        """Les vitesses moteur ne doivent jamais depasser le speed max"""
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
        """Le parametre speed doit reduire proportionnellement"""
        controller.move(vx=0, vy=1, omega=0, speed=50)
        status = controller.get_status()
        for motor in ['FL', 'FR', 'RL', 'RR']:
            assert status[motor]['speed'] == 50


class TestInputClamping:
    def test_clamp_vx(self, controller):
        """vx > 1 doit etre clamp a 1"""
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
        """Entree zero = arret"""
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
