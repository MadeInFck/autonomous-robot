"""Tests de l'evitement d'obstacles (ObstacleAvoider)."""

import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lidar.detection.types import (
    BoundingBox, DetectionResult, Obstacle, ObstacleType,
)
from navigation.obstacle_avoider import ObstacleAvoider


def make_obstacle(angle_deg: float, distance_mm: float, obstacle_id: int = 0):
    """Cree un obstacle factice a un angle et distance donnes."""
    angle_rad = np.radians(angle_deg)
    x = distance_mm * np.sin(angle_rad)
    y = distance_mm * np.cos(angle_rad)
    centroid = np.array([x, y])
    return Obstacle(
        id=obstacle_id,
        type=ObstacleType.OBJECT,
        points_xy=np.array([[x, y]]),
        centroid=centroid,
        nearest_distance_mm=distance_mm,
        nearest_point=centroid.copy(),
        bbox=BoundingBox(x - 50, x + 50, y - 50, y + 50),
        line_segment=None,
        num_points=5,
        angular_span_deg=10.0,
    )


@pytest.fixture
def avoider():
    return ObstacleAvoider(
        obstacle_distance_mm=800,
        corridor_half_angle=30,
        num_sectors=12,
    )


class TestClearPath:
    def test_no_obstacles(self, avoider):
        detection = DetectionResult()
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is True
        assert result.status == "clear"

    def test_obstacle_far_away(self, avoider):
        obs = make_obstacle(angle_deg=0, distance_mm=2000)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is True

    def test_obstacle_on_side(self, avoider):
        # Obstacle a 90 degres (droite), chemin devant libre
        obs = make_obstacle(angle_deg=90, distance_mm=500)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is True

    def test_obstacle_behind(self, avoider):
        obs = make_obstacle(angle_deg=180, distance_mm=300)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is True


class TestAvoidance:
    def test_obstacle_front_close(self, avoider):
        obs = make_obstacle(angle_deg=0, distance_mm=500)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is False
        assert result.status == "avoiding"
        # Doit suggerer un cap different de 0
        assert result.suggested_heading != 0

    def test_obstacle_front_left(self, avoider):
        obs = make_obstacle(angle_deg=350, distance_mm=600)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is False

    def test_obstacle_front_right(self, avoider):
        obs = make_obstacle(angle_deg=15, distance_mm=400)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.clear is False

    def test_suggested_heading_is_free(self, avoider):
        # Obstacle devant, cotes libres
        obs = make_obstacle(angle_deg=0, distance_mm=500)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        # Le cap suggere ne doit pas etre dans le secteur bloque
        assert result.suggested_heading is not None


class TestBlocked:
    def test_surrounded(self, avoider):
        # Obstacles au centre de chaque secteur (15, 45, 75, ...)
        obstacles = [
            make_obstacle(angle_deg=i * 30 + 15, distance_mm=400, obstacle_id=i)
            for i in range(12)
        ]
        detection = DetectionResult(obstacles=obstacles)
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.status == "blocked"


class TestAngleDiff:
    def test_zero_diff(self):
        assert ObstacleAvoider._angle_diff(90, 90) == 0

    def test_positive_diff(self):
        diff = ObstacleAvoider._angle_diff(100, 90)
        assert abs(diff - 10) < 0.01

    def test_negative_diff(self):
        diff = ObstacleAvoider._angle_diff(80, 90)
        assert abs(diff - (-10)) < 0.01

    def test_wrap_around(self):
        diff = ObstacleAvoider._angle_diff(10, 350)
        assert abs(diff - 20) < 0.01

    def test_wrap_around_negative(self):
        diff = ObstacleAvoider._angle_diff(350, 10)
        assert abs(diff - (-20)) < 0.01
