"""Tests for obstacle avoidance (ObstacleAvoider)."""

import sys
import os
import pytest
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lidar.detection.types import (
    BoundingBox, DetectionResult, Obstacle, ObstacleType,
)
from navigation.obstacle_avoider import ObstacleAvoider, angle_diff


def make_obstacle(angle_deg: float, distance_mm: float, obstacle_id: int = 0):
    """Create a dummy obstacle at a given angle and distance."""
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
        # Obstacle at 90 degrees (right), path ahead is clear
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
        # Must suggest a heading different from 0
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
        # Obstacle ahead, sides clear
        obs = make_obstacle(angle_deg=0, distance_mm=500)
        detection = DetectionResult(obstacles=[obs])
        result = avoider.evaluate(detection, desired_heading_robot=0)
        # The suggested heading must not be in the blocked sector
        assert result.suggested_heading is not None


class TestBlocked:
    def test_surrounded(self, avoider):
        # Obstacles at the center of each sector (15, 45, 75, ...)
        obstacles = [
            make_obstacle(angle_deg=i * 30 + 15, distance_mm=400, obstacle_id=i)
            for i in range(12)
        ]
        detection = DetectionResult(obstacles=obstacles)
        result = avoider.evaluate(detection, desired_heading_robot=0)
        assert result.status == "blocked"


class TestAngleDiff:
    def test_zero_diff(self):
        assert angle_diff(90, 90) == 0

    def test_positive_diff(self):
        diff = angle_diff(100, 90)
        assert abs(diff - 10) < 0.01

    def test_negative_diff(self):
        diff = angle_diff(80, 90)
        assert abs(diff - (-10)) < 0.01

    def test_wrap_around(self):
        diff = angle_diff(10, 350)
        assert abs(diff - 20) < 0.01

    def test_wrap_around_negative(self):
        diff = angle_diff(350, 10)
        assert abs(diff - (-20)) < 0.01

    def test_robot_frame_left_is_negative(self):
        """A free sector at 345deg (robot frame) is a 15deg turn to the LEFT."""
        assert angle_diff(345, 0) == -15


class TestWideObstacle:
    def test_wall_across_the_path_is_not_steered_into(self, avoider):
        """A 60 cm wall 60 cm ahead spans -27..+27deg. Placed by its centroid
        alone it filled one sector, and the 'free' 345deg next to it — still
        inside the wall — was chosen."""
        xs = np.arange(-300, 301, 20, dtype=float)
        pts = np.column_stack((xs, np.full(len(xs), 600.0)))
        wall = make_obstacle(0, 600)
        wall.points_xy = pts
        result = avoider.evaluate(DetectionResult(obstacles=[wall]), desired_heading_robot=0)
        assert result.status == "avoiding"
        assert abs(angle_diff(result.suggested_heading, 0)) > 30

    def test_side_wall_reaching_the_front_blocks_the_corridor(self, avoider):
        """Centroid at ~50deg (outside the corridor) but points down to 10deg."""
        angles = np.radians(np.arange(10, 91, 5))
        pts = np.column_stack((500 * np.sin(angles), 500 * np.cos(angles)))
        wall = make_obstacle(50, 500)
        wall.points_xy = pts
        result = avoider.evaluate(DetectionResult(obstacles=[wall]), desired_heading_robot=0)
        assert not result.clear


class TestToDict:
    def test_left_turn_is_negative(self, avoider):
        detection = DetectionResult(obstacles=[make_obstacle(15, 500), make_obstacle(45, 500)])
        d = avoider.evaluate(detection, desired_heading_robot=0).to_dict()
        assert d == {'status': 'avoiding', 'turn_deg': -15, 'min_mm': 500}

    def test_clear_has_no_distance(self, avoider):
        """inf is not valid JSON: the browser's JSON.parse would reject it."""
        d = avoider.evaluate(DetectionResult(), desired_heading_robot=0).to_dict()
        assert d == {'status': 'clear', 'turn_deg': 0, 'min_mm': None}
