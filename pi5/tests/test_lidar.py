"""Tests for the lidar pipeline: scan assembly, DBSCAN, detection, tracking."""

import sys
import os
import time
import unittest.mock

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.modules.setdefault('rplidarc1', unittest.mock.MagicMock())

from lidar.data import Scan, ScanPoint
from lidar.scanner import LidarScanner
from lidar.detection.clustering import dbscan
from lidar.detection.obstacles import ObstacleDetector
from lidar.detection.tracker import ObstacleTracker
from lidar.detection.types import ObstacleType


def polar_points(angle_deg, distance_mm, quality=40):
    return [ScanPoint(a, d, quality) for a, d in zip(angle_deg, distance_mm)]


# ===== Scan assembly =====

class TestScanAssembly:
    @staticmethod
    def _feed(scanner, angles, quality=40, distance=1000.0):
        for a in angles:
            scanner._accumulate_point({'a_deg': float(a), 'd_mm': distance, 'q': quality})

    def test_scan_published_on_wrap(self):
        scanner = LidarScanner('/dev/null')
        self._feed(scanner, range(0, 360, 2))
        assert scanner.get_last_scan() is None          # revolution not closed yet
        self._feed(scanner, [1])
        scan = scanner.get_last_scan()
        assert scan is not None and scan.num_points == 180

    def test_only_last_scan_kept(self):
        """Every revolution used to be queued forever (memory leak)."""
        scanner = LidarScanner('/dev/null')
        for _ in range(50):
            self._feed(scanner, range(0, 360, 2))
        assert scanner.get_last_scan().num_points == 180
        assert not hasattr(scanner, '_scan_queue')

    def test_low_quality_points_dropped(self):
        scanner = LidarScanner('/dev/null', min_quality=10)
        self._feed(scanner, range(0, 360, 2), quality=5)
        self._feed(scanner, range(0, 360, 2), quality=20)
        self._feed(scanner, [1])
        assert scanner.get_last_scan().num_points == 180

    def test_missing_distance_is_invalid(self):
        point = ScanPoint.from_sdk_dict({'a_deg': 10.0, 'd_mm': None, 'q': 30})
        assert point.distance_mm == 0.0 and not point.is_valid


class TestStartStop:
    """The dashboard switches the lidar off and on again."""

    @pytest.fixture
    def scanner(self, monkeypatch):
        monkeypatch.setattr(LidarScanner, '_run_loop', lambda self, loop: loop.close())
        # A fresh SDK object per connection, like the real constructor
        monkeypatch.setattr('lidar.scanner.RPLidar', lambda *a: unittest.mock.MagicMock())
        return LidarScanner('/dev/null')

    def test_restart_after_stop(self, scanner):
        scanner.start()
        first = scanner._lidar
        TestScanAssembly._feed(scanner, list(range(0, 360, 2)) + [1])
        assert scanner.get_last_scan() is not None
        scanner.stop()
        assert not scanner.is_running
        assert scanner.get_last_scan() is None        # no frozen scan shown as live
        first.shutdown.assert_called_once()
        scanner.start()
        assert scanner.is_running and scanner._lidar is not first

    def test_stop_and_start_are_idempotent(self, scanner):
        scanner.stop()                                # never started: no-op
        scanner.start()
        lidar = scanner._lidar
        scanner.start()                               # already running: same port
        assert scanner._lidar is lidar


class TestScanGeometry:
    def test_to_xy_robot_frame(self):
        """0deg = forward = +Y, 90deg = right = +X."""
        scan = Scan(points=polar_points([0, 90, 180], [1000, 2000, 500]))
        xy = scan.to_xy()
        np.testing.assert_allclose(xy, [[0, 1000], [2000, 0], [0, -500]], atol=1e-9)

    def test_to_xy_filters(self):
        scan = Scan(points=[ScanPoint(0, 1000, 40), ScanPoint(10, 0, 40), ScanPoint(20, 900, 3)])
        assert scan.to_xy(min_quality=10).shape == (1, 2)
        assert Scan().to_xy().shape == (0, 2)


# ===== DBSCAN =====

class TestDbscan:
    def test_two_blobs_and_noise(self):
        blob_a = np.array([[0, 0], [10, 0], [0, 10], [10, 10]], dtype=float)
        blob_b = blob_a + 1000
        noise = np.array([[5000.0, 5000.0]])
        labels = dbscan(np.vstack([blob_a, blob_b, noise]), eps=50, min_samples=3)
        assert list(labels) == [0, 0, 0, 0, 1, 1, 1, 1, -1]

    def test_min_samples_counts_the_point_itself(self):
        pair = np.array([[0.0, 0.0], [10.0, 0.0]])
        assert list(dbscan(pair, eps=50, min_samples=2)) == [0, 0]
        assert list(dbscan(pair, eps=50, min_samples=3)) == [-1, -1]

    def test_eps_is_inclusive(self):
        pts = np.array([[0.0, 0.0], [150.0, 0.0], [300.0, 0.0]])
        assert list(dbscan(pts, eps=150, min_samples=2)) == [0, 0, 0]

    def test_chain_is_one_cluster(self):
        chain = np.column_stack((np.arange(0, 2000, 100.0), np.zeros(20)))
        assert set(dbscan(chain, eps=150, min_samples=3)) == {0}

    def test_border_point_kept_by_first_cluster(self):
        # x=100 is within eps of a core point of each group but is not a core
        # itself: it joins whichever cluster is found first, and does not merge them
        group_a = [-10, -5, -2, 0]
        group_b = [200, 202, 205, 210]

        def run(xs):
            pts = np.column_stack((np.array(xs, dtype=float), np.zeros(len(xs))))
            return list(dbscan(pts, eps=100, min_samples=4))

        assert run(group_a + [100] + group_b) == [0] * 5 + [1] * 4
        assert run(group_b + [100] + group_a) == [0] * 5 + [1] * 4

    def test_empty(self):
        assert dbscan(np.empty((0, 2)), eps=150, min_samples=3).shape == (0,)


# ===== Detection =====

def wall_scan(distance_mm=1500, half_width_mm=1000, step_mm=20):
    """A straight wall in front of the robot."""
    xs = np.arange(-half_width_mm, half_width_mm + 1, step_mm, dtype=float)
    angles = np.degrees(np.arctan2(xs, distance_mm)) % 360
    dists = np.hypot(xs, distance_mm)
    return Scan(points=polar_points(angles, dists), timestamp=time.time())


def object_scan(angle_deg=90, distance_mm=700, radius_mm=60):
    """A small round object, plus far scattered noise."""
    rng = np.random.default_rng(0)
    centre = np.array([np.sin(np.radians(angle_deg)), np.cos(np.radians(angle_deg))]) * distance_mm
    blob = centre + rng.normal(0, radius_mm / 2, (25, 2))
    noise = rng.uniform(-6000, 6000, (5, 2))
    pts = np.vstack([blob, noise])
    angles = np.degrees(np.arctan2(pts[:, 0], pts[:, 1])) % 360
    return Scan(points=polar_points(angles, np.hypot(pts[:, 0], pts[:, 1])), timestamp=time.time())


class TestObstacleDetector:
    def test_wall_classified(self):
        result = ObstacleDetector().detect(wall_scan())
        assert len(result.obstacles) == 1
        wall = result.obstacles[0]
        assert wall.type is ObstacleType.WALL
        assert wall.line_segment.length_mm > 1800
        assert wall.line_segment.linearity > 0.99
        assert abs(wall.nearest_distance_mm - 1500) < 1

    def test_object_found_where_it_is(self):
        result = ObstacleDetector().detect(object_scan(angle_deg=90, distance_mm=700))
        nearest = result.nearest_obstacle
        assert nearest.type is ObstacleType.OBJECT
        assert nearest.centroid[0] > 600 and abs(nearest.centroid[1]) < 100   # to the right
        assert result.noise_points >= 4

    def test_empty_scan(self):
        result = ObstacleDetector().detect(Scan())
        assert result.obstacles == [] and result.noise_points == 0


# ===== Tracking =====

class TestObstacleTracker:
    def test_confirmed_after_two_hits(self):
        tracker = ObstacleTracker(min_hits=2)
        detector = ObstacleDetector()
        assert tracker.update(detector.detect(object_scan())).obstacles == []
        assert len(tracker.update(detector.detect(object_scan())).obstacles) == 1

    def test_position_smoothed(self):
        tracker = ObstacleTracker(min_hits=1, smoothing=0.5)
        detector = ObstacleDetector()
        first = tracker.update(detector.detect(object_scan(distance_mm=700))).obstacles[0]
        x0 = first.centroid[0]
        moved = tracker.update(detector.detect(object_scan(distance_mm=800))).obstacles[0]
        assert x0 < moved.centroid[0] < x0 + 100      # half-way, not a jump

    def test_track_expires(self):
        tracker = ObstacleTracker(min_hits=1, max_age=2)
        detector = ObstacleDetector()
        tracker.update(detector.detect(object_scan()))
        for _ in range(3):
            last = tracker.update(detector.detect(Scan()))
        assert last.obstacles == []
