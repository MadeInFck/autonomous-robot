"""Tests for the patrol manager (PatrolManager)."""

import json
import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from navigation.patrol_manager import PatrolManager, Waypoint


@pytest.fixture
def pm():
    return PatrolManager(waypoint_radius=3.0)


class TestWaypointCRUD:
    def test_record_waypoint(self, pm):
        idx = pm.record_waypoint(48.8566, 2.3522, "Paris")
        assert idx == 0
        assert pm.count == 1

    def test_record_multiple(self, pm):
        pm.record_waypoint(48.8566, 2.3522, "Paris")
        pm.record_waypoint(-33.8688, 151.2093, "Sydney")
        assert pm.count == 2

    def test_auto_label(self, pm):
        pm.record_waypoint(48.8566, 2.3522)
        wps = pm.get_waypoints()
        assert wps[0]["label"] == "WP0"

    def test_delete_waypoint(self, pm):
        pm.record_waypoint(48.8566, 2.3522, "A")
        pm.record_waypoint(0.0, 0.0, "B")
        assert pm.delete_waypoint(0)
        assert pm.count == 1
        assert pm.get_waypoints()[0]["label"] == "B"

    def test_delete_invalid_index(self, pm):
        assert pm.delete_waypoint(5) is False

    def test_get_waypoints_empty(self, pm):
        assert pm.get_waypoints() == []

    def test_get_waypoints_active_flag(self, pm):
        pm.record_waypoint(1.0, 1.0)
        pm.record_waypoint(2.0, 2.0)
        wps = pm.get_waypoints()
        assert wps[0]["active"] is True
        assert wps[1]["active"] is False


class TestNavigation:
    def test_get_current_target_empty(self, pm):
        assert pm.get_current_target() is None

    def test_get_current_target(self, pm):
        pm.record_waypoint(48.8566, 2.3522, "Paris")
        target = pm.get_current_target()
        assert target.latitude == 48.8566

    def test_advance(self, pm):
        pm.record_waypoint(1.0, 1.0)
        pm.record_waypoint(2.0, 2.0)
        assert pm.advance() is True
        assert pm.current_index == 1

    def test_advance_past_end(self, pm):
        pm.record_waypoint(1.0, 1.0)
        assert pm.advance() is False
        assert pm.is_complete()

    def test_is_complete_empty(self, pm):
        assert pm.is_complete()

    def test_reset(self, pm):
        pm.record_waypoint(1.0, 1.0)
        pm.record_waypoint(2.0, 2.0)
        pm.advance()
        pm.advance()
        assert pm.is_complete()
        pm.reset()
        assert pm.current_index == 0
        assert not pm.is_complete()


class TestGPSCalculations:
    def test_haversine_same_point(self, pm):
        pm.record_waypoint(48.8566, 2.3522)
        dist = pm.distance_to_target(48.8566, 2.3522)
        assert dist == 0.0

    def test_haversine_known_distance(self, pm):
        # Paris -> Lyon ~392 km
        pm.record_waypoint(45.7640, 4.8357)  # Lyon
        dist = pm.distance_to_target(48.8566, 2.3522)  # Paris
        assert 390_000 < dist < 395_000

    def test_haversine_short_distance(self, pm):
        # Two points ~100m apart
        pm.record_waypoint(48.8567, 2.3522)
        dist = pm.distance_to_target(48.8566, 2.3522)
        assert 10 < dist < 15  # ~11m pour 0.0001 deg de latitude

    def test_bearing_north(self, pm):
        # Point to the north: bearing ~0
        pm.record_waypoint(49.0, 2.3522)
        bearing = pm.bearing_to_target(48.0, 2.3522)
        assert abs(bearing - 0) < 1 or abs(bearing - 360) < 1

    def test_bearing_east(self, pm):
        # Point to the east: bearing ~90
        pm.record_waypoint(48.8566, 3.3522)
        bearing = pm.bearing_to_target(48.8566, 2.3522)
        assert 85 < bearing < 95

    def test_bearing_south(self, pm):
        # Point to the south: bearing ~180
        pm.record_waypoint(47.0, 2.3522)
        bearing = pm.bearing_to_target(48.0, 2.3522)
        assert 175 < bearing < 185

    def test_bearing_no_target(self, pm):
        assert pm.bearing_to_target(48.0, 2.0) is None

    def test_distance_no_target(self, pm):
        assert pm.distance_to_target(48.0, 2.0) is None

    def test_target_reached(self, pm):
        pm.record_waypoint(48.85660, 2.35220)
        # Position very close (< 3m)
        assert pm.is_target_reached(48.85660, 2.35220)

    def test_target_not_reached(self, pm):
        pm.record_waypoint(48.85660, 2.35220)
        # Position ~1km away
        assert not pm.is_target_reached(48.8566, 2.3400)


class TestPersistence:
    def test_save_load(self, pm):
        pm.record_waypoint(48.8566, 2.3522, "Paris")
        pm.record_waypoint(-33.8688, 151.2093, "Sydney")

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            filepath = f.name

        try:
            pm.save(filepath)

            pm2 = PatrolManager()
            assert pm2.load(filepath)
            assert pm2.count == 2

            wps = pm2.get_waypoints()
            assert wps[0]["label"] == "Paris"
            assert abs(wps[1]["latitude"] - (-33.8688)) < 0.0001
        finally:
            os.unlink(filepath)

    def test_load_nonexistent(self, pm):
        assert pm.load("/tmp/nonexistent_file_xyz.json") is False

    def test_save_creates_directory(self, pm):
        pm.record_waypoint(1.0, 1.0)
        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = os.path.join(tmpdir, "subdir", "route.json")
            pm.save(filepath)
            assert os.path.exists(filepath)
