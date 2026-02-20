"""GPS waypoint patrol route management."""

import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Optional


# Mean Earth radius in meters
EARTH_RADIUS_M = 6_371_000


@dataclass
class Waypoint:
    """A GPS waypoint."""
    latitude: float
    longitude: float
    label: str = ""


class PatrolManager:
    """Manages a list of GPS waypoints for patrol.

    Features:
      - Recording / deleting waypoints
      - Saving / loading from JSON
      - Sequential navigation with looping
      - Bearing and distance calculation (haversine)
    """

    def __init__(self, waypoint_radius: float = 3.0):
        """
        Args:
            waypoint_radius: Distance in meters to consider a waypoint reached.
        """
        self._waypoints: list[Waypoint] = []
        self._current_index: int = 0
        self.waypoint_radius = waypoint_radius

    # --- CRUD waypoints ---

    def record_waypoint(self, lat: float, lon: float, label: str = "") -> int:
        """Records a new waypoint. Returns its index."""
        if not label:
            label = f"WP{len(self._waypoints)}"
        wp = Waypoint(latitude=lat, longitude=lon, label=label)
        self._waypoints.append(wp)
        return len(self._waypoints) - 1

    def delete_waypoint(self, index: int) -> bool:
        """Deletes a waypoint by index. Returns True if deleted."""
        if 0 <= index < len(self._waypoints):
            self._waypoints.pop(index)
            if self._current_index >= len(self._waypoints):
                self._current_index = max(0, len(self._waypoints) - 1)
            return True
        return False

    def get_waypoints(self) -> list[dict]:
        """Returns the list of waypoints with current index."""
        return [
            {
                "index": i,
                "latitude": wp.latitude,
                "longitude": wp.longitude,
                "label": wp.label,
                "active": i == self._current_index,
            }
            for i, wp in enumerate(self._waypoints)
        ]

    @property
    def count(self) -> int:
        return len(self._waypoints)

    @property
    def current_index(self) -> int:
        return self._current_index

    # --- Navigation ---

    def get_current_target(self) -> Optional[Waypoint]:
        """Returns the current waypoint, or None if no waypoints."""
        if not self._waypoints:
            return None
        if self._current_index >= len(self._waypoints):
            return None
        return self._waypoints[self._current_index]

    def advance(self) -> bool:
        """Advances to the next waypoint. Returns False if patrol is complete."""
        if not self._waypoints:
            return False
        self._current_index += 1
        if self._current_index >= len(self._waypoints):
            return False
        return True

    def is_complete(self) -> bool:
        """True if all waypoints have been visited."""
        return self._current_index >= len(self._waypoints)

    def reset(self):
        """Resets the index to 0 to restart the patrol."""
        self._current_index = 0

    # --- GPS calculations ---

    def bearing_to_target(self, lat: float, lon: float) -> Optional[float]:
        """Computes the bearing to the current waypoint in degrees (0-360).

        Args:
            lat, lon: Current position in decimal degrees.

        Returns:
            Bearing in degrees (0=North, 90=East) or None if no target.
        """
        target = self.get_current_target()
        if target is None:
            return None
        return self._bearing(lat, lon, target.latitude, target.longitude)

    def distance_to_target(self, lat: float, lon: float) -> Optional[float]:
        """Computes the distance to the current waypoint in meters.

        Args:
            lat, lon: Current position in decimal degrees.

        Returns:
            Distance in meters or None if no target.
        """
        target = self.get_current_target()
        if target is None:
            return None
        return self._haversine(lat, lon, target.latitude, target.longitude)

    def is_target_reached(self, lat: float, lon: float) -> bool:
        """True if the current position is within the waypoint radius."""
        dist = self.distance_to_target(lat, lon)
        if dist is None:
            return False
        return dist <= self.waypoint_radius

    # --- JSON persistence ---

    def save(self, filepath: str):
        """Saves waypoints to JSON."""
        data = {
            "waypoints": [asdict(wp) for wp in self._waypoints],
            "waypoint_radius": self.waypoint_radius,
        }
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def load(self, filepath: str) -> bool:
        """Loads waypoints from JSON. Returns True if successful."""
        if not os.path.exists(filepath):
            return False
        with open(filepath, "r") as f:
            data = json.load(f)
        self._waypoints = [
            Waypoint(**wp) for wp in data.get("waypoints", [])
        ]
        if "waypoint_radius" in data:
            self.waypoint_radius = data["waypoint_radius"]
        self._current_index = 0
        return True

    # --- Geodesic formulas ---

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Haversine distance between two GPS points in meters."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS_M * c

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Initial bearing between two GPS points in degrees (0-360)."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))
        return bearing % 360
