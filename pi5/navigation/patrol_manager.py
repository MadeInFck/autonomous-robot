"""Gestion du parcours de patrouille par waypoints GPS."""

import json
import math
import os
from dataclasses import dataclass, asdict
from typing import Optional


# Rayon moyen de la Terre en metres
EARTH_RADIUS_M = 6_371_000


@dataclass
class Waypoint:
    """Un point de passage GPS."""
    latitude: float
    longitude: float
    label: str = ""


class PatrolManager:
    """Gere une liste de waypoints GPS pour la patrouille.

    Fonctionnalites :
      - Enregistrement / suppression de waypoints
      - Sauvegarde / chargement depuis JSON
      - Navigation sequentielle avec boucle
      - Calcul de bearing et distance (haversine)
    """

    def __init__(self, waypoint_radius: float = 3.0):
        """
        Args:
            waypoint_radius: Distance en metres pour considerer un waypoint atteint.
        """
        self._waypoints: list[Waypoint] = []
        self._current_index: int = 0
        self.waypoint_radius = waypoint_radius

    # --- CRUD waypoints ---

    def record_waypoint(self, lat: float, lon: float, label: str = "") -> int:
        """Enregistre un nouveau waypoint. Retourne son index."""
        if not label:
            label = f"WP{len(self._waypoints)}"
        wp = Waypoint(latitude=lat, longitude=lon, label=label)
        self._waypoints.append(wp)
        return len(self._waypoints) - 1

    def delete_waypoint(self, index: int) -> bool:
        """Supprime un waypoint par index. Retourne True si supprime."""
        if 0 <= index < len(self._waypoints):
            self._waypoints.pop(index)
            if self._current_index >= len(self._waypoints):
                self._current_index = max(0, len(self._waypoints) - 1)
            return True
        return False

    def get_waypoints(self) -> list[dict]:
        """Retourne la liste des waypoints avec index courant."""
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
        """Retourne le waypoint courant, ou None si pas de waypoints."""
        if not self._waypoints:
            return None
        if self._current_index >= len(self._waypoints):
            return None
        return self._waypoints[self._current_index]

    def advance(self) -> bool:
        """Passe au waypoint suivant. Retourne False si patrouille terminee."""
        if not self._waypoints:
            return False
        self._current_index += 1
        if self._current_index >= len(self._waypoints):
            return False
        return True

    def is_complete(self) -> bool:
        """True si tous les waypoints ont ete visites."""
        return self._current_index >= len(self._waypoints)

    def reset(self):
        """Remet l'index a 0 pour recommencer la patrouille."""
        self._current_index = 0

    # --- Calculs GPS ---

    def bearing_to_target(self, lat: float, lon: float) -> Optional[float]:
        """Calcule le cap (bearing) vers le waypoint courant en degres (0-360).

        Args:
            lat, lon: Position actuelle en degres decimaux.

        Returns:
            Cap en degres (0=Nord, 90=Est) ou None si pas de cible.
        """
        target = self.get_current_target()
        if target is None:
            return None
        return self._bearing(lat, lon, target.latitude, target.longitude)

    def distance_to_target(self, lat: float, lon: float) -> Optional[float]:
        """Calcule la distance au waypoint courant en metres.

        Args:
            lat, lon: Position actuelle en degres decimaux.

        Returns:
            Distance en metres ou None si pas de cible.
        """
        target = self.get_current_target()
        if target is None:
            return None
        return self._haversine(lat, lon, target.latitude, target.longitude)

    def is_target_reached(self, lat: float, lon: float) -> bool:
        """True si la position actuelle est dans le rayon du waypoint courant."""
        dist = self.distance_to_target(lat, lon)
        if dist is None:
            return False
        return dist <= self.waypoint_radius

    # --- Persistance JSON ---

    def save(self, filepath: str):
        """Sauvegarde les waypoints en JSON."""
        data = {
            "waypoints": [asdict(wp) for wp in self._waypoints],
            "waypoint_radius": self.waypoint_radius,
        }
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def load(self, filepath: str) -> bool:
        """Charge les waypoints depuis JSON. Retourne True si reussi."""
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

    # --- Formules geodesiques ---

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distance haversine entre deux points GPS en metres."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS_M * c

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Bearing initial entre deux points GPS en degres (0-360)."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))
        return bearing % 360
