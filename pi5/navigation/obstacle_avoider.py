"""Evitement d'obstacles temps reel base sur les scans lidar."""

import math
from dataclasses import dataclass
from typing import Optional

from lidar.detection.types import DetectionResult


@dataclass
class AvoidanceResult:
    """Resultat de l'analyse d'evitement."""
    clear: bool                   # True si le chemin est libre
    suggested_heading: float      # Cap suggere en degres (0-360)
    min_distance_mm: float        # Distance de l'obstacle le plus proche dans le couloir
    status: str                   # "clear", "avoiding", "blocked"


class ObstacleAvoider:
    """Analyse les obstacles detectes et suggere un cap d'evitement.

    Divise le champ de vision en secteurs et cherche le passage libre
    le plus proche du cap souhaite.

    Convention : 0deg = avant du robot (+Y), 90deg = droite (+X).
    Le cap du robot est donne par le GPS heading.
    Les angles lidar sont dans le repere robot (0deg = avant).
    """

    def __init__(
        self,
        obstacle_distance_mm: float = 800,
        corridor_half_angle: float = 30,
        num_sectors: int = 12,
    ):
        """
        Args:
            obstacle_distance_mm: Distance min avant evitement (mm).
            corridor_half_angle: Demi-angle du couloir frontal (degres).
            num_sectors: Nombre de secteurs pour l'analyse 360deg.
        """
        self.obstacle_distance_mm = obstacle_distance_mm
        self.corridor_half_angle = corridor_half_angle
        self.num_sectors = num_sectors
        self.sector_size = 360.0 / num_sectors

    def evaluate(
        self, detection: DetectionResult, desired_heading_robot: float
    ) -> AvoidanceResult:
        """Evalue si le chemin est libre et suggere un cap.

        Args:
            detection: Resultat de detection d'obstacles (repere robot).
            desired_heading_robot: Cap souhaite dans le repere robot (degres, 0=avant).

        Returns:
            AvoidanceResult avec le cap suggere.
        """
        if not detection.obstacles:
            return AvoidanceResult(
                clear=True,
                suggested_heading=desired_heading_robot,
                min_distance_mm=float('inf'),
                status="clear",
            )

        # Construire la carte des secteurs occupes
        sector_min_dist = [float('inf')] * self.num_sectors
        for obstacle in detection.obstacles:
            # Angle du centroide dans le repere robot
            cx, cy = obstacle.centroid
            angle = math.degrees(math.atan2(cx, cy)) % 360
            sector_idx = int(angle / self.sector_size) % self.num_sectors
            dist = obstacle.nearest_distance_mm
            if dist < sector_min_dist[sector_idx]:
                sector_min_dist[sector_idx] = dist

        # Verifier le couloir frontal autour du cap souhaite
        min_front_dist = self._check_corridor(
            sector_min_dist, desired_heading_robot
        )

        if min_front_dist > self.obstacle_distance_mm:
            return AvoidanceResult(
                clear=True,
                suggested_heading=desired_heading_robot,
                min_distance_mm=min_front_dist,
                status="clear",
            )

        # Chemin bloque : chercher le secteur libre le plus proche
        best_heading = self._find_free_sector(
            sector_min_dist, desired_heading_robot
        )

        if best_heading is not None:
            return AvoidanceResult(
                clear=False,
                suggested_heading=best_heading,
                min_distance_mm=min_front_dist,
                status="avoiding",
            )

        # Tout bloque
        return AvoidanceResult(
            clear=False,
            suggested_heading=desired_heading_robot,
            min_distance_mm=min_front_dist,
            status="blocked",
        )

    def _check_corridor(
        self, sector_min_dist: list[float], heading: float
    ) -> float:
        """Retourne la distance min dans le couloir autour du cap."""
        min_dist = float('inf')
        for i in range(self.num_sectors):
            sector_center = (i + 0.5) * self.sector_size
            diff = self._angle_diff(sector_center, heading)
            if abs(diff) <= self.corridor_half_angle:
                if sector_min_dist[i] < min_dist:
                    min_dist = sector_min_dist[i]
        return min_dist

    def _find_free_sector(
        self, sector_min_dist: list[float], desired_heading: float
    ) -> Optional[float]:
        """Trouve le secteur libre le plus proche du cap souhaite."""
        best_heading = None
        best_diff = 360.0

        for i in range(self.num_sectors):
            if sector_min_dist[i] > self.obstacle_distance_mm:
                sector_center = (i + 0.5) * self.sector_size
                diff = abs(self._angle_diff(sector_center, desired_heading))
                if diff < best_diff:
                    best_diff = diff
                    best_heading = sector_center

        return best_heading

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Difference d'angle signee, resultat dans [-180, 180]."""
        diff = (a - b + 180) % 360 - 180
        return diff
