"""Real-time obstacle avoidance based on lidar scans."""

import math
from dataclasses import dataclass
from typing import Optional

from lidar.detection.types import DetectionResult


@dataclass
class AvoidanceResult:
    """Result of the avoidance analysis."""
    clear: bool                   # True if the path is clear
    suggested_heading: float      # Suggested heading in degrees (0-360)
    min_distance_mm: float        # Distance to the nearest obstacle in the corridor
    status: str                   # "clear", "avoiding", "blocked"


class ObstacleAvoider:
    """Analyzes detected obstacles and suggests an avoidance heading.

    Divides the field of view into sectors and finds the free passage
    closest to the desired heading.

    Convention: 0deg = robot front (+Y), 90deg = right (+X).
    Robot heading is given by GPS heading.
    Lidar angles are in robot frame (0deg = forward).
    """

    def __init__(
        self,
        obstacle_distance_mm: float = 800,
        corridor_half_angle: float = 30,
        num_sectors: int = 12,
    ):
        """
        Args:
            obstacle_distance_mm: Minimum distance before avoidance (mm).
            corridor_half_angle: Half-angle of the frontal corridor (degrees).
            num_sectors: Number of sectors for the 360deg analysis.
        """
        self.obstacle_distance_mm = obstacle_distance_mm
        self.corridor_half_angle = corridor_half_angle
        self.num_sectors = num_sectors
        self.sector_size = 360.0 / num_sectors

    def evaluate(
        self, detection: DetectionResult, desired_heading_robot: float
    ) -> AvoidanceResult:
        """Evaluates whether the path is clear and suggests a heading.

        Args:
            detection: Obstacle detection result (robot frame).
            desired_heading_robot: Desired heading in robot frame (degrees, 0=forward).

        Returns:
            AvoidanceResult with the suggested heading.
        """
        if not detection.obstacles:
            return AvoidanceResult(
                clear=True,
                suggested_heading=desired_heading_robot,
                min_distance_mm=float('inf'),
                status="clear",
            )

        # Build the occupied sector map
        sector_min_dist = [float('inf')] * self.num_sectors
        for obstacle in detection.obstacles:
            # Centroid angle in robot frame
            cx, cy = obstacle.centroid
            angle = math.degrees(math.atan2(cx, cy)) % 360
            sector_idx = int(angle / self.sector_size) % self.num_sectors
            dist = obstacle.nearest_distance_mm
            if dist < sector_min_dist[sector_idx]:
                sector_min_dist[sector_idx] = dist

        # Check the frontal corridor around the desired heading
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

        # Path blocked: find the nearest free sector
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

        # Completely blocked
        return AvoidanceResult(
            clear=False,
            suggested_heading=desired_heading_robot,
            min_distance_mm=min_front_dist,
            status="blocked",
        )

    def _check_corridor(
        self, sector_min_dist: list[float], heading: float
    ) -> float:
        """Returns the minimum distance in the corridor around the heading."""
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
        """Finds the free sector closest to the desired heading."""
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
        """Signed angle difference, result in [-180, 180]."""
        diff = (a - b + 180) % 360 - 180
        return diff
