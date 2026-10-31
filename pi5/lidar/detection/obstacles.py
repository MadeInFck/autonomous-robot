"""Obstacle detection using DBSCAN clustering."""

import time
from typing import Optional

import numpy as np

from lidar.data import Scan
from lidar.detection.clustering import dbscan
from lidar.detection.geo import (
    compute_angular_span,
    compute_bounding_box,
    fit_line_segment,
)
from lidar.detection.types import DetectionResult, LineSegment, Obstacle, ObstacleType


class ObstacleDetector:
    """Detects and classifies obstacles in a LiDAR scan.

    Pipeline:
      1. Extract cartesian coordinates (filtered by quality)
      2. DBSCAN clustering on (x, y) points
      3. For each cluster: geometric computation + classification
      4. Sort by ascending distance
    """

    def __init__(
        self,
        min_quality: int = 10,
        eps_mm: float = 150,
        min_samples: int = 3,
        wall_linearity_threshold: float = 0.85,
        wall_min_length_mm: float = 300,
        wall_max_fit_error_mm: float = 50,
    ):
        self.min_quality = min_quality
        self.eps_mm = eps_mm
        self.min_samples = min_samples
        self.wall_linearity_threshold = wall_linearity_threshold
        self.wall_min_length_mm = wall_min_length_mm
        self.wall_max_fit_error_mm = wall_max_fit_error_mm

    def detect(self, scan: Scan) -> DetectionResult:
        """Detects obstacles in a scan."""
        t0 = time.perf_counter()

        points = scan.to_xy(min_quality=self.min_quality)
        labels = dbscan(points, eps=self.eps_mm, min_samples=self.min_samples)

        obstacles = [
            self._build_obstacle(i, points[labels == i])
            for i in range(labels.max() + 1 if len(labels) else 0)
        ]
        obstacles.sort(key=lambda o: o.nearest_distance_mm)

        return DetectionResult(
            obstacles=obstacles,
            noise_points=int(np.sum(labels == -1)),
            scan_timestamp=scan.timestamp,
            processing_time_ms=(time.perf_counter() - t0) * 1000,
        )

    def _build_obstacle(self, obstacle_id: int, points: np.ndarray) -> Obstacle:
        """Builds an Obstacle from a point cluster."""
        distances = np.linalg.norm(points, axis=1)
        nearest_idx = int(np.argmin(distances))
        obstacle_type, line_seg = self._classify(points)

        return Obstacle(
            id=obstacle_id,
            type=obstacle_type,
            points_xy=points,
            centroid=points.mean(axis=0),
            nearest_distance_mm=float(distances[nearest_idx]),
            nearest_point=points[nearest_idx],
            bbox=compute_bounding_box(points),
            line_segment=line_seg,
            num_points=len(points),
            angular_span_deg=compute_angular_span(points),
        )

    def _classify(self, points: np.ndarray) -> tuple[ObstacleType, Optional[LineSegment]]:
        """Classifies a cluster into an obstacle type."""
        if len(points) < 2:
            return ObstacleType.UNKNOWN, None

        line_seg = fit_line_segment(points)
        is_wall = (
            line_seg.linearity >= self.wall_linearity_threshold
            and line_seg.length_mm >= self.wall_min_length_mm
            and line_seg.fit_error_mm <= self.wall_max_fit_error_mm
        )
        if is_wall:
            return ObstacleType.WALL, line_seg
        return ObstacleType.OBJECT, None
