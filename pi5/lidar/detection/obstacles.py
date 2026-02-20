"""Obstacle detection using DBSCAN clustering."""

import time

import numpy as np
from sklearn.cluster import DBSCAN

from lidar.data import Scan

from lidar.detection.geo import (
    compute_angular_span,
    compute_bounding_box,
    fit_line_segment,
    linearity_ratio,
)
from lidar.detection.types import DetectionResult, Obstacle, ObstacleType


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

        x, y, _ = scan.to_cartesian_arrays(min_quality=self.min_quality)

        if len(x) == 0:
            return DetectionResult(
                scan_timestamp=scan.timestamp,
                processing_time_ms=(time.perf_counter() - t0) * 1000,
            )

        points = np.column_stack((x, y))

        db = DBSCAN(eps=self.eps_mm, min_samples=self.min_samples)
        labels = db.fit_predict(points)

        noise_count = int(np.sum(labels == -1))
        cluster_ids = set(labels)
        cluster_ids.discard(-1)

        obstacles = []
        for i, cluster_id in enumerate(sorted(cluster_ids)):
            mask = labels == cluster_id
            cluster_points = points[mask]

            obstacle = self._build_obstacle(i, cluster_points)
            obstacles.append(obstacle)

        obstacles.sort(key=lambda o: o.nearest_distance_mm)

        elapsed_ms = (time.perf_counter() - t0) * 1000
        return DetectionResult(
            obstacles=obstacles,
            noise_points=noise_count,
            scan_timestamp=scan.timestamp,
            processing_time_ms=elapsed_ms,
        )

    def _build_obstacle(self, obstacle_id: int, points: np.ndarray) -> Obstacle:
        """Builds an Obstacle from a point cluster."""
        centroid = points.mean(axis=0)
        distances = np.linalg.norm(points, axis=1)
        nearest_idx = int(np.argmin(distances))
        nearest_distance = float(distances[nearest_idx])
        nearest_point = points[nearest_idx]

        bbox = compute_bounding_box(points)
        angular_span = compute_angular_span(points)

        obstacle_type, line_seg = self._classify(points)

        return Obstacle(
            id=obstacle_id,
            type=obstacle_type,
            points_xy=points,
            centroid=centroid,
            nearest_distance_mm=nearest_distance,
            nearest_point=nearest_point,
            bbox=bbox,
            line_segment=line_seg,
            num_points=len(points),
            angular_span_deg=angular_span,
        )

    def _classify(
        self, points: np.ndarray
    ) -> tuple[ObstacleType, "LineSegment | None"]:
        """Classifies a cluster into an obstacle type."""
        if len(points) < 2:
            return ObstacleType.UNKNOWN, None

        lin = linearity_ratio(points)
        line_seg = fit_line_segment(points)

        is_wall = (
            lin >= self.wall_linearity_threshold
            and line_seg.length_mm >= self.wall_min_length_mm
            and line_seg.fit_error_mm <= self.wall_max_fit_error_mm
        )

        if is_wall:
            return ObstacleType.WALL, line_seg

        return ObstacleType.OBJECT, None
