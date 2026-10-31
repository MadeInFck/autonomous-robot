"""Data types for obstacle detection."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class ObstacleType(Enum):
    """Detected obstacle type."""
    WALL = "wall"
    OBJECT = "object"
    UNKNOWN = "unknown"


@dataclass
class LineSegment:
    """Line segment fitted to a set of points."""
    start: np.ndarray          # (2,) start point
    end: np.ndarray            # (2,) end point
    direction: np.ndarray      # (2,) unit direction vector
    normal: np.ndarray         # (2,) unit normal vector
    length_mm: float           # segment length in mm
    fit_error_mm: float        # RMS fit error in mm
    linearity: float           # PCA linearity ratio (0.5=disc, 1=line)


@dataclass
class BoundingBox:
    """Axis-aligned bounding box."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass
class Obstacle:
    """A detected obstacle in a scan."""
    id: int
    type: ObstacleType
    points_xy: np.ndarray              # (N, 2) point coordinates
    centroid: np.ndarray               # (2,) center of mass
    nearest_distance_mm: float         # distance to the nearest point from origin
    nearest_point: np.ndarray          # (2,) nearest point
    bbox: BoundingBox
    line_segment: Optional[LineSegment]  # populated if type == WALL
    num_points: int
    angular_span_deg: float            # angular span from origin


@dataclass
class DetectionResult:
    """Complete result of a detection on a scan."""
    obstacles: list[Obstacle] = field(default_factory=list)
    noise_points: int = 0
    scan_timestamp: float = 0.0
    processing_time_ms: float = 0.0

    @property
    def nearest_obstacle(self) -> Optional[Obstacle]:
        """Returns the nearest obstacle, or None."""
        if not self.obstacles:
            return None
        return min(self.obstacles, key=lambda o: o.nearest_distance_mm)
