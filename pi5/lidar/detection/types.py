"""Data types for obstacle detection."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class ObstacleType(Enum):
    """Detected obstacle type."""
    WALL = "wall"
    CORNER = "corner"
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

    def to_dict(self) -> dict:
        return {
            "start": self.start.tolist(),
            "end": self.end.tolist(),
            "direction": self.direction.tolist(),
            "normal": self.normal.tolist(),
            "length_mm": self.length_mm,
            "fit_error_mm": self.fit_error_mm,
        }


@dataclass
class BoundingBox:
    """Axis-aligned bounding box."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)

    def to_dict(self) -> dict:
        return {
            "x_min": self.x_min, "x_max": self.x_max,
            "y_min": self.y_min, "y_max": self.y_max,
        }


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

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "type": self.type.value,
            "centroid": self.centroid.tolist(),
            "nearest_distance_mm": self.nearest_distance_mm,
            "nearest_point": self.nearest_point.tolist(),
            "bbox": self.bbox.to_dict(),
            "line_segment": self.line_segment.to_dict() if self.line_segment else None,
            "num_points": self.num_points,
            "angular_span_deg": self.angular_span_deg,
        }


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

    def by_type(self, obstacle_type: ObstacleType) -> list[Obstacle]:
        """Filters obstacles by type."""
        return [o for o in self.obstacles if o.type == obstacle_type]
