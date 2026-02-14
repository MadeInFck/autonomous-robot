"""Types de donnees pour la detection d'obstacles."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class ObstacleType(Enum):
    """Type d'obstacle detecte."""
    WALL = "wall"
    CORNER = "corner"
    OBJECT = "object"
    UNKNOWN = "unknown"


@dataclass
class LineSegment:
    """Segment de droite ajuste sur un ensemble de points."""
    start: np.ndarray          # (2,) point de debut
    end: np.ndarray            # (2,) point de fin
    direction: np.ndarray      # (2,) vecteur unitaire de direction
    normal: np.ndarray         # (2,) vecteur unitaire normal
    length_mm: float           # longueur du segment en mm
    fit_error_mm: float        # erreur RMS de l'ajustement en mm

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
    """Boite englobante alignee sur les axes."""
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
    """Un obstacle detecte dans un scan."""
    id: int
    type: ObstacleType
    points_xy: np.ndarray              # (N, 2) coordonnees des points
    centroid: np.ndarray               # (2,) centre de masse
    nearest_distance_mm: float         # distance au point le plus proche de l'origine
    nearest_point: np.ndarray          # (2,) point le plus proche
    bbox: BoundingBox
    line_segment: Optional[LineSegment]  # renseigne si type == WALL
    num_points: int
    angular_span_deg: float            # etendue angulaire depuis l'origine

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
    """Resultat complet d'une detection sur un scan."""
    obstacles: list[Obstacle] = field(default_factory=list)
    noise_points: int = 0
    scan_timestamp: float = 0.0
    processing_time_ms: float = 0.0

    @property
    def nearest_obstacle(self) -> Optional[Obstacle]:
        """Retourne l'obstacle le plus proche, ou None."""
        if not self.obstacles:
            return None
        return min(self.obstacles, key=lambda o: o.nearest_distance_mm)

    def by_type(self, obstacle_type: ObstacleType) -> list[Obstacle]:
        """Filtre les obstacles par type."""
        return [o for o in self.obstacles if o.type == obstacle_type]
