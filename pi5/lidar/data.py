"""Data models for LiDAR scans."""

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class ScanPoint:
    """A single LiDAR measurement point."""

    angle_deg: float       # Angle in degrees (0-360)
    distance_mm: float     # Distance in millimeters
    quality: int           # Signal quality (0-63)

    @property
    def x_mm(self) -> float:
        """X coordinate in mm. Robot convention: 0deg = forward = +Y, 90deg = right = +X."""
        return self.distance_mm * math.sin(math.radians(self.angle_deg))

    @property
    def y_mm(self) -> float:
        """Y coordinate in mm. Robot convention: 0deg = forward = +Y."""
        return self.distance_mm * math.cos(math.radians(self.angle_deg))

    @property
    def is_valid(self) -> bool:
        """True if the point has non-zero distance and quality."""
        return self.distance_mm > 0 and self.quality > 0

    def to_dict(self) -> dict:
        return {
            "angle_deg": self.angle_deg,
            "distance_mm": self.distance_mm,
            "quality": self.quality,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ScanPoint":
        return cls(
            angle_deg=d["angle_deg"],
            distance_mm=d["distance_mm"],
            quality=d["quality"],
        )

    @classmethod
    def from_sdk_dict(cls, d: dict) -> Optional["ScanPoint"]:
        """Creates a ScanPoint from the rplidarc1 SDK format.

        The SDK returns: {"q": int, "a_deg": float, "d_mm": float|None}
        """
        distance = d.get("d_mm")
        if distance is None:
            distance = 0.0
        return cls(
            angle_deg=d["a_deg"],
            distance_mm=float(distance),
            quality=d["q"],
        )


@dataclass
class Scan:
    """A complete scan (one 360deg revolution of the LiDAR)."""

    points: list[ScanPoint] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    @property
    def valid_points(self) -> list[ScanPoint]:
        """Points with valid distance and quality."""
        return [p for p in self.points if p.is_valid]

    @property
    def num_points(self) -> int:
        return len(self.points)

    @property
    def num_valid(self) -> int:
        return len(self.valid_points)

    @property
    def mean_quality(self) -> float:
        valid = self.valid_points
        if not valid:
            return 0.0
        return sum(p.quality for p in valid) / len(valid)

    def filter_by_quality(self, min_quality: int = 1) -> "Scan":
        """Returns a new Scan with only the points above the quality threshold."""
        return Scan(
            points=[p for p in self.points if p.quality >= min_quality],
            timestamp=self.timestamp,
        )

    def to_cartesian_arrays(self, min_quality: int = 1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (x_mm, y_mm, qualities) as numpy arrays.

        Only returns valid points above the quality threshold.
        """
        pts = [p for p in self.points if p.is_valid and p.quality >= min_quality]
        if not pts:
            return np.array([]), np.array([]), np.array([])

        angles = np.radians([p.angle_deg for p in pts])
        dists = np.array([p.distance_mm for p in pts])
        qualities = np.array([p.quality for p in pts])

        x = dists * np.sin(angles)
        y = dists * np.cos(angles)
        return x, y, qualities

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "num_points": self.num_points,
            "num_valid": self.num_valid,
            "mean_quality": round(self.mean_quality, 1),
            "points": [p.to_dict() for p in self.points],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Scan":
        return cls(
            points=[ScanPoint.from_dict(p) for p in d["points"]],
            timestamp=d.get("timestamp", 0.0),
        )
