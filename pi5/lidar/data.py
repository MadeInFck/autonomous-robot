"""Data models for LiDAR scans."""

import time
from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class ScanPoint:
    """A single LiDAR measurement point.

    Robot convention: 0deg = forward = +Y, 90deg = right = +X (clockwise).
    Slotted: a revolution is ~500 of them, created ~5000 times per second.
    """

    angle_deg: float       # Angle in degrees (0-360)
    distance_mm: float     # Distance in millimeters
    quality: int           # Signal quality (0-63)

    @property
    def is_valid(self) -> bool:
        """True if the point has non-zero distance and quality."""
        return self.distance_mm > 0 and self.quality > 0

    @classmethod
    def from_sdk_dict(cls, d: dict) -> "ScanPoint":
        """Creates a ScanPoint from the rplidarc1 SDK format.

        The SDK returns: {"q": int, "a_deg": float, "d_mm": float|None}
        """
        return cls(
            angle_deg=d["a_deg"],
            distance_mm=float(d.get("d_mm") or 0.0),
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

    def to_xy(self, min_quality: int = 1) -> np.ndarray:
        """Returns the valid points above the quality threshold as an (N, 2) array of (x_mm, y_mm)."""
        polar = [(p.angle_deg, p.distance_mm) for p in self.points
                 if p.is_valid and p.quality >= min_quality]
        if not polar:
            return np.empty((0, 2))
        polar = np.asarray(polar, dtype=float)
        angles = np.radians(polar[:, 0])
        dists = polar[:, 1]
        return np.column_stack((dists * np.sin(angles), dists * np.cos(angles)))
