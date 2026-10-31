"""Geometric functions for obstacle detection."""

import numpy as np

from lidar.detection.types import BoundingBox, LineSegment


def fit_line_segment(points: np.ndarray) -> LineSegment:
    """Fits a line segment to a set of points via PCA."""
    centroid = points.mean(axis=0)
    centered = points - centroid

    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(centered.T))
    direction = eigenvectors[:, 1]
    normal = eigenvectors[:, 0]

    projections = centered @ direction
    t_min, t_max = projections.min(), projections.max()

    errors = centered @ normal
    total = eigenvalues.sum()

    return LineSegment(
        start=centroid + t_min * direction,
        end=centroid + t_max * direction,
        direction=direction,
        normal=normal,
        length_mm=float(t_max - t_min),
        fit_error_mm=float(np.sqrt(np.mean(errors ** 2))),
        # Share of the variance along the main axis (0.5=disc, 1=line)
        linearity=float(eigenvalues[1] / total) if total > 0 else 0.0,
    )


def compute_bounding_box(points: np.ndarray) -> BoundingBox:
    """Computes the axis-aligned bounding box."""
    return BoundingBox(
        x_min=float(points[:, 0].min()),
        x_max=float(points[:, 0].max()),
        y_min=float(points[:, 1].min()),
        y_max=float(points[:, 1].max()),
    )


def compute_angular_span(points: np.ndarray) -> float:
    """Computes the angular span of points as seen from the origin (in degrees)."""
    angles = np.arctan2(points[:, 0], points[:, 1])
    span = angles.max() - angles.min()
    if span > np.pi:
        angles_shifted = angles % (2 * np.pi)
        span = angles_shifted.max() - angles_shifted.min()
        if span > np.pi:
            span = 2 * np.pi - span

    return float(np.degrees(span))
