"""Fonctions geometriques pour la detection d'obstacles."""

import numpy as np

from lidar.detection.types import BoundingBox, LineSegment


def fit_line_segment(points: np.ndarray) -> LineSegment:
    """Ajuste un segment de droite sur un ensemble de points via PCA."""
    centroid = points.mean(axis=0)
    centered = points - centroid

    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    direction = eigenvectors[:, 1]
    normal = eigenvectors[:, 0]

    projections = centered @ direction
    t_min, t_max = projections.min(), projections.max()

    start = centroid + t_min * direction
    end = centroid + t_max * direction
    length_mm = float(t_max - t_min)

    errors = centered @ normal
    fit_error_mm = float(np.sqrt(np.mean(errors ** 2)))

    return LineSegment(
        start=start,
        end=end,
        direction=direction,
        normal=normal,
        length_mm=length_mm,
        fit_error_mm=fit_error_mm,
    )


def compute_bounding_box(points: np.ndarray) -> BoundingBox:
    """Calcule la boite englobante alignee sur les axes."""
    return BoundingBox(
        x_min=float(points[:, 0].min()),
        x_max=float(points[:, 0].max()),
        y_min=float(points[:, 1].min()),
        y_max=float(points[:, 1].max()),
    )


def compute_angular_span(points: np.ndarray) -> float:
    """Calcule l'etendue angulaire des points vus depuis l'origine (en degres)."""
    angles = np.arctan2(points[:, 0], points[:, 1])
    angle_min = angles.min()
    angle_max = angles.max()

    span = angle_max - angle_min
    if span > np.pi:
        angles_shifted = angles % (2 * np.pi)
        span = angles_shifted.max() - angles_shifted.min()
        if span > np.pi:
            span = 2 * np.pi - span

    return float(np.degrees(span))


def linearity_ratio(points: np.ndarray) -> float:
    """Ratio de linearite base sur PCA (0=disque, 1=ligne)."""
    if len(points) < 2:
        return 0.0

    centered = points - points.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues = np.linalg.eigvalsh(cov)

    total = eigenvalues.sum()
    if total == 0:
        return 0.0

    return float(eigenvalues[1] / total)
