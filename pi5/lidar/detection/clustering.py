"""Density-based clustering (DBSCAN) in plain numpy."""

import numpy as np


def dbscan(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """Labels an (N, 2) point set like sklearn.cluster.DBSCAN(eps, min_samples).fit_predict.

    A point is a core point when at least min_samples points (itself included)
    lie within eps of it. Clusters are numbered 0, 1, ... in the order of their
    first core point; a border point reachable from two clusters stays in the
    first one. Noise is labelled -1.

    A lidar revolution is a few hundred points, so a dense distance matrix
    (a few MB) is cheaper than a tree search — and keeps scikit-learn and
    scipy, heavy to install and to import on the Pi, out of the robot.
    """
    n = len(points)
    labels = np.full(n, -1, dtype=np.intp)
    if n == 0:
        return labels

    d2 = np.subtract.outer(points[:, 0], points[:, 0])
    d2 *= d2
    dy2 = np.subtract.outer(points[:, 1], points[:, 1])
    dy2 *= dy2
    d2 += dy2
    neighbors = d2 <= eps * eps
    is_core = neighbors.sum(axis=1) >= min_samples

    cluster = 0
    for seed in np.flatnonzero(is_core):
        if labels[seed] != -1:
            continue
        labels[seed] = cluster
        frontier = [seed]
        while frontier:
            reached = np.flatnonzero(neighbors[frontier.pop()] & (labels == -1))
            labels[reached] = cluster
            # Only core points extend the cluster; border points just join it
            frontier.extend(reached[is_core[reached]])
        cluster += 1
    return labels
