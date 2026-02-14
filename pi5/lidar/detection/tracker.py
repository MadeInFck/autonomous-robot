"""Suivi temporel des obstacles entre frames successifs."""

from dataclasses import dataclass, field

import numpy as np

from lidar.detection.types import BoundingBox, DetectionResult, LineSegment, Obstacle, ObstacleType


@dataclass
class TrackedObstacle:
    """Un obstacle suivi dans le temps."""
    track_id: int
    obstacle: Obstacle
    hits: int = 1
    age: int = 0
    _centroid_smooth: np.ndarray = field(default=None, repr=False)
    _bbox_smooth: BoundingBox = field(default=None, repr=False)

    def __post_init__(self):
        if self._centroid_smooth is None:
            self._centroid_smooth = self.obstacle.centroid.copy()
        if self._bbox_smooth is None:
            self._bbox_smooth = BoundingBox(
                self.obstacle.bbox.x_min, self.obstacle.bbox.x_max,
                self.obstacle.bbox.y_min, self.obstacle.bbox.y_max,
            )


class ObstacleTracker:
    """Associe les obstacles entre frames pour un affichage stable.

    Pipeline par frame :
      1. Association par proximite centroide (greedy, plus proche d'abord)
      2. Mise a jour EMA des tracks associes
      3. Nouveaux tracks pour les obstacles non associes
      4. Suppression des tracks non vus depuis max_age frames
      5. Retour des tracks confirmes (hits >= min_hits)
    """

    def __init__(
        self,
        match_radius_mm: float = 300,
        max_age: int = 10,
        min_hits: int = 2,
        smoothing: float = 0.3,
    ):
        self.match_radius_mm = match_radius_mm
        self.max_age = max_age
        self.min_hits = min_hits
        self.smoothing = smoothing

        self._tracks: list[TrackedObstacle] = []
        self._next_id = 0

    def update(self, result: DetectionResult) -> DetectionResult:
        """Met a jour les tracks avec un nouveau DetectionResult."""
        new_obstacles = result.obstacles
        matched_track_indices: set[int] = set()
        matched_obs_indices: set[int] = set()

        if self._tracks and new_obstacles:
            track_centroids = np.array([t._centroid_smooth for t in self._tracks])
            obs_centroids = np.array([o.centroid for o in new_obstacles])
            dists = np.linalg.norm(
                track_centroids[:, None, :] - obs_centroids[None, :, :], axis=2
            )

            flat_indices = np.argsort(dists, axis=None)
            for flat_idx in flat_indices:
                ti = int(flat_idx // len(new_obstacles))
                oi = int(flat_idx % len(new_obstacles))
                if ti in matched_track_indices or oi in matched_obs_indices:
                    continue
                if dists[ti, oi] > self.match_radius_mm:
                    break
                self._update_track(self._tracks[ti], new_obstacles[oi])
                matched_track_indices.add(ti)
                matched_obs_indices.add(oi)

        for oi, obs in enumerate(new_obstacles):
            if oi not in matched_obs_indices:
                self._create_track(obs)

        for ti, track in enumerate(self._tracks):
            if ti not in matched_track_indices:
                track.age += 1

        self._tracks = [t for t in self._tracks if t.age <= self.max_age]

        confirmed = [
            t.obstacle for t in self._tracks if t.hits >= self.min_hits
        ]
        confirmed.sort(key=lambda o: o.nearest_distance_mm)

        for i, obs in enumerate(confirmed):
            obs.id = i

        return DetectionResult(
            obstacles=confirmed,
            noise_points=result.noise_points,
            scan_timestamp=result.scan_timestamp,
            processing_time_ms=result.processing_time_ms,
        )

    def _create_track(self, obs: Obstacle):
        track = TrackedObstacle(
            track_id=self._next_id,
            obstacle=obs,
        )
        self._next_id += 1
        self._tracks.append(track)

    def _update_track(self, track: TrackedObstacle, obs: Obstacle):
        a = self.smoothing

        track._centroid_smooth = (1 - a) * track._centroid_smooth + a * obs.centroid

        old_bb = track._bbox_smooth
        new_bb = obs.bbox
        track._bbox_smooth = BoundingBox(
            x_min=(1 - a) * old_bb.x_min + a * new_bb.x_min,
            x_max=(1 - a) * old_bb.x_max + a * new_bb.x_max,
            y_min=(1 - a) * old_bb.y_min + a * new_bb.y_min,
            y_max=(1 - a) * old_bb.y_max + a * new_bb.y_max,
        )

        nearest_point = (1 - a) * track.obstacle.nearest_point + a * obs.nearest_point
        nearest_distance = float(np.linalg.norm(nearest_point))

        line_seg = obs.line_segment
        if line_seg and track.obstacle.line_segment:
            old_seg = track.obstacle.line_segment
            line_seg = LineSegment(
                start=(1 - a) * old_seg.start + a * line_seg.start,
                end=(1 - a) * old_seg.end + a * line_seg.end,
                direction=line_seg.direction,
                normal=line_seg.normal,
                length_mm=(1 - a) * old_seg.length_mm + a * line_seg.length_mm,
                fit_error_mm=(1 - a) * old_seg.fit_error_mm + a * line_seg.fit_error_mm,
            )

        obstacle_type = obs.type

        track.obstacle = Obstacle(
            id=track.track_id,
            type=obstacle_type,
            points_xy=obs.points_xy,
            centroid=track._centroid_smooth.copy(),
            nearest_distance_mm=nearest_distance,
            nearest_point=nearest_point,
            bbox=BoundingBox(
                track._bbox_smooth.x_min, track._bbox_smooth.x_max,
                track._bbox_smooth.y_min, track._bbox_smooth.y_max,
            ),
            line_segment=line_seg,
            num_points=obs.num_points,
            angular_span_deg=(1 - a) * track.obstacle.angular_span_deg + a * obs.angular_span_deg,
        )

        track.hits += 1
        track.age = 0

    def reset(self):
        self._tracks.clear()
        self._next_id = 0
