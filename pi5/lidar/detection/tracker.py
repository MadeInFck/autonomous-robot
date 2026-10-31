"""Temporal tracking of obstacles between successive frames."""

from dataclasses import dataclass

import numpy as np

from lidar.detection.types import BoundingBox, DetectionResult, LineSegment, Obstacle


@dataclass
class TrackedObstacle:
    """An obstacle tracked over time. `obstacle` holds the smoothed state."""
    track_id: int
    obstacle: Obstacle
    hits: int = 1
    age: int = 0


class ObstacleTracker:
    """Associates obstacles between frames for stable display.

    Pipeline per frame:
      1. Association by centroid proximity (greedy, nearest first)
      2. EMA update of associated tracks
      3. New tracks for unassociated obstacles
      4. Removal of tracks unseen for max_age frames
      5. Return confirmed tracks (hits >= min_hits)
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
        """Updates the tracks with a new DetectionResult."""
        new_obstacles = result.obstacles
        matched_track_indices: set[int] = set()
        matched_obs_indices: set[int] = set()

        if self._tracks and new_obstacles:
            track_centroids = np.array([t.obstacle.centroid for t in self._tracks])
            obs_centroids = np.array([o.centroid for o in new_obstacles])
            dists = np.linalg.norm(
                track_centroids[:, None, :] - obs_centroids[None, :, :], axis=2
            )

            for flat_idx in np.argsort(dists, axis=None):
                ti, oi = divmod(int(flat_idx), len(new_obstacles))
                if ti in matched_track_indices or oi in matched_obs_indices:
                    continue
                if dists[ti, oi] > self.match_radius_mm:
                    break
                self._update_track(self._tracks[ti], new_obstacles[oi])
                matched_track_indices.add(ti)
                matched_obs_indices.add(oi)

        for ti, track in enumerate(self._tracks):
            if ti not in matched_track_indices:
                track.age += 1

        for oi, obs in enumerate(new_obstacles):
            if oi not in matched_obs_indices:
                self._tracks.append(TrackedObstacle(track_id=self._next_id, obstacle=obs))
                self._next_id += 1

        self._tracks = [t for t in self._tracks if t.age <= self.max_age]

        confirmed = [t.obstacle for t in self._tracks if t.hits >= self.min_hits]
        confirmed.sort(key=lambda o: o.nearest_distance_mm)
        for i, obs in enumerate(confirmed):
            obs.id = i

        return DetectionResult(
            obstacles=confirmed,
            noise_points=result.noise_points,
            scan_timestamp=result.scan_timestamp,
            processing_time_ms=result.processing_time_ms,
        )

    def _update_track(self, track: TrackedObstacle, obs: Obstacle):
        a = self.smoothing
        old = track.obstacle

        def ema(previous, new):
            return (1 - a) * previous + a * new

        nearest_point = ema(old.nearest_point, obs.nearest_point)

        line_seg = obs.line_segment
        if line_seg and old.line_segment:
            line_seg = LineSegment(
                start=ema(old.line_segment.start, line_seg.start),
                end=ema(old.line_segment.end, line_seg.end),
                direction=line_seg.direction,
                normal=line_seg.normal,
                length_mm=ema(old.line_segment.length_mm, line_seg.length_mm),
                fit_error_mm=ema(old.line_segment.fit_error_mm, line_seg.fit_error_mm),
                linearity=line_seg.linearity,
            )

        track.obstacle = Obstacle(
            id=track.track_id,
            type=obs.type,
            points_xy=obs.points_xy,
            centroid=ema(old.centroid, obs.centroid),
            nearest_distance_mm=float(np.linalg.norm(nearest_point)),
            nearest_point=nearest_point,
            bbox=BoundingBox(
                x_min=ema(old.bbox.x_min, obs.bbox.x_min),
                x_max=ema(old.bbox.x_max, obs.bbox.x_max),
                y_min=ema(old.bbox.y_min, obs.bbox.y_min),
                y_max=ema(old.bbox.y_max, obs.bbox.y_max),
            ),
            line_segment=line_seg,
            num_points=obs.num_points,
            angular_span_deg=ema(old.angular_span_deg, obs.angular_span_deg),
        )
        track.hits += 1
        track.age = 0

    def reset(self):
        self._tracks.clear()
        self._next_id = 0
