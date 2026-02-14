"""Grille d'occupation log-odds avec ray-tracing Bresenham."""

import numpy as np

from lidar.data import Scan


class OccupancyGrid:
    """Grille d'occupation probabiliste mise a jour par log-odds.

    La grille est centree : l'origine du monde (0, 0) correspond au centre
    de la grille. Chaque cellule stocke un log-odds ratio, mis a jour par
    ray-tracing Bresenham pour chaque mesure LiDAR.

    Taille par defaut : 200x200 cellules a 50mm/cellule = 10m x 10m.
    """

    def __init__(
        self,
        size_mm: float = 10_000,
        resolution_mm: float = 50,
        log_odds_hit: float = 0.9,
        log_odds_miss: float = -0.4,
        log_odds_max: float = 5.0,
        log_odds_min: float = -5.0,
    ):
        self.size_mm = size_mm
        self.resolution_mm = resolution_mm
        self.log_odds_hit = log_odds_hit
        self.log_odds_miss = log_odds_miss
        self.log_odds_max = log_odds_max
        self.log_odds_min = log_odds_min

        self.grid_size = int(size_mm / resolution_mm)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float64)

    def world_to_grid(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """Convertit des coordonnees monde (mm) en indices de grille (row, col)."""
        col = int(x_mm / self.resolution_mm + self.grid_size / 2)
        row = int(-y_mm / self.resolution_mm + self.grid_size / 2)
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convertit des indices de grille en coordonnees monde (mm)."""
        x_mm = (col - self.grid_size / 2) * self.resolution_mm
        y_mm = -(row - self.grid_size / 2) * self.resolution_mm
        return x_mm, y_mm

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.grid_size and 0 <= col < self.grid_size

    def update(
        self,
        scan: Scan,
        min_quality: int = 10,
        origin_x_mm: float = 0.0,
        origin_y_mm: float = 0.0,
    ):
        """Met a jour la grille avec un scan LiDAR."""
        x, y, _ = scan.to_cartesian_arrays(min_quality=min_quality)

        if len(x) == 0:
            return

        origin_row, origin_col = self.world_to_grid(origin_x_mm, origin_y_mm)

        for px, py in zip(x, y):
            point_x = px + origin_x_mm
            point_y = py + origin_y_mm
            end_row, end_col = self.world_to_grid(point_x, point_y)

            cells = self._bresenham(origin_row, origin_col, end_row, end_col)

            for r, c in cells[:-1]:
                if self._in_bounds(r, c):
                    self.grid[r, c] = np.clip(
                        self.grid[r, c] + self.log_odds_miss,
                        self.log_odds_min,
                        self.log_odds_max,
                    )

            if cells:
                r, c = cells[-1]
                if self._in_bounds(r, c):
                    self.grid[r, c] = np.clip(
                        self.grid[r, c] + self.log_odds_hit,
                        self.log_odds_min,
                        self.log_odds_max,
                    )

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
        """Algorithme de Bresenham pour tracer une ligne entre deux cellules."""
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc

        r, c = r0, c0
        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

        return cells

    def to_probability(self) -> np.ndarray:
        """Convertit la grille log-odds en probabilites [0, 1]."""
        return 1.0 / (1.0 + np.exp(-self.grid))

    def to_image_array(self) -> np.ndarray:
        """Convertit la grille en image uint8 : noir=occupe, blanc=libre, gris=inconnu."""
        prob = self.to_probability()
        img = ((1.0 - prob) * 255).astype(np.uint8)
        return img

    def reset(self):
        """Remet la grille a zero."""
        self.grid[:] = 0.0
