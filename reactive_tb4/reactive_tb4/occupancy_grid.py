"""Log-odds occupancy grid used by the mapping behavior.

The grid is a 2D numpy array. Each cell holds the log-odds that it is
occupied: 0 means "no idea" (50/50), positive means probably a wall, negative
means probably open floor. Every laser beam lowers the log-odds of the cells
it passes through (free space) and raises the log-odds of the cell where it
stopped (a hit). Adding log-odds is the standard, cheap way to do a Bayesian
update for each cell independently. No ROS imports here.
"""

import math
from typing import Sequence, Tuple

import numpy as np


class LogOddsGrid:
    """A fixed-size occupancy grid in the odom frame."""

    def __init__(self, width_m: float, height_m: float, resolution: float,
                 origin_x: float, origin_y: float, hit_log_odds: float = 0.85,
                 miss_log_odds: float = -0.4, clamp: float = 4.0):
        """Allocate the grid.

        Args:
            width_m: Size of the map along x (m).
            height_m: Size of the map along y (m).
            resolution: Cell edge length (m).
            origin_x: World x of the grid's lower-left corner (m).
            origin_y: World y of the grid's lower-left corner (m).
            hit_log_odds: Amount added to a cell a beam ended in.
            miss_log_odds: Amount added to cells a beam passed through.
            clamp: Log-odds are kept within +/- this so the map can still change.

        """
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self._hit = hit_log_odds
        self._miss = miss_log_odds
        self._clamp = clamp
        # Row index is y, column index is x, matching nav_msgs/OccupancyGrid.
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    def world_to_cell(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Convert world coordinates (m) to integer (column, row) indices."""
        col = np.floor((x - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((y - self.origin_y) / self.resolution).astype(np.int64)
        return col, row

    def _flat_indices(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Return unique flat indices of in-bounds cells containing the points."""
        col, row = self.world_to_cell(x, y)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return np.unique(row[inside] * self.width + col[inside])

    def update_from_scan(self, sensor_x: float, sensor_y: float, sensor_yaw: float,
                         ranges: Sequence[float], angle_min: float,
                         angle_increment: float, range_min: float, range_max: float,
                         max_trace: float = 4.0) -> None:
        """Fold one laser scan into the map.

        Args:
            sensor_x: Lidar x position in the map frame (m).
            sensor_y: Lidar y position in the map frame (m).
            sensor_yaw: Lidar heading in the map frame (rad).
            ranges: Raw ``LaserScan.ranges``.
            angle_min: Angle of the first beam (rad, laser frame).
            angle_increment: Angle between beams (rad).
            range_min: Shorter readings are ignored.
            range_max: Longer readings (or inf) are treated as "no hit".
            max_trace: Free space is only traced this far out, since long
                beams at grazing angles are less trustworthy (m).

        """
        r = np.asarray(ranges, dtype=np.float64)
        angles = sensor_yaw + angle_min + angle_increment * np.arange(r.size)
        valid = np.isfinite(r) & (r >= range_min)
        hit = valid & (r <= range_max) & (r <= max_trace)
        # Beams with no return still tell us the space in front of them is open.
        no_return = ~np.isfinite(r) | (r > range_max)
        trace_len = np.where(hit, r, np.where(no_return, max_trace, 0.0))
        trace_len = np.minimum(trace_len, max_trace)

        # Sample points every half cell along every beam at once (vectorized).
        step = self.resolution * 0.5
        samples = np.arange(0.0, max_trace, step)
        cos_a = np.cos(angles)[:, None]
        sin_a = np.sin(angles)[:, None]
        # Stop one cell short of the hit so the wall cell is not marked free.
        keep = samples[None, :] < (trace_len[:, None] - self.resolution)
        xs = (sensor_x + samples[None, :] * cos_a)[keep]
        ys = (sensor_y + samples[None, :] * sin_a)[keep]
        free = self._flat_indices(xs, ys)

        hx = sensor_x + r[hit] * np.cos(angles[hit])
        hy = sensor_y + r[hit] * np.sin(angles[hit])
        occupied = self._flat_indices(hx, hy)
        free = np.setdiff1d(free, occupied, assume_unique=True)

        flat = self.log_odds.reshape(-1)
        seen = self.observed.reshape(-1)
        flat[free] += self._miss
        flat[occupied] += self._hit
        np.clip(flat, -self._clamp, self._clamp, out=flat)
        seen[free] = True
        seen[occupied] = True

    def to_occupancy_values(self) -> np.ndarray:
        """Return int8 values for nav_msgs/OccupancyGrid (-1 unknown, 0..100)."""
        probability = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        values = np.round(probability * 100.0).astype(np.int8)
        values[~self.observed] = -1
        return values


def pose_compose(x: float, y: float, yaw: float, dx: float, dy: float,
                 dyaw: float) -> Tuple[float, float, float]:
    """Return pose (x, y, yaw) followed by a relative offset (dx, dy, dyaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, yaw + dyaw
