"""Unit tests for the log-odds grid (no ROS needed)."""

import math

from reactive_tb4.occupancy_grid import LogOddsGrid, pose_compose


def test_wall_hit_and_free_space():
    """A beam straight along +x marks free cells then an occupied cell."""
    grid = LogOddsGrid(4.0, 4.0, 0.1, -2.0, -2.0)
    for _ in range(5):
        grid.update_from_scan(0.0, 0.0, 0.0, [1.0], 0.0, 0.0, 0.1, 10.0)
    values = grid.to_occupancy_values()
    col, row = grid.world_to_cell(1.0, 0.0)
    assert values[row, col] > 50
    col, row = grid.world_to_cell(0.5, 0.0)
    assert values[row, col] < 50
    col, row = grid.world_to_cell(-1.0, -1.0)
    assert values[row, col] == -1


def test_pose_compose():
    """Rotating by 90 deg maps a +x offset to +y."""
    x, y, yaw = pose_compose(1.0, 0.0, math.pi / 2, 0.5, 0.0, 0.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 0.5) < 1e-9
