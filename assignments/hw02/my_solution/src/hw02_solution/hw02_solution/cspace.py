"""Conservative configuration space and full-segment collision checks."""

from dataclasses import replace
import math

import numpy as np
from scipy.ndimage import binary_dilation

from .grid import GridMap


PLANNING_RADIUS = 0.30
Point = tuple[float, float]


def inflate(grid: GridMap) -> GridMap:
    """Block every cell whose area touches a radius-expanded obstacle/exterior."""
    limit = math.nextafter(PLANNING_RADIUS, math.inf)
    if min(grid.width, grid.height) * grid.resolution <= 2 * limit:
        return replace(grid, data=(100,) * len(grid.data))
    reach = math.ceil(PLANNING_RADIUS / grid.resolution) + 1
    offsets = np.arange(-reach, reach + 1)
    # Distance between closed cell squares, not between their centers.
    gaps = np.maximum(np.abs(offsets) - 1, 0) * grid.resolution
    kernel = np.hypot(gaps[:, None], gaps[None, :]) <= limit
    occupied = np.asarray(grid.data).reshape(grid.height, grid.width) != 0
    blocked = binary_dilation(occupied, structure=kernel)
    rows = np.arange(grid.height)
    columns = np.arange(grid.width)
    wall_rows = np.minimum(rows, grid.height - 1 - rows) * grid.resolution <= limit
    wall_columns = np.minimum(columns, grid.width - 1 - columns) * grid.resolution <= limit
    blocked |= wall_rows[:, None] | wall_columns[None, :]
    return replace(grid, data=tuple(np.where(blocked, 100, 0).ravel().tolist()))


def _touches_cell(start: Point, end: Point, lower: Point, upper: Point) -> bool:
    """Intersect the complete segment with a closed cell using slab intervals."""
    enter, leave = 0.0, 1.0
    for first, last, low, high in zip(start, end, lower, upper):
        delta = last - first
        if delta == 0:
            if first < low or first > high:
                return False
        else:
            near, far = sorted(((low - first) / delta, (high - first) / delta))
            enter, leave = max(enter, near), min(leave, far)
            if enter > math.nextafter(leave, math.inf):
                return False
    return True


def edge_is_free(grid: GridMap, start: Point, end: Point) -> bool:
    """Test a closed segment on an inflated grid; invalid coordinates raise."""
    start_row, start_column = grid.world_to_grid(*start)
    end_row, end_column = grid.world_to_grid(*end)
    if any(x == grid.origin_x or y == grid.origin_y for x, y in (start, end)):
        return False
    # Include neighbors so coordinate roundoff cannot hide grid-line contacts.
    left = max(0, min(start_column, end_column) - 1)
    right = min(grid.width - 1, max(start_column, end_column) + 1)
    bottom = max(0, min(start_row, end_row) - 1)
    top = min(grid.height - 1, max(start_row, end_row) + 1)
    for row in range(bottom, top + 1):
        for column in range(left, right + 1):
            if grid.data[row * grid.width + column] == 0:
                continue
            lower = (grid.origin_x + column * grid.resolution,
                     grid.origin_y + row * grid.resolution)
            upper = (grid.origin_x + (column + 1) * grid.resolution,
                     grid.origin_y + (row + 1) * grid.resolution)
            if _touches_cell(start, end, lower, upper):
                return False
    return True


def point_is_free(grid: GridMap, point: Point) -> bool:
    """Apply the same closed-cell rules to a zero-length segment."""
    return edge_is_free(grid, point, point)
