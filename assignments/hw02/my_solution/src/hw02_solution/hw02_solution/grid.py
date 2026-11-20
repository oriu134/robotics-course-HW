"""ROS-independent occupancy-grid metadata and coordinate conversion."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GridMap:
    """Store an axis-aligned occupancy grid in row-major order."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Map dimensions must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('Map resolution must be finite and positive')
        if not all(math.isfinite(v) for v in (self.origin_x, self.origin_y)):
            raise ValueError('Map origin must be finite')
        if len(self.data) != self.width * self.height:
            raise ValueError('Map data length does not match its dimensions')
        if any(value < -1 or value > 100 for value in self.data):
            raise ValueError('Occupancy values must be between -1 and 100')

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Return (row, column); reject nonfinite or out-of-map coordinates."""
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('Target coordinates must be finite')
        if not (self.origin_x <= x < self.origin_x + self.width * self.resolution
                and self.origin_y <= y < self.origin_y + self.height * self.resolution):
            raise ValueError(f'Point ({x}, {y}) is outside the map')
        # Division can round an already bounds-checked point up to width/height.
        return (min(self.height - 1, math.floor((y - self.origin_y) / self.resolution)),
                min(self.width - 1, math.floor((x - self.origin_x) / self.resolution)))

    def grid_to_world(self, row: int, column: int) -> tuple[float, float]:
        """Return the world coordinates of a cell's center."""
        if not (0 <= row < self.height and 0 <= column < self.width):
            raise ValueError(f'Cell ({row}, {column}) is outside the map')
        return (self.origin_x + (column + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)
