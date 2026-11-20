"""Seeded, bounded PRM construction; no route search or robot movement."""

from dataclasses import dataclass
import math
import random

from scipy.spatial import cKDTree

from .cspace import edge_is_free, Point, point_is_free
from .grid import GridMap


DEFAULT_SAMPLES = 2000
DEFAULT_NEIGHBORS = 30
DEFAULT_MAX_DRAWS = 20000


@dataclass(frozen=True)
class Roadmap:
    """Store vertices and symmetric (neighbor index, Euclidean cost) adjacency."""

    points: tuple[Point, ...]
    adjacency: tuple[tuple[tuple[int, float], ...], ...]
    draws: int
    candidate_edges: int

    @property
    def edge_count(self) -> int:
        """Count undirected edges once."""
        return sum(len(neighbors) for neighbors in self.adjacency) // 2


def build_prm(
    grid: GridMap, start: Point, goal: Point, *, seed: int = 0,
    samples: int = DEFAULT_SAMPLES, neighbors: int = DEFAULT_NEIGHBORS,
    max_draws: int = DEFAULT_MAX_DRAWS,
) -> Roadmap:
    """Sample safe positions and connect k-nearest candidates after full checks."""
    settings = (seed, samples, neighbors, max_draws)
    if any(type(value) is not int or value < 0 for value in settings) or neighbors == 0:
        raise ValueError('PRM settings must be nonnegative integers; neighbors must be positive')
    for name, point in (('start', start), ('goal', goal)):
        if not point_is_free(grid, point):
            raise ValueError(f'PRM {name} collides with configuration-space obstacles')
    points = [start, goal]
    seen = {start, goal}
    rng = random.Random(seed)
    upper_x = math.nextafter(grid.origin_x + grid.width * grid.resolution, grid.origin_x)
    upper_y = math.nextafter(grid.origin_y + grid.height * grid.resolution, grid.origin_y)
    draws = 0
    while len(points) < samples + 2 and draws < max_draws:
        point = (rng.uniform(grid.origin_x, upper_x), rng.uniform(grid.origin_y, upper_y))
        draws += 1
        if point not in seen and point_is_free(grid, point):
            points.append(point)
            seen.add(point)
    if len(points) != samples + 2:
        raise ValueError(
            f'Sampling budget exhausted: {len(points) - 2}/{samples} accepted '
            f'after {draws} draws')
    tree = cKDTree(points)
    _, nearest = tree.query(points, k=min(neighbors + 1, len(points)))
    adjacency: list[list[tuple[int, float]]] = [[] for _ in points]
    checked: set[tuple[int, int]] = set()
    for index, candidates in enumerate(nearest):
        for candidate in candidates:
            other = int(candidate)
            if other == index:
                continue
            pair = (min(index, other), max(index, other))
            if pair in checked:
                continue
            checked.add(pair)
            if edge_is_free(grid, points[index], points[other]):
                cost = math.dist(points[index], points[other])
                adjacency[index].append((other, cost))
                adjacency[other].append((index, cost))
    return Roadmap(
        tuple(points), tuple(tuple(sorted(edges)) for edges in adjacency),
        draws, len(checked))
