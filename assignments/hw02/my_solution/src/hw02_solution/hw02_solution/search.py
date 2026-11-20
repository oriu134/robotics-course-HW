"""A* on a Euclidean-weighted PRM and complete collision-checked routes."""

from dataclasses import dataclass
import heapq
import math

from .cspace import edge_is_free, Point, point_is_free
from .grid import GridMap
from .roadmap import Roadmap


@dataclass(frozen=True)
class Route:
    """Store ordered start-to-goal positions and total length in metres."""

    points: tuple[Point, ...]
    length: float


def a_star(graph: Roadmap, start: int = 0, goal: int = 1) -> tuple[int, ...] | None:
    """Return a shortest vertex sequence, or None when the goal is disconnected."""
    count = len(graph.points)
    if (len(graph.adjacency) != count
            or any(type(index) is not int or not 0 <= index < count for index in (start, goal))):
        raise ValueError('A* requires valid start/goal indices and matching adjacency')
    if any(len(point) != 2 or not all(math.isfinite(v) for v in point)
           for point in graph.points):
        raise ValueError('Roadmap positions must be finite 2D coordinates')
    heuristic = tuple(math.dist(point, graph.points[goal]) for point in graph.points)
    if not all(math.isfinite(value) for value in heuristic):
        raise ValueError('Roadmap distances must be finite')
    for index, edges in enumerate(graph.adjacency):
        for other, cost in edges:
            if type(other) is not int or not 0 <= other < count:
                raise ValueError('Roadmap contains an invalid neighbor index')
            if (not math.isfinite(cost)
                    or cost != math.dist(graph.points[index], graph.points[other])):
                raise ValueError('Roadmap edge costs must be finite Euclidean lengths')
    best = {start: 0.0}
    previous: dict[int, int] = {}
    frontier = [(heuristic[start], 0.0, start)]
    while frontier:
        _, travelled, current = heapq.heappop(frontier)
        if travelled != best[current]:
            continue
        if current == goal:
            route = [goal]
            while route[-1] != start:
                route.append(previous[route[-1]])
            return tuple(reversed(route))
        for other, cost in graph.adjacency[current]:
            candidate = travelled + cost
            if not math.isfinite(candidate):
                raise ValueError('Route cost must remain finite')
            if candidate < best.get(other, math.inf):
                best[other] = candidate
                previous[other] = current
                heapq.heappush(frontier, (candidate + heuristic[other], candidate, other))
    return None


def plan_route(grid: GridMap, graph: Roadmap) -> Route | None:
    """Search vertices 0 to 1 and independently validate the complete result."""
    if len(graph.points) < 2:
        raise ValueError('Route planning requires start and goal vertices')
    if not all(point_is_free(grid, point) for point in graph.points[:2]):
        raise ValueError('Route start/goal must be collision-free')
    indices = a_star(graph)
    if indices is None:
        return None
    if (not indices or indices[0] != 0 or indices[-1] != 1
            or any(type(index) is not int or not 0 <= index < len(graph.points)
                   for index in indices)):
        raise ValueError('A* did not return a complete start-to-goal route')
    points = tuple(graph.points[index] for index in indices)
    if not all(point_is_free(grid, point) for point in points):
        raise ValueError('Route contains a colliding position')
    for first, last in zip(indices, indices[1:]):
        if not any(other == last for other, _ in graph.adjacency[first]):
            raise ValueError('Route contains a connection that is not a roadmap edge')
        if not edge_is_free(grid, graph.points[first], graph.points[last]):
            raise ValueError('Route contains a colliding segment')
    length = math.fsum(math.dist(first, last) for first, last in zip(points, points[1:]))
    return Route(points, length)
