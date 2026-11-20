"""Focused tests of shortest paths, no-path handling, and route validation."""

from dataclasses import replace
import math

from hw02_solution import search
from hw02_solution.cspace import inflate
from hw02_solution.grid import GridMap
from hw02_solution.roadmap import Roadmap
import pytest


def make_graph(points, pairs):
    adjacency = [[] for _ in points]
    for first, last in pairs:
        cost = math.dist(points[first], points[last])
        adjacency[first].append((last, cost))
        adjacency[last].append((first, cost))
    return Roadmap(tuple(points), tuple(tuple(edges) for edges in adjacency), 0, len(pairs))


def test_astar_chooses_shortest_route_not_greedy_direction():
    points = ((1.5, 1.5), (5.5, 1.5), (3.5, 3.5), (2.5, 1.5), (4.5, 1.5))
    graph = make_graph(points, ((0, 2), (2, 1), (0, 3), (3, 4), (4, 1)))
    assert search.a_star(graph) == (0, 3, 4, 1)
    space = inflate(GridMap(8, 8, 1.0, 0.0, 0.0, (0,) * 64))
    result = search.plan_route(space, graph)
    assert result is not None
    assert result.points == tuple(points[index] for index in (0, 3, 4, 1))
    assert result.length == 4.0


def test_disconnected_and_identical_endpoints():
    point = (1.5, 1.5)
    graph = make_graph((point, (5.5, 1.5)), ())
    space = inflate(GridMap(8, 8, 1.0, 0.0, 0.0, (0,) * 64))
    assert search.a_star(graph) is None
    assert search.plan_route(space, graph) is None
    assert search.a_star(graph, start=0, goal=0) == (0,)
    graph = make_graph((point, point), ((0, 1),))
    result = search.plan_route(space, graph)
    assert result is not None and result.length == 0.0
    assert result.points[0] == result.points[-1] == point


def test_invalid_graph_or_endpoint_indices_raise():
    graph = make_graph(((1.5, 1.5), (5.5, 1.5)), ((0, 1),))
    for options in ({'start': -1}, {'goal': 2}, {'start': 0.5}):
        with pytest.raises(ValueError, match='indices'):
            search.a_star(graph, **options)
    with pytest.raises(ValueError, match='adjacency'):
        search.a_star(replace(graph, adjacency=()))
    with pytest.raises(ValueError, match='Euclidean'):
        search.a_star(replace(graph, adjacency=(((1, 0.1),), ((0, 0.1),))))
    space = inflate(GridMap(8, 8, 1.0, 0.0, 0.0, (0,) * 64))
    with pytest.raises(ValueError, match='start/goal'):
        search.plan_route(space, make_graph(((0.5, 0.5), (5.5, 1.5)), ()))
    with pytest.raises(ValueError, match='vertices'):
        search.plan_route(space, make_graph(((1.5, 1.5),), ()))


def test_validation_rejects_colliding_incomplete_or_fabricated_routes(monkeypatch):
    points = ((1.5, 1.5), (5.5, 1.5), (3.5, 3.5))
    graph = make_graph(points, ((0, 1),))
    cells = [0] * 64
    cells[1 * 8 + 3] = 100
    space = GridMap(8, 8, 1.0, 0.0, 0.0, tuple(cells))
    with pytest.raises(ValueError, match='colliding segment'):
        search.plan_route(space, graph)
    monkeypatch.setattr(search, 'a_star', lambda graph: (0,))
    with pytest.raises(ValueError, match='complete'):
        search.plan_route(space, graph)
    monkeypatch.setattr(search, 'a_star', lambda graph: (0, 2, 1))
    with pytest.raises(ValueError, match='not a roadmap edge'):
        search.plan_route(space, graph)
