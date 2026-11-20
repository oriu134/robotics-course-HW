"""Focused tests of seeded sampling, graph invariants, and finite budgets."""

import math

from hw02_solution.cspace import edge_is_free, inflate, point_is_free
from hw02_solution.grid import GridMap
from hw02_solution.roadmap import build_prm
import pytest


def test_seeded_graph_is_reproducible_safe_and_undirected():
    space = inflate(GridMap(8, 8, 0.25, 0.0, 0.0, (0,) * 64))
    start, goal = (0.625, 0.625), (1.375, 1.375)
    options = {'seed': 2, 'samples': 40, 'neighbors': 6, 'max_draws': 1000}
    graph = build_prm(space, start, goal, **options)
    assert graph == build_prm(space, start, goal, **options)
    assert graph.points != build_prm(
        space, start, goal, seed=3, samples=40, neighbors=6, max_draws=1000).points
    assert graph.points[:2] == (start, goal)
    assert len(set(graph.points)) == len(graph.points) == 42
    assert graph.draws <= 1000
    assert graph.candidate_edges <= 42 * 6
    assert graph.edge_count > 0
    for index, point in enumerate(graph.points):
        assert point_is_free(space, point)
        assert len({other for other, _ in graph.adjacency[index]}) == len(graph.adjacency[index])
        for other, cost in graph.adjacency[index]:
            assert other != index
            assert cost == pytest.approx(math.dist(point, graph.points[other]))
            assert (index, cost) in graph.adjacency[other]
            assert edge_is_free(space, point, graph.points[other])


def test_disconnected_room_is_not_bridged():
    raw = [0] * 64
    for row in range(8):
        raw[row * 8 + 4] = 100
    space = inflate(GridMap(8, 8, 1.0, 0.0, 0.0, tuple(raw)))
    graph = build_prm(
        space, (1.5, 3.5), (6.5, 3.5), samples=30, neighbors=5, max_draws=500)
    reached, pending = {0}, [0]
    while pending:
        for other, _ in graph.adjacency[pending.pop()]:
            if other not in reached:
                reached.add(other)
                pending.append(other)
    assert 1 not in reached
    for index, edges in enumerate(graph.adjacency):
        for other, _ in edges:
            assert edge_is_free(space, graph.points[index], graph.points[other])


def test_rejection_sampling_stops_at_its_budget():
    cells = [100] * 10000
    cells[101] = cells[9898] = 0
    space = GridMap(100, 100, 1.0, 0.0, 0.0, tuple(cells))
    with pytest.raises(ValueError, match='0/1 accepted after 3 draws'):
        build_prm(space, (1.5, 1.5), (98.5, 98.5), samples=1, max_draws=3)


def test_invalid_settings_and_colliding_endpoints_raise():
    space = inflate(GridMap(8, 8, 0.25, 0.0, 0.0, (0,) * 64))
    start, goal = (0.625, 0.625), (1.375, 1.375)
    for options in ({'samples': -1}, {'neighbors': 0}, {'seed': -1},
                    {'max_draws': math.inf}):
        with pytest.raises(ValueError, match='settings'):
            build_prm(space, start, goal, **options)
    with pytest.raises(ValueError, match='start'):
        build_prm(space, (0.125, 0.125), goal)
    with pytest.raises(ValueError, match='outside'):
        build_prm(space, start, (2.0, 1.0))


def test_zero_samples_and_coincident_endpoints():
    space = inflate(GridMap(8, 8, 0.25, 0.0, 0.0, (0,) * 64))
    point = (0.625, 0.625)
    graph = build_prm(space, point, point, samples=0, neighbors=1, max_draws=0)
    assert graph.points == (point, point)
    assert graph.adjacency == (((1, 0.0),), ((0, 0.0),))
    assert graph.draws == 0 and graph.edge_count == 1
