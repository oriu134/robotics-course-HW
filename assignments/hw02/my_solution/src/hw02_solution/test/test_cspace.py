"""Geometry tests independent of ROS and Gazebo."""

import math

from hw02_solution.cspace import edge_is_free, inflate, PLANNING_RADIUS, point_is_free
from hw02_solution.grid import GridMap
import pytest


def make_grid(width=25, height=25, resolution=0.1, obstacles=()):
    data = [0] * (width * height)
    for row, column, value in obstacles:
        data[row * width + column] = value
    return GridMap(width, height, resolution, 0.0, 0.0, tuple(data))


@pytest.mark.parametrize('value', [100, -1, 1])
def test_inflation_accounts_for_cell_area_and_unknowns(value):
    raw = make_grid(obstacles=[(12, 12, value)])
    space = inflate(raw)
    assert PLANNING_RADIUS == 0.30
    assert space.data[12 * 25 + 16] == 100
    assert space.data[15 * 25 + 15] == 100
    assert space.data[14 * 25 + 16] == 0
    assert space.data[12 * 25 + 17] == 0
    assert raw.data[12 * 25 + 16] == 0
    assert (space.width, space.height, space.resolution) == (25, 25, 0.1)


def test_exterior_is_inflated_without_occupied_boundary_cells():
    space = inflate(make_grid())
    for index in (0, 3, 21, 24):
        assert space.data[index * 25 + 12] == 100
        assert space.data[12 * 25 + index] == 100
    assert point_is_free(space, (0.45, 0.45))
    assert not point_is_free(space, (0.35, 1.25))


def test_robot_cannot_fit_in_a_tiny_map():
    space = inflate(make_grid(5, 5))
    assert set(space.data) == {100}


@pytest.mark.parametrize('right_wall, expected', [(13, False), (17, True)])
def test_narrow_gap_closes_while_wide_gap_stays_open(right_wall, expected):
    obstacles = [(row, column, 100) for row in range(25) for column in (7, right_wall)]
    space = inflate(make_grid(obstacles=obstacles))
    middle = (8 + right_wall) * 0.1 / 2
    assert edge_is_free(space, (middle, 0.65), (middle, 1.85)) is expected


def test_inflation_matches_exhaustive_square_distance_oracle():
    raw = make_grid(17, 15, 0.125, [(7, 8, 100), (5, 10, -1), (10, 4, 1)])
    space = inflate(raw)
    for row in range(raw.height):
        for column in range(raw.width):
            left, bottom = column * raw.resolution, row * raw.resolution
            right, top = left + raw.resolution, bottom + raw.resolution
            clearance = min(left, bottom, raw.width * raw.resolution - right,
                            raw.height * raw.resolution - top)
            for other_row in range(raw.height):
                for other_column in range(raw.width):
                    if raw.data[other_row * raw.width + other_column] == 0:
                        continue
                    x, y = other_column * raw.resolution, other_row * raw.resolution
                    dx = max(x - right, left - (x + raw.resolution), 0)
                    dy = max(y - top, bottom - (y + raw.resolution), 0)
                    clearance = min(clearance, math.hypot(dx, dy))
            assert (space.data[row * raw.width + column] != 0) == (clearance <= 0.30)


@pytest.mark.parametrize('start, end, expected', [
    ((0.5, 2.5), (4.5, 2.5), False),
    ((0.5, 0.5), (4.5, 4.5), False),
    ((0.5, 3.5), (3.5, 0.5), False),
    ((2.0, 0.5), (2.0, 4.5), False),
    ((0.5, 3.0), (4.5, 3.0), False),
    ((0.5, 1.5), (2.0, 2.0), False),
    ((0.5, 1.5), (4.5, 1.5), True),
])
def test_entire_segment_including_grazing_and_grid_lines(start, end, expected):
    space = make_grid(6, 6, 1.0, [(2, 2, 100)])
    assert edge_is_free(space, start, end) is expected
    assert edge_is_free(space, end, start) is expected


@pytest.mark.parametrize('point, expected', [
    ((2.5, 2.5), False), ((2.0, 2.0), False), ((3.0, 3.0), False),
    ((1.5, 1.5), True), ((0.0, 3.5), False),
])
def test_points_and_zero_length_edges_use_closed_cells(point, expected):
    space = make_grid(6, 6, 1.0, [(2, 2, -1)])
    assert point_is_free(space, point) is expected
    assert edge_is_free(space, point, point) is expected


def test_free_endpoints_do_not_hide_a_thin_obstacle():
    space = make_grid(320, 3, 0.025, [(1, 167, 100)])
    start, end = (0.0125, 0.0375), (7.9875, 0.0375)
    assert point_is_free(space, start) and point_is_free(space, end)
    assert not edge_is_free(space, start, end)


@pytest.mark.parametrize('resolution, origin', [
    (0.1, (1.2, -2.3)), (0.02500000037252903, (0.0, 0.0)),
])
def test_closed_contacts_survive_world_grid_roundoff(resolution, origin):
    base = make_grid(6, 6, resolution, [(2, 2, 100)])
    space = GridMap(6, 6, resolution, *origin, base.data)
    for offset in (2, 3):
        point = (origin[0] + offset * resolution, origin[1] + offset * resolution)
        assert not point_is_free(space, point)
    start = (origin[0] + 0.5 * resolution, origin[1] + 3 * resolution)
    end = (origin[0] + 4.5 * resolution, origin[1] + 3 * resolution)
    assert not edge_is_free(space, start, end)
    assert not edge_is_free(space, end, start)


@pytest.mark.parametrize('point', [
    (-0.1, 1.0), (2.5, 1.0), (math.nan, 1.0), (1.0, math.inf),
])
def test_invalid_edge_coordinates_raise_explicitly(point):
    space = inflate(make_grid())
    with pytest.raises(ValueError):
        edge_is_free(space, (1.25, 1.25), point)
