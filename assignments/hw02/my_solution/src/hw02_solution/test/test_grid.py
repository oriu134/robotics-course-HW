"""Tests of grid geometry that do not require ROS or Gazebo."""

from dataclasses import replace
import math

from hw02_solution.grid import GridMap
import pytest


@pytest.fixture
def grid():
    return GridMap(4, 3, 0.5, 1.0, -2.0, tuple(range(12)))


def test_row_major_order_and_world_coordinates(grid):
    assert grid.world_to_grid(2.1, -1.4) == (1, 2)
    assert grid.data[1 * grid.width + 2] == 6
    assert grid.grid_to_world(1, 2) == (2.25, -1.25)
    assert grid.world_to_grid(1.0, -2.0) == (0, 0)
    for row in range(grid.height):
        for column in range(grid.width):
            assert grid.world_to_grid(*grid.grid_to_world(row, column)) == (row, column)


@pytest.mark.parametrize('point', [
    (3.0, -1.0), (2.0, -0.5), (0.99, -1.0), (2.0, -2.01),
    (math.nan, -1.0), (2.0, math.inf),
])
def test_invalid_world_coordinates(grid, point):
    with pytest.raises(ValueError):
        grid.world_to_grid(*point)


@pytest.mark.parametrize('cell', [(-1, 0), (0, -1), (3, 0), (0, 4)])
def test_invalid_grid_coordinates(grid, cell):
    with pytest.raises(ValueError):
        grid.grid_to_world(*cell)


@pytest.mark.parametrize('changes', [
    {'width': 0}, {'height': -1}, {'resolution': 0.0},
    {'resolution': math.nan}, {'origin_x': math.inf}, {'origin_y': math.nan},
    {'data': (0,)}, {'data': (-2,) * 12}, {'data': (101,) * 12},
])
def test_invalid_metadata(grid, changes):
    with pytest.raises(ValueError):
        replace(grid, **changes)


def test_unknown_cells_are_preserved(grid):
    assert replace(grid, data=(-1,) * 12).data == (-1,) * 12


def test_roundoff_just_inside_translated_map_edge():
    grid = GridMap(3, 3, 0.5, -2.3, -2.3, (0,) * 9)
    upper = grid.origin_x + grid.width * grid.resolution
    inside = math.nextafter(upper, grid.origin_x)
    assert grid.world_to_grid(inside, inside) == (2, 2)
    with pytest.raises(ValueError):
        grid.world_to_grid(upper, inside)


def test_supplied_map_extents():
    grid = GridMap(320, 240, 0.025, 0.0, 0.0, (0,) * (320 * 240))
    assert grid.world_to_grid(1.0, 3.0) == (120, 40)
    assert grid.world_to_grid(7.0, 3.0) == (120, 280)
    assert grid.world_to_grid(math.nextafter(8.0, 0.0), 3.0) == (120, 319)
    with pytest.raises(ValueError):
        grid.world_to_grid(8.0, 3.0)
