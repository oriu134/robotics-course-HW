"""Check candidate red-pixel segmentation without ROS execution or movement."""

from hw01_solution.camera_listener import make_red_mask
import numpy as np
import pytest


def test_red_mask_selects_colors_without_changing_rgb():
    """Produce a binary two-dimensional mask without mutating input pixels."""
    pixels = np.array([[
        [255, 0, 0],
        [170, 40, 30],
        [150, 100, 100],
        [219, 219, 219],
        [0, 0, 255],
        [255, 255, 0],
        [0, 255, 0],
        [0, 0, 0],
        [255, 120, 120],
    ]], dtype=np.uint8)
    original = pixels.copy()
    mask = make_red_mask(pixels)
    assert mask.shape == (1, 9)
    assert mask.dtype == np.uint8
    assert mask.tolist() == [[255, 255, 255, 0, 0, 0, 0, 0, 0]]
    assert set(np.unique(mask).tolist()) == {0, 255}
    assert np.array_equal(pixels, original)


@pytest.mark.parametrize(('color', 'expected'), [
    ((149, 0, 0), 0),
    ((150, 0, 0), 255),
    ((255, 100, 0), 255),
    ((255, 101, 0), 0),
    ((255, 0, 100), 255),
    ((255, 0, 101), 0),
])
def test_red_threshold_boundaries(color, expected):
    """Apply the inclusive threshold values exactly on each RGB channel."""
    pixels = np.array([[color]], dtype=np.uint8)
    assert make_red_mask(pixels).tolist() == [[expected]]
