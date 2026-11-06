"""Check selected-region depth sampling without ROS execution or movement."""

from hw01_solution.camera_listener import measure_marker_depth
import numpy as np
import pytest


def test_only_selected_marker_pixels_contribute_to_depth():
    """Exclude background even when it has finite in-range depth."""
    depths = np.full((4, 4), 2.5, dtype=np.float32)
    depths[1:3, 1:3] = 0.94
    mask = np.zeros((4, 4), dtype=np.uint8)
    mask[1:3, 1:3] = 255
    before = depths.copy()
    distance, reason = measure_marker_depth(depths, mask)
    assert distance == pytest.approx(0.94)
    assert 'accepted' in reason
    assert np.array_equal(depths, before)


def test_median_limits_an_isolated_valid_outlier():
    """Do not let one distant sample dominate the representative depth."""
    depths = np.array([[0.94, 0.94, 2.5]], dtype=np.float32)
    mask = np.full(depths.shape, 255, dtype=np.uint8)
    distance, reason = measure_marker_depth(depths, mask)
    assert distance == pytest.approx(0.94), reason


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf, 0.0, -1.0, 0.01, 3.1])
def test_invalid_depth_values_cannot_become_a_measurement(value):
    """Reject nonfinite, nonpositive and out-of-range sensor samples."""
    depths = np.array([[value]], dtype=np.float32)
    mask = np.full(depths.shape, 255, dtype=np.uint8)
    distance, reason = measure_marker_depth(depths, mask)
    assert distance is None
    assert 'No valid' in reason


def test_camera_clipping_boundaries_are_inclusive():
    """Retain finite samples exactly at the sensor's valid range boundaries."""
    depths = np.array([[0.05, 3.0]], dtype=np.float32)
    distance, reason = measure_marker_depth(
        depths, np.full(depths.shape, 255, dtype=np.uint8)
    )
    assert distance == pytest.approx(1.525), reason


@pytest.mark.parametrize(('depths', 'accepted'), [
    ([0.9, 1.1, np.nan, 0.0], True),
    ([1.0, np.nan, np.nan, 0.0], False),
])
def test_at_least_half_the_selected_region_must_have_valid_depth(depths, accepted):
    """Apply the validity-fraction boundary without trusting a lone sample."""
    pixels = np.array([depths], dtype=np.float32)
    mask = np.full(pixels.shape, 255, dtype=np.uint8)
    distance, reason = measure_marker_depth(pixels, mask)
    assert (distance is not None) == accepted
    if accepted:
        assert distance == pytest.approx(1.0)
    else:
        assert 'Less than half' in reason


@pytest.mark.parametrize(('depths', 'mask'), [
    (np.ones((2, 2), dtype=np.float64), np.full((2, 2), 255, dtype=np.uint8)),
    (np.ones((2, 2), dtype=np.float32), np.full((1, 2), 255, dtype=np.uint8)),
    (np.ones((2, 2), dtype=np.float32), np.full((2, 2), 255, dtype=np.float32)),
    (np.ones((2, 2), dtype=np.float32), np.full((2, 2), 128, dtype=np.uint8)),
    (np.ones((2, 2), dtype=np.float32), np.zeros((2, 2), dtype=np.uint8)),
    (np.zeros((0, 2), dtype=np.float32), np.zeros((0, 2), dtype=np.uint8)),
])
def test_malformed_arrays_raise_explicit_errors(depths, mask):
    """Keep malformed input distinct from missing depth in a valid image."""
    with pytest.raises(ValueError):
        measure_marker_depth(depths, mask)
