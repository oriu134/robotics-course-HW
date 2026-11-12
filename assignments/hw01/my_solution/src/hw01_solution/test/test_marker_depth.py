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
