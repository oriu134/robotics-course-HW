"""Check marker-region selection using synthetic image-space masks."""

import cv2
from hw01_solution.camera_listener import detect_marker
import numpy as np
import pytest


def round_mask():
    """Create a round region at an asymmetric column/row location."""
    mask = np.zeros((50, 70), dtype=np.uint8)
    cv2.circle(mask, (35, 20), 6, 255, -1)
    return mask


def test_round_region_center_is_column_then_row_and_input_is_preserved():
    """Return the correct pixel center and selected region without altering input."""
    mask = round_mask()
    original = mask.copy()
    detection, reason = detect_marker(mask)
    assert detection is not None, reason
    assert detection.pixel == pytest.approx((35.0, 20.0))
    assert detection.mask.dtype == np.uint8
    assert np.array_equal(detection.mask, original)
    assert np.array_equal(mask, original)


def test_subpixel_center_is_not_rounded():
    """Preserve a fractional center for later calibrated reconstruction."""
    rows, columns = np.indices((50, 70))
    mask = (((columns - 35.5)**2 + (rows - 20.5)**2) <= 36).astype(np.uint8) * 255
    detection, reason = detect_marker(mask)
    assert detection is not None, reason
    assert detection.pixel == pytest.approx((35.5, 20.5))


def test_small_noise_does_not_shift_center_or_enter_selected_mask():
    """Exclude isolated noise from both the centroid and future depth sampling."""
    clean = round_mask()
    noisy = clean.copy()
    noisy[40, 60] = 255
    detection, reason = detect_marker(noisy)
    assert detection is not None, reason
    assert detection.pixel == pytest.approx((35.0, 20.0))
    assert np.array_equal(detection.mask, clean)


@pytest.mark.parametrize(('noise_area', 'accepted'), [(19, True), (20, False)])
def test_meaningful_region_area_boundary(noise_area, accepted):
    """Treat 20 pixels as meaningful while ignoring smaller separate regions."""
    mask = round_mask()
    mask[42, 3:3 + noise_area] = 255
    detection, reason = detect_marker(mask)
    assert (detection is not None) == accepted
    if not accepted:
        assert 'ambiguous' in reason


def test_missing_or_tiny_regions_are_rejected():
    """Do not invent a target for missing red pixels or a one-pixel speck."""
    mask = np.zeros((50, 70), dtype=np.uint8)
    detection, reason = detect_marker(mask)
    assert detection is None
    assert 'No red region' in reason
    mask[20, 35] = 255
    detection, reason = detect_marker(mask)
    assert detection is None
    assert 'too small' in reason


def test_multiple_round_regions_are_rejected():
    """Reject ambiguous candidates instead of choosing an arbitrary target."""
    mask = round_mask()
    cv2.circle(mask, (15, 35), 6, 255, -1)
    detection, reason = detect_marker(mask)
    assert detection is None
    assert 'ambiguous' in reason


def test_border_clipped_region_is_rejected():
    """Reject a region whose full extent cannot be observed inside the image."""
    mask = np.zeros((50, 70), dtype=np.uint8)
    cv2.circle(mask, (3, 20), 6, 255, -1)
    detection, reason = detect_marker(mask)
    assert detection is None
    assert 'border' in reason


@pytest.mark.parametrize('occlusion', ['half', 'quarter'])
def test_clearly_occluded_round_regions_are_rejected(occlusion):
    """Do not use the biased center of a visibly truncated circular region."""
    mask = round_mask()
    if occlusion == 'half':
        mask[:, :35] = 0
    else:
        mask[:20, :35] = 0
    detection, reason = detect_marker(mask)
    assert detection is None, reason


@pytest.mark.parametrize('shape', ['rectangle', 'ring', 'line'])
def test_non_marker_shapes_are_rejected(shape):
    """Reject filled squares, hollow rings and long thin components."""
    mask = np.zeros((50, 70), dtype=np.uint8)
    if shape == 'rectangle':
        mask[12:27, 25:40] = 255
    elif shape == 'ring':
        cv2.circle(mask, (35, 20), 10, 255, 1)
    else:
        mask[20, 20:50] = 255
    detection, reason = detect_marker(mask)
    assert detection is None, reason
