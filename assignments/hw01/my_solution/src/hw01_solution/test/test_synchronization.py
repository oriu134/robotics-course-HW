"""Check timestamp pairing and RGB decoding without Gazebo or arm motion."""

import struct

import cv2
from hw01_solution.camera_listener import CameraListener
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.fixture
def listener():
    """Create an unspun node for deterministic message-filter input."""
    rclpy.init()
    node = CameraListener()
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def image_at(stamp_ns: int, *, depth: bool = False) -> Image:
    """Construct a valid one-pixel image with a controlled capture timestamp."""
    message = Image(
        width=1,
        height=1,
        encoding='32FC1' if depth else 'rgb8',
        step=4 if depth else 3,
        data=struct.pack('<f', 1.0) if depth else b'\xff\x00\x00',
    )
    message.header.frame_id = 'camera_optical_frame'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    return message


def marker_images(stamp_ns: int):
    """Create aligned RGB/depth messages containing a complete red circle."""
    pixels = np.zeros((40, 60, 3), dtype=np.uint8)
    cv2.circle(pixels, (30, 20), 6, (255, 0, 0), -1)
    rgb = image_at(stamp_ns)
    rgb.width, rgb.height, rgb.step = 60, 40, 180
    rgb.data = pixels.tobytes()
    depth = image_at(stamp_ns, depth=True)
    depth.width, depth.height, depth.step = 60, 40, 240
    depth.data = np.ones((40, 60), dtype=np.float32).tobytes()
    return rgb, depth


@pytest.mark.parametrize('depth_first', [False, True])
def test_equal_timestamps_in_either_arrival_order(listener, depth_first):
    """Pair identical capture times regardless of delivery order."""
    rgb = image_at(1_000_000_000)
    depth = image_at(1_000_000_000, depth=True)
    inputs = [
        (listener.image_subscription, rgb),
        (listener.depth_subscription, depth),
    ]
    if depth_first:
        inputs.reverse()
    for subscription, message in inputs:
        subscription.signalMessage(message)
    assert listener.received_images == 1
    assert listener.received_depth_images == 1
    assert listener.received_pairs == 1


@pytest.mark.parametrize(('gap_ns', 'expected_pairs'), [
    (49_999_999, 1),
    (-49_999_999, 1),
    (50_000_000, 0),
    (-50_000_000, 0),
    (50_000_001, 0),
    (-50_000_001, 0),
    (100_000_000, 0),
    (-100_000_000, 0),
])
def test_strict_fifty_millisecond_tolerance(listener, gap_ns, expected_pairs):
    """Honor the standard helper's strict boundary in both time directions."""
    listener.image_subscription.signalMessage(image_at(117_600_000_000))
    listener.depth_subscription.signalMessage(
        image_at(117_600_000_000 + gap_ns, depth=True)
    )
    assert listener.received_pairs == expected_pairs


def test_matching_capture_time_not_message_counter(listener):
    """Keep an unmatched depth sample until its matching RGB sample arrives."""
    pairs = []
    listener.image_synchronizer.registerCallback(
        lambda rgb, depth: pairs.append((rgb, depth))
    )
    old_rgb = image_at(117_600_000_000)
    depth = image_at(117_700_000_000, depth=True)
    matching_rgb = image_at(117_700_000_000)
    listener.image_subscription.signalMessage(old_rgb)
    listener.depth_subscription.signalMessage(depth)
    assert listener.received_pairs == 0
    listener.image_subscription.signalMessage(matching_rgb)
    assert listener.received_pairs == 1
    assert pairs == [(matching_rgb, depth)]
    assert listener.received_images == 2
    assert listener.received_depth_images == 1


def test_old_images_are_evicted_from_bounded_queue(listener):
    """Discard old unmatched images when the ten-message buffer fills."""
    for index in range(11):
        listener.image_subscription.signalMessage(
            image_at(1_000_000_000 + index * 100_000_000)
        )
    listener.depth_subscription.signalMessage(image_at(1_000_000_000, depth=True))
    assert listener.received_pairs == 0
    listener.depth_subscription.signalMessage(image_at(2_000_000_000, depth=True))
    assert listener.received_pairs == 1


def test_matched_rgb_is_not_reused(listener):
    """Consume both messages so another depth message cannot reuse old RGB."""
    listener.image_subscription.signalMessage(image_at(1_000_000_000))
    listener.depth_subscription.signalMessage(image_at(1_000_000_000, depth=True))
    assert listener.received_pairs == 1
    listener.depth_subscription.signalMessage(image_at(1_000_000_000, depth=True))
    assert listener.received_pairs == 1


def test_rgb_message_becomes_pixel_array(listener):
    """Decode a matched RGB message with the correct dimensions and channels."""
    listener.image_subscription.signalMessage(image_at(1_000_000_000))
    listener.depth_subscription.signalMessage(image_at(1_000_000_000, depth=True))
    pixels = listener.latest_rgb_pixels
    assert pixels is not None
    assert pixels.shape == (1, 1, 3)
    assert pixels.dtype == np.uint8
    assert pixels.tolist() == [[[255, 0, 0]]]
    mask = listener.latest_red_mask
    assert mask is not None
    assert mask.shape == (1, 1)
    assert mask.tolist() == [[255]]


def test_bgr_message_is_converted_to_rgb(listener):
    """Honor the incoming encoding rather than confusing red and blue."""
    rgb = image_at(1_000_000_000)
    rgb.encoding = 'bgr8'
    rgb.data = bytes([10, 20, 30])
    listener.on_synchronized_images(rgb, image_at(1_000_000_000, depth=True))
    pixels = listener.latest_rgb_pixels
    assert pixels is not None
    assert pixels.dtype == np.uint8
    assert pixels.tolist() == [[[30, 20, 10]]]


def test_padded_rgb_rows_are_decoded_without_mutating_message(listener):
    """Remove row padding on a copy so each row retains its actual pixels."""
    rgb = image_at(1_000_000_000)
    rgb.height = 2
    rgb.step = 4
    payload = bytes([10, 20, 30, 99, 40, 50, 60, 88])
    rgb.data = payload
    depth = image_at(1_000_000_000, depth=True)
    depth.height = 2
    depth.data = struct.pack('<ff', 1.0, 1.0)
    listener.on_synchronized_images(rgb, depth)
    pixels = listener.latest_rgb_pixels
    assert pixels is not None
    assert pixels.shape == (2, 1, 3)
    assert pixels.tolist() == [[[10, 20, 30]], [[40, 50, 60]]]
    assert rgb.step == 4
    assert bytes(rgb.data) == payload


@pytest.mark.parametrize('bad_case', [
    'unknown_encoding', 'short_payload', 'empty_image', 'short_row_stride',
])
def test_decode_errors_clear_previous_pixels_and_are_reported(listener, capfd, bad_case):
    """Never retain an earlier image as a success-shaped decoding fallback."""
    listener.on_synchronized_images(
        image_at(1_000_000_000), image_at(1_000_000_000, depth=True)
    )
    assert listener.latest_rgb_pixels is not None
    assert listener.latest_red_mask is not None
    capfd.readouterr()
    bad = image_at(2_000_000_000)
    if bad_case == 'unknown_encoding':
        bad.encoding = 'unsupported'
    elif bad_case == 'short_payload':
        bad.data = b'\x00'
    elif bad_case == 'empty_image':
        bad.width = 0
    else:
        bad.step = 2
        bad.data = b'\x00\x00'
    listener.on_synchronized_images(bad, image_at(2_000_000_000, depth=True))
    assert listener.latest_rgb_pixels is None
    assert listener.latest_red_mask is None
    output = capfd.readouterr()
    assert 'Cannot decode RGB image' in output.err


def test_missing_red_region_replaces_previous_mask_and_is_reported(listener, capfd):
    """Keep a zero mask for a valid non-red image instead of reusing red pixels."""
    listener.on_synchronized_images(
        image_at(1_000_000_000), image_at(1_000_000_000, depth=True)
    )
    assert listener.latest_red_mask is not None
    assert np.count_nonzero(listener.latest_red_mask) == 1
    capfd.readouterr()
    gray = image_at(2_000_000_000)
    gray.data = bytes([219, 219, 219])
    listener.received_pairs = 29
    listener.on_synchronized_images(gray, image_at(2_000_000_000, depth=True))
    mask = listener.latest_red_mask
    assert mask is not None
    assert np.count_nonzero(mask) == 0
    assert 'no red region in current image' in capfd.readouterr().err


def test_synchronized_images_store_marker_center(listener):
    """Run region selection through the actual synchronized callback wiring."""
    rgb, depth = marker_images(1_000_000_000)
    listener.image_subscription.signalMessage(rgb)
    listener.depth_subscription.signalMessage(depth)
    marker = listener.latest_marker
    assert marker is not None
    assert marker.pixel == pytest.approx((30.0, 20.0))
    assert marker.mask.shape == (40, 60)


@pytest.mark.parametrize('failure', ['missing', 'ambiguous', 'decode_error'])
def test_old_marker_is_cleared_and_failure_is_reported(listener, capfd, failure):
    """Never keep a previously accepted center after a failed new observation."""
    rgb, depth = marker_images(1_000_000_000)
    listener.on_synchronized_images(rgb, depth)
    assert listener.latest_marker is not None
    capfd.readouterr()
    rgb, depth = marker_images(2_000_000_000)
    if failure == 'missing':
        rgb.data = np.full((40, 60, 3), 219, dtype=np.uint8).tobytes()
    elif failure == 'ambiguous':
        pixels = np.frombuffer(rgb.data, dtype=np.uint8).reshape(40, 60, 3).copy()
        cv2.circle(pixels, (12, 30), 5, (255, 0, 0), -1)
        rgb.data = pixels.tobytes()
    else:
        rgb.data = b'\x00'
    listener.on_synchronized_images(rgb, depth)
    assert listener.latest_marker is None
    output = capfd.readouterr().err
    if failure == 'decode_error':
        assert 'Cannot decode RGB image' in output
    else:
        assert 'Marker not detected' in output
