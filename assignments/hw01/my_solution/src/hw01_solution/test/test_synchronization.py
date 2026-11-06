"""Check timestamp pairing without Gazebo, image processing or arm motion."""

import struct

from hw01_solution.camera_listener import CameraListener
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
