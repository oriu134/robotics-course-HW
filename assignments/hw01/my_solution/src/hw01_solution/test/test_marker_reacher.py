"""Check reconstruction, TF2 use and motion gating without Gazebo or a real arm."""

from geometry_msgs.msg import PointStamped, TransformStamped
from hw01_interfaces.action import MoveToPoint
from hw01_solution.marker_reacher import (
    hover_goal,
    Intrinsics,
    intrinsics_from_camera_info,
    MarkerReacher,
    pixel_to_optical_point,
    STABLE_OBSERVATIONS,
)
import numpy as np
import pytest
import rclpy
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

from test_synchronization import marker_images

FRAME = 'camera_optical_frame'
NOW_NS = 10_000_000_000


class FakeArm:
    """Record goals instead of contacting the motion helper."""

    def __init__(self):
        self.busy = False
        self.goals = []
        self.futures = []
        self.cancelled = False

    def ready(self):
        return True

    def move_to_point(self, point):
        self.busy = True
        self.goals.append(point)
        future = Future()
        self.futures.append(future)
        return future

    def finish(self, success, message='done'):
        self.busy = False
        self.futures[-1].set_result(MoveToPoint.Result(success=success, message=message))

    def cancel(self):
        self.cancelled = True
        self.busy = False


def camera_info(fx=50.0, fy=50.0, cx=30.0, cy=20.0) -> CameraInfo:
    info = CameraInfo(width=60, height=40)
    info.header.frame_id = FRAME
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def overhead_transform() -> TransformStamped:
    """Camera 1 m above base_link looking down: optical +Z maps to base -Z."""
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = FRAME
    transform.transform.translation.x = 0.4
    transform.transform.translation.z = 1.0
    # 180-degree rotation about base X: optical (x, y, z) -> base (x, -y, -z).
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    return transform


@pytest.fixture
def reacher():
    rclpy.init()
    node = MarkerReacher()
    node.arm = FakeArm()
    clock = node.get_clock()
    clock._set_ros_time_is_active(True)
    clock.set_ros_time_override(Time(nanoseconds=NOW_NS))
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ready(node, *, tf=True):
    node.on_camera_info(camera_info())
    if tf:
        node.tf_buffer.set_transform_static(overhead_transform(), 'test')


def feed(node, stamp_ns=NOW_NS, count=1):
    for _ in range(count):
        rgb, depth = marker_images(stamp_ns)
        node.on_synchronized_images(rgb, depth)


def test_intrinsics_come_from_camera_info():
    intrinsics = intrinsics_from_camera_info(camera_info(fx=80.0, cx=12.5))
    assert (intrinsics.fx, intrinsics.cx, intrinsics.fy) == (80.0, 12.5, 50.0)


@pytest.mark.parametrize('fx', [0.0, -1.0, float('nan')])
def test_invalid_intrinsics_are_rejected(fx):
    with pytest.raises(ValueError):
        intrinsics_from_camera_info(camera_info(fx=fx))


def test_principal_point_back_projects_onto_optical_axis():
    assert pixel_to_optical_point(30.0, 20.0, 0.9, Intrinsics(50, 50, 30, 20)) == (
        pytest.approx(0.0), pytest.approx(0.0), 0.9)


def test_back_projection_scales_with_optical_z_depth():
    intrinsics = Intrinsics(fx=100.0, fy=200.0, cx=10.0, cy=10.0)
    x, y, z = pixel_to_optical_point(20.0, 30.0, 2.0, intrinsics)
    assert (x, y, z) == (pytest.approx(0.2), pytest.approx(0.2), 2.0)


@pytest.mark.parametrize('depth', [0.0, -1.0, float('nan'), float('inf')])
def test_back_projection_rejects_invalid_depth(depth):
    with pytest.raises(ValueError):
        pixel_to_optical_point(0.0, 0.0, depth, Intrinsics(1, 1, 0, 0))


def test_hover_goal_adds_offset_only_in_base_frame():
    target = PointStamped()
    target.header.frame_id = 'base_link'
    target.point.x, target.point.y, target.point.z = 0.3, -0.1, 0.06
    goal = hover_goal(target)
    assert (goal.point.x, goal.point.y, goal.point.z) == (0.3, -0.1, pytest.approx(0.14))
    target.header.frame_id = FRAME
    with pytest.raises(ValueError):
        hover_goal(target)


def test_marker_is_transformed_with_tf2_and_published(reacher):
    ready(reacher)
    published = {}
    reacher.camera_target_publisher.publish = lambda m: published.update(camera=m)
    reacher.base_target_publisher.publish = lambda m: published.update(base=m)
    feed(reacher)
    camera, base = published['camera'], published['base']
    assert camera.header.frame_id == FRAME
    # Marker centered at the principal point, depth 1 m in the synthetic images.
    assert (camera.point.x, camera.point.y, camera.point.z) == (
        pytest.approx(0.0, abs=1e-6), pytest.approx(0.0, abs=1e-6), pytest.approx(1.0))
    assert base.header.frame_id == 'base_link'
    assert base.header.stamp == camera.header.stamp
    assert (base.point.x, base.point.y, base.point.z) == (
        pytest.approx(0.4), pytest.approx(0.0, abs=1e-6), pytest.approx(0.0, abs=1e-6))


def test_one_goal_after_stable_observations_with_hover_offset(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS - 1)
    assert reacher.arm.goals == []
    feed(reacher, count=5)
    assert len(reacher.arm.goals) == 1, 'only one motion may be active'
    goal = reacher.arm.goals[0]
    assert goal.header.frame_id == 'base_link'
    assert goal.point.z == pytest.approx(0.08)


def test_success_is_not_repeated_for_the_same_target(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS)
    reacher.arm.finish(True)
    feed(reacher, count=5)
    assert len(reacher.arm.goals) == 1


def test_failure_is_reported_and_retried_later(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS)
    reacher.arm.finish(False, 'target is outside the arm reach')
    assert reacher.failed_attempts == 1
    feed(reacher, count=STABLE_OBSERVATIONS)
    assert len(reacher.arm.goals) == 1, 'retry waits for the delay'
    reacher.retry_after_wall = 0.0
    feed(reacher, count=1)
    assert len(reacher.arm.goals) == 2


def test_stale_simulation_timestamp_sends_no_goal(reacher):
    ready(reacher)
    feed(reacher, stamp_ns=NOW_NS - 600_000_000, count=5)
    assert reacher.arm.goals == []
    assert reacher.latest_target_base is None


def test_missing_transform_sends_no_goal(reacher):
    ready(reacher, tf=False)
    feed(reacher, count=5)
    assert reacher.arm.goals == []


def test_missing_calibration_sends_no_goal(reacher):
    reacher.tf_buffer.set_transform_static(overhead_transform(), 'test')
    feed(reacher, count=5)
    assert reacher.arm.goals == []


def test_missing_marker_clears_earlier_observations(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS - 1)
    rgb, depth = marker_images(NOW_NS)
    rgb.data = bytes(len(rgb.data))  # all black: no marker
    reacher.on_synchronized_images(rgb, depth)
    assert reacher.latest_target_base is None
    feed(reacher, count=1)
    assert reacher.arm.goals == []


def test_camera_silence_invalidates_observations(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS - 1)
    reacher.last_camera_wall -= 2.5
    reacher.check_camera_silence()
    assert reacher.stable_targets == [] and reacher.latest_target_base is None
    feed(reacher, count=1)
    assert reacher.arm.goals == []


def test_interrupt_cancels_pending_motion(reacher):
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS)
    reacher.cancel_motion(timeout_sec=0.1)
    assert reacher.arm.cancelled


def test_reported_success_far_from_measured_tip_counts_as_failure(reacher):
    from geometry_msgs.msg import PoseStamped
    ready(reacher)
    feed(reacher, count=STABLE_OBSERVATIONS)
    tip = PoseStamped()
    tip.header.frame_id = 'base_link'
    tip.header.stamp = Time(nanoseconds=NOW_NS).to_msg()
    tip.pose.position.x = 0.1
    reacher.on_tip(tip)
    reacher.arm.finish(True)
    assert reacher.failed_attempts == 1
    assert reacher.reached_target is None
    assert np.isfinite(reacher.retry_after_wall)
