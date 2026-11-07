"""Reconstruct the red marker, transform it with TF2 and hover the arm tip above it."""

from dataclasses import dataclass
import math
import signal
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from hw01_sim.arm_client import ArmClient
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    InvalidArgumentException,
    LookupException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .camera_listener import CameraListener


BASE_FRAME = 'base_link'
HOVER_OFFSET_METRES = 0.08
# Assignment staleness limits: simulated image age and wall-clock stream silence.
MAX_OBSERVATION_AGE_SECONDS = 0.5
MAX_CAMERA_SILENCE_WALL_SECONDS = 2.0
# Consecutive agreeing observations needed before commanding motion.
STABLE_OBSERVATIONS = 3
STABLE_TOLERANCE_METRES = 0.01
# The checker requires 0.02 m; a success outside it is treated as a failure.
REACH_TOLERANCE_METRES = 0.02
MAX_MOTION_ATTEMPTS = 3
RETRY_DELAY_WALL_SECONDS = 2.0


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters taken from CameraInfo.k (row-major 3x3 matrix)."""

    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0


def intrinsics_from_camera_info(info: CameraInfo) -> Intrinsics:
    """Read and validate the intrinsic matrix instead of assuming a focal length."""
    k = [float(value) for value in info.k]
    if len(k) != 9 or not all(math.isfinite(value) for value in k):
        raise ValueError('Camera matrix must contain nine finite values.')
    if k[0] <= 0.0 or k[4] <= 0.0:
        raise ValueError('Camera focal lengths must be positive.')
    return Intrinsics(fx=k[0], fy=k[4], cx=k[2], cy=k[5], skew=k[1])


def pixel_to_optical_point(
    u: float, v: float, depth: float, intrinsics: Intrinsics
) -> tuple[float, float, float]:
    """
    Back-project pixel (u, v) with optical-Z depth into camera_optical_frame.

    The pinhole model gives u = fx*X/Z + skew*Y/Z + cx and v = fy*Y/Z + cy.
    Depth is Z itself (not a Euclidean range), so X and Y scale linearly with it.
    """
    if not (math.isfinite(depth) and depth > 0.0):
        raise ValueError('Depth must be finite and positive.')
    y = (v - intrinsics.cy) * depth / intrinsics.fy
    x = (u - intrinsics.cx - intrinsics.skew * y / depth) * depth / intrinsics.fx
    return x, y, depth


def hover_goal(target: PointStamped) -> PointStamped:
    """Return a base-frame goal HOVER_OFFSET_METRES above the marker estimate."""
    if target.header.frame_id != BASE_FRAME:
        raise ValueError('Hover goals must be computed in base_link.')
    goal = PointStamped()
    goal.header = target.header
    goal.point.x = target.point.x
    goal.point.y = target.point.y
    goal.point.z = target.point.z + HOVER_OFFSET_METRES
    return goal


def _as_array(point: PointStamped) -> np.ndarray:
    return np.array([point.point.x, point.point.y, point.point.z], dtype=float)


class MarkerReacher(CameraListener):
    """Turn synchronized camera observations into at most one active arm motion."""

    def __init__(self) -> None:
        """Add calibration, TF2, target publishers and the motion helper."""
        super().__init__(
            'marker_reacher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.camera_info: CameraInfo | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )
        self.base_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_base', 10
        )
        self.latest_tip: PoseStamped | None = None
        self.create_subscription(PoseStamped, '/hw01/measured_tip', self.on_tip, 10)
        self.arm = ArmClient(self)
        self.latest_target_base: PointStamped | None = None
        self.stable_targets: list[np.ndarray] = []
        self.active_goal: PointStamped | None = None
        self.reached_target: np.ndarray | None = None
        self.failed_attempts = 0
        self.retry_after_wall = 0.0
        self.last_camera_wall: float | None = None
        self._camera_stale = False
        self._pipeline_reason: str | None = None
        self._pipeline_reports = 0
        self.create_timer(
            0.5, self.check_camera_silence, clock=Clock(clock_type=ClockType.SYSTEM_TIME)
        )

    # ----- inputs ---------------------------------------------------------

    def on_camera_info(self, message: CameraInfo) -> None:
        """Keep the latest calibration for back-projection."""
        super().on_camera_info(message)
        self.camera_info = message

    def on_tip(self, message: PoseStamped) -> None:
        """Keep the physics-measured tip pose for checking motion outcomes."""
        self.latest_tip = message

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Estimate the marker from this pair and decide whether to move."""
        super().on_synchronized_images(rgb, depth)
        self.last_camera_wall = time.monotonic()
        if self._camera_stale:
            self.get_logger().info('Fresh camera data resumed.')
            self._camera_stale = False
        target, reason = self.estimate_target(rgb)
        self.report_pipeline(reason, target is None)
        self.latest_target_base = target
        if target is None:
            # A missing or unreliable observation must not reuse an earlier target.
            self.stable_targets.clear()
            return
        self.consider_motion(target)

    def check_camera_silence(self) -> None:
        """Invalidate observations when no camera pair arrived for 2 wall seconds."""
        silent = (
            self.last_camera_wall is None
            or time.monotonic() - self.last_camera_wall > MAX_CAMERA_SILENCE_WALL_SECONDS
        )
        if not silent:
            return
        self.latest_target_base = None
        self.stable_targets.clear()
        if not self._camera_stale:
            self._camera_stale = True
            self.get_logger().warning(
                'No fresh synchronized camera data for '
                f'{MAX_CAMERA_SILENCE_WALL_SECONDS:.0f} wall-clock seconds; '
                'not sending goals.'
            )

    # ----- perception -----------------------------------------------------

    def estimate_target(self, rgb: Image) -> tuple[PointStamped | None, str]:
        """Return the marker's base-frame point for this pair, or a rejection reason."""
        detection = self.latest_marker
        depth = self.latest_marker_depth
        if detection is None or depth is None:
            return None, 'Marker or its depth is unavailable.'
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return None, 'Waiting for simulation time on /clock.'
        stamp = Time.from_msg(rgb.header.stamp, clock_type=now.clock_type)
        age = (now - stamp).nanoseconds / 1e9
        if age > MAX_OBSERVATION_AGE_SECONDS:
            return None, f'Observation is stale ({age:.2f} simulated seconds old).'
        if age < -MAX_OBSERVATION_AGE_SECONDS:
            return None, 'Observation timestamp is in the future (clock reset?).'
        info = self.camera_info
        if info is None:
            return None, 'No camera calibration received yet.'
        if info.header.frame_id != rgb.header.frame_id:
            return None, 'Camera calibration frame does not match the image frame.'
        if (info.width, info.height) != (rgb.width, rgb.height):
            return None, 'Camera calibration size does not match the image.'
        try:
            intrinsics = intrinsics_from_camera_info(info)
            u, v = detection.pixel
            x, y, z = pixel_to_optical_point(u, v, depth, intrinsics)
        except ValueError as error:
            return None, f'Cannot reconstruct marker: {error}'
        camera_point = PointStamped()
        camera_point.header = rgb.header
        camera_point.point.x, camera_point.point.y, camera_point.point.z = x, y, z
        self.camera_target_publisher.publish(camera_point)
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, rgb.header.frame_id, stamp
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
            InvalidArgumentException,
        ) as error:
            return None, f'TF {rgb.header.frame_id} -> {BASE_FRAME} unavailable: {error}'
        base_point = do_transform_point(camera_point, transform)
        base_point.header.frame_id = BASE_FRAME
        base_point.header.stamp = rgb.header.stamp
        if not np.all(np.isfinite(_as_array(base_point))):
            return None, 'Transformed marker point is not finite.'
        self.base_target_publisher.publish(base_point)
        return base_point, (
            f'Marker camera=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'base=({base_point.point.x:.3f}, {base_point.point.y:.3f}, '
            f'{base_point.point.z:.3f})'
        )

    def report_pipeline(self, reason: str, rejected: bool) -> None:
        """Log reason changes immediately and repeated reasons every 30th pair."""
        self._pipeline_reports += 1
        was_rejected = self._pipeline_reason is not None
        changed = rejected != was_rejected or (
            rejected and reason != self._pipeline_reason
        )
        if changed or self._pipeline_reports % 30 == 0:
            if rejected:
                self.get_logger().warning('No target: ' + reason)
            else:
                self.get_logger().info(reason)
        self._pipeline_reason = reason if rejected else None

    # ----- motion ---------------------------------------------------------

    def consider_motion(self, target: PointStamped) -> None:
        """Send one goal once fresh observations agree and no motion is active."""
        position = _as_array(target)
        if self.stable_targets and np.linalg.norm(
            position - self.stable_targets[-1]
        ) > STABLE_TOLERANCE_METRES:
            self.stable_targets.clear()
        self.stable_targets = (self.stable_targets + [position])[-STABLE_OBSERVATIONS:]
        if len(self.stable_targets) < STABLE_OBSERVATIONS or self.arm.busy:
            return
        if self.reached_target is not None and np.linalg.norm(
            position - self.reached_target
        ) <= STABLE_TOLERANCE_METRES:
            return
        if self.failed_attempts >= MAX_MOTION_ATTEMPTS:
            return
        if time.monotonic() < self.retry_after_wall:
            return
        if not self.arm.ready():
            self.get_logger().warning(
                'Motion helper not available yet.', throttle_duration_sec=5.0
            )
            return
        # The goal comes from this current observation, not an averaged history.
        goal = hover_goal(target)
        self.active_goal = goal
        self.get_logger().info(
            f'Sending hover goal ({goal.point.x:.3f}, {goal.point.y:.3f}, '
            f'{goal.point.z:.3f}) in {BASE_FRAME}.'
        )
        self.arm.move_to_point(goal).add_done_callback(self.on_motion_done)

    def on_motion_done(self, future) -> None:
        """Report the helper result and compare it with the measured tip."""
        result = future.result()
        goal = self.active_goal
        self.active_goal = None
        if goal is None:
            return
        if result.success:
            error = self.measured_tip_error(goal)
            if error is None:
                self.get_logger().info(
                    f'Motion succeeded: {result.message} '
                    '(no fresh /hw01/measured_tip to confirm).'
                )
                self.record_success(goal)
                return
            if error <= REACH_TOLERANCE_METRES:
                self.get_logger().info(
                    f'Motion succeeded: {result.message}; '
                    f'measured tip is {error:.4f} m from the goal.'
                )
                self.record_success(goal)
                return
            message = (
                f'helper reported success but the measured tip is {error:.4f} m '
                'from the goal'
            )
        else:
            message = result.message
        self.failed_attempts += 1
        self.retry_after_wall = time.monotonic() + RETRY_DELAY_WALL_SECONDS
        give_up = self.failed_attempts >= MAX_MOTION_ATTEMPTS
        self.get_logger().error(
            f'Motion failed ({self.failed_attempts}/{MAX_MOTION_ATTEMPTS}): {message}'
            + ('; giving up.' if give_up else '; will retry from a fresh observation.')
        )

    def record_success(self, goal: PointStamped) -> None:
        """Remember the reached marker estimate so it is not commanded again."""
        target = _as_array(goal)
        target[2] -= HOVER_OFFSET_METRES
        self.reached_target = target
        self.failed_attempts = 0

    def measured_tip_error(self, goal: PointStamped) -> float | None:
        """Return the measured tip distance to goal, or None when it is missing/stale."""
        tip = self.latest_tip
        if tip is None or tip.header.frame_id != BASE_FRAME:
            return None
        age = (self.get_clock().now() - Time.from_msg(tip.header.stamp)).nanoseconds
        if abs(age) / 1e9 > MAX_OBSERVATION_AGE_SECONDS:
            return None
        position = tip.pose.position
        return float(np.linalg.norm(
            np.array([position.x, position.y, position.z]) - _as_array(goal)
        ))

    def cancel_motion(self, timeout_sec: float = 2.0) -> None:
        """Cancel a pending motion and spin briefly so the request is delivered."""
        if not self.arm.busy:
            return
        self.get_logger().info('Cancelling pending motion.')
        self.arm.cancel()
        deadline = time.monotonic() + timeout_sec
        while self.arm.busy and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def _interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def main(args=None) -> None:
    """Run until interrupted, then cancel any pending motion before exiting."""
    # Keep the ROS context alive on Ctrl+C/SIGTERM so the cancel can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = MarkerReacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.cancel_motion()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
