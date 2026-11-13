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
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .camera_listener import CameraListener


BASE_FRAME = 'base_link'
HOVER_OFFSET_METRES = 0.08
MAX_OBSERVATION_AGE_SECONDS = 0.5  # simulated time
MAX_CAMERA_SILENCE_WALL_SECONDS = 2.0  # wall-clock time
REACH_TOLERANCE_METRES = 0.02  # same tolerance as check_reach
MAX_MOTION_ATTEMPTS = 3
RETRY_DELAY_WALL_SECONDS = 2.0


# ----- geometry (pure functions, no ROS) -----------------------------------

@dataclass(frozen=True)
class Intrinsics:
    """Pinhole parameters from CameraInfo.k (row-major 3x3 matrix)."""

    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0


def intrinsics_from_camera_info(info: CameraInfo) -> Intrinsics:
    """Read and validate the intrinsic matrix (no hardcoded focal length)."""
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

    Inverts u = fx*X/Z + skew*Y/Z + cx and v = fy*Y/Z + cy, with Z = depth.
    The depth was already validated by measure_marker_depth.
    """
    y = (v - intrinsics.cy) * depth / intrinsics.fy
    x = (u - intrinsics.cx - intrinsics.skew * y / depth) * depth / intrinsics.fx
    return x, y, depth


def hover_goal(target: PointStamped) -> PointStamped:
    """Return the base_link goal HOVER_OFFSET_METRES above the marker."""
    if target.header.frame_id != BASE_FRAME:
        raise ValueError('Hover goals must be computed in base_link.')
    goal = PointStamped(header=target.header)
    goal.point.x = target.point.x
    goal.point.y = target.point.y
    goal.point.z = target.point.z + HOVER_OFFSET_METRES
    return goal


def _xyz(point) -> np.ndarray:
    """Return a Point (or PointStamped) as a numpy array."""
    point = getattr(point, 'point', point)
    return np.array([point.x, point.y, point.z], dtype=float)


class Rejected(Exception):
    """This observation cannot be used; the message says why."""


# ----- ROS node -------------------------------------------------------------

class MarkerReacher(CameraListener):
    """Turn each camera pair into a marker estimate; send at most one motion."""

    def __init__(self) -> None:
        """Add calibration, TF2, target publishers and the motion helper."""
        super().__init__(
            'marker_reacher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.latest_tip: PoseStamped | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_camera', 10
        )
        self.base_target_publisher = self.create_publisher(
            PointStamped, '/hw01/target_base', 10
        )
        self.create_subscription(PoseStamped, '/hw01/measured_tip', self.on_tip, 10)
        self.arm = ArmClient(self)

        # Observation state: cleared whenever an observation is missing or stale.
        self.latest_target_base: PointStamped | None = None
        self.last_camera_wall: float | None = None
        # Motion state.
        self.active_goal: PointStamped | None = None
        self.done = False  # one stationary marker per launch: stop after success
        self.failed_attempts = 0
        self.retry_after_wall = 0.0
        self.shutting_down = False
        # Logging state.
        self._last_reason: str | None = None
        self._camera_silent = False

        wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)  # keeps ticking if sim stops
        self.create_timer(0.5, self.check_camera_silence, clock=wall_clock)

    # ----- inputs -----

    def on_tip(self, message: PoseStamped) -> None:
        """Keep the physics-measured tip pose."""
        self.latest_tip = message

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Estimate the marker from this pair and maybe start a motion."""
        super().on_synchronized_images(rgb, depth)
        self.last_camera_wall = time.monotonic()
        if self._camera_silent:
            self._camera_silent = False
            self.get_logger().info('Fresh camera data resumed.')
        try:
            target = self.estimate_target(rgb)
        except Rejected as reason:
            # A missing or unreliable observation must not reuse an earlier target.
            self.forget_observations()
            self.log_once(f'No target: {reason}', warning=True)
            return
        self.latest_target_base = target
        self.consider_motion(target)

    def check_camera_silence(self) -> None:
        """Wall-clock timer: forget observations after 2 s without camera data."""
        if self.last_camera_wall is None:
            return  # nothing received yet, so nothing to forget
        if time.monotonic() - self.last_camera_wall <= MAX_CAMERA_SILENCE_WALL_SECONDS:
            return
        self.forget_observations()
        if not self._camera_silent:
            self._camera_silent = True
            self.get_logger().warning('No camera data for 2 wall-clock seconds; not moving.')

    def forget_observations(self) -> None:
        """Drop the current target so it can never be reused."""
        self.latest_target_base = None

    def log_once(self, message: str, warning: bool = False) -> None:
        """Log only when the message changes, to avoid one line per frame."""
        if message == self._last_reason:
            return
        self._last_reason = message
        # rclpy needs one fixed severity per call site, hence two calls.
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    # ----- perception: pixel -> camera point -> base point -----

    def estimate_target(self, rgb: Image) -> PointStamped:
        """Return the marker in base_link for this pair, or raise Rejected."""
        self.check_fresh(rgb)
        camera_point = self.reconstruct(rgb)
        self.camera_target_publisher.publish(camera_point)
        base_point = self.to_base_frame(camera_point)
        self.base_target_publisher.publish(base_point)
        cam, base = _xyz(camera_point).round(3), _xyz(base_point).round(3)
        self.log_once(f'Marker camera={tuple(cam.tolist())} base={tuple(base.tolist())}')
        return base_point

    def check_fresh(self, rgb: Image) -> None:
        """Reject observations older than 0.5 simulated seconds."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            raise Rejected('waiting for simulation time on /clock.')
        age = (now - Time.from_msg(rgb.header.stamp)).nanoseconds / 1e9
        if age > MAX_OBSERVATION_AGE_SECONDS:
            raise Rejected(f'observation is stale ({age:.2f} simulated seconds old).')
        if age < -MAX_OBSERVATION_AGE_SECONDS:
            raise Rejected('observation timestamp is in the future (clock reset?).')

    def reconstruct(self, rgb: Image) -> PointStamped:
        """Back-project the marker pixel + depth into the image's optical frame."""
        if self.latest_marker is None or self.latest_marker_depth is None:
            raise Rejected('marker or its depth is unavailable.')
        info = self.camera_info
        if info is None:
            raise Rejected('no camera calibration received yet.')
        if info.header.frame_id != rgb.header.frame_id:
            raise Rejected('camera calibration frame does not match the image frame.')
        u, v = self.latest_marker.pixel
        try:
            xyz = pixel_to_optical_point(
                u, v, self.latest_marker_depth, intrinsics_from_camera_info(info)
            )
        except ValueError as error:
            raise Rejected(f'cannot reconstruct marker: {error}') from error
        point = PointStamped(header=rgb.header)  # optical frame + capture stamp
        point.point.x, point.point.y, point.point.z = xyz
        return point

    def to_base_frame(self, camera_point: PointStamped) -> PointStamped:
        """Transform the point into base_link with TF2 at the image timestamp."""
        source = camera_point.header.frame_id
        try:
            # Zero timeout: never block the callback; the next frame retries.
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, source, Time.from_msg(camera_point.header.stamp)
            )
        except TransformException as error:
            raise Rejected(f'TF {source} -> {BASE_FRAME} unavailable: {error}') from error
        base_point = do_transform_point(camera_point, transform)
        base_point.header = Header(stamp=camera_point.header.stamp, frame_id=BASE_FRAME)
        if not np.all(np.isfinite(_xyz(base_point))):
            raise Rejected('transformed marker point is not finite.')
        return base_point

    # ----- motion -----

    def consider_motion(self, target: PointStamped) -> None:
        """Send a hover goal from this fresh observation unless something blocks it."""
        if not self.may_send():
            return
        goal = hover_goal(target)
        self.active_goal = goal
        x, y, z = _xyz(goal)
        self.get_logger().info(f'Sending hover goal ({x:.3f}, {y:.3f}, {z:.3f}) in base_link.')
        self.arm.move_to_point(goal).add_done_callback(self.on_motion_done)

    def may_send(self) -> bool:
        """Check the one-motion rule, an earlier success and retry limits."""
        if self.done or self.arm.busy or self.failed_attempts >= MAX_MOTION_ATTEMPTS:
            return False
        if time.monotonic() < self.retry_after_wall:
            return False
        if not self.arm.ready():
            self.get_logger().warning('Motion helper not available yet.', throttle_duration_sec=5)
            return False
        return True

    def on_motion_done(self, future) -> None:
        """Log the helper result, double-checked against the measured tip."""
        goal, self.active_goal = self.active_goal, None
        if goal is None:
            return
        result = future.result()
        if not result.success:
            self.record_failure(result.message)
            return
        error = self.measured_tip_error(goal)
        if error is not None and error > REACH_TOLERANCE_METRES:
            self.record_failure(f'helper reported success but the tip is {error:.4f} m away')
            return
        tip = self.latest_tip
        where = '' if tip is None else ' at ({:.3f}, {:.3f}, {:.3f})'.format(
            *_xyz(tip.pose.position)
        )
        checked = (
            'no fresh measured tip' if error is None
            else f'tip {error:.4f} m from goal{where}'
        )
        self.get_logger().info(f'Motion succeeded: {result.message} ({checked}).')
        self.done = True

    def record_failure(self, message: str) -> None:
        """Count a failed motion and schedule a retry from a fresh observation."""
        if self.shutting_down:
            self.get_logger().info(f'Motion stopped: {message}')
            return
        self.failed_attempts += 1
        self.retry_after_wall = time.monotonic() + RETRY_DELAY_WALL_SECONDS
        give_up = self.failed_attempts >= MAX_MOTION_ATTEMPTS
        self.get_logger().error(
            f'Motion failed ({self.failed_attempts}/{MAX_MOTION_ATTEMPTS}): {message}; '
            + ('giving up.' if give_up else 'will retry from a fresh observation.')
        )

    def measured_tip_error(self, goal: PointStamped) -> float | None:
        """Distance from the measured tip to the goal; None if missing or stale."""
        tip = self.latest_tip
        if tip is None or tip.header.frame_id != BASE_FRAME:
            return None
        age = (self.get_clock().now() - Time.from_msg(tip.header.stamp)).nanoseconds / 1e9
        if abs(age) > MAX_OBSERVATION_AGE_SECONDS:
            return None
        return float(np.linalg.norm(_xyz(tip.pose.position) - _xyz(goal)))

    def cancel_motion(self, timeout_sec: float = 2.0) -> None:
        """Cancel a pending motion and spin briefly so the request is delivered."""
        self.shutting_down = True
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
    # Own signal handling keeps the ROS context alive so the cancel can be sent.
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
