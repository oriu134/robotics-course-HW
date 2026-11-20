"""Nonblocking, one-destination-at-a-time execution through the supplied helper."""

from collections.abc import Callable
import math
import time

from geometry_msgs.msg import PoseStamped
from hw02_interfaces.action import GoTo
from hw02_sim.robot_client import RobotClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future

from .cspace import edge_is_free, Point
from .grid import GridMap
from .search import Route


class WaypointExecutor:
    """Run one validated mission; failures require a node/simulation restart."""

    def __init__(self, node: Node, report: Callable[[str, str], None]) -> None:
        self.node, self.report = node, report
        self.robot = RobotClient(node)
        self.state = 'idle'
        self.route: Route | None = None
        self.cspace: GridMap | None = None
        self.index = 0
        self.future: Future[GoTo.Result] | None = None
        self.pose: Point | None = None
        self.pose_time = self.clock_changed = self.deadline = 0.0
        self.clock_seen = 0
        node.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_pose, 10)
        self.timer = node.create_timer(
            0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @property
    def active(self) -> bool:
        """Include readiness waiting as part of the one-shot mission."""
        return self.state in ('waiting', 'executing')

    @property
    def pending(self) -> bool:
        """Keep spinning during cancellation until the helper returns."""
        return self.future is not None and not self.future.done()

    def start(self, route: Route, cspace: GridMap) -> None:
        """Arm execution without sending a goal from the planning callback."""
        if self.state != 'idle' or not route.points:
            raise ValueError('Execution requires a complete route and a fresh node')
        self.route, self.cspace = route, cspace
        self.state = 'waiting'
        self.deadline = time.monotonic() + 30.0
        self.clock_seen = self.node.get_clock().now().nanoseconds
        self.clock_changed = 0.0
        self.node.get_logger().info(
            'Execution requested; waiting for fresh pose/clock and helper.')

    def on_pose(self, message: PoseStamped) -> None:
        """Never retain an invalid localization sample as current feedback."""
        point = message.pose.position
        if (message.header.frame_id != 'map'
                or not all(math.isfinite(v) for v in (point.x, point.y, point.z))):
            self.pose, self.pose_time = None, 0.0
            if self.active:
                self.cancel('Invalid robot localization')
            else:
                self.node.get_logger().error('Rejected robot localization: invalid frame/position')
            return
        self.pose, self.pose_time = (point.x, point.y), time.monotonic()

    def cancel(self, reason: str) -> None:
        """Latch failure and cancel even when acceptance has not arrived yet."""
        if self.state in ('idle', 'failed'):
            return
        self.state = 'failed'
        self.robot.cancel()
        self.report('failed', reason)

    def tick(self) -> None:
        """Advance only after a successful result and independently measured arrival."""
        if not self.active:
            return
        if self.route is None or self.cspace is None:
            self.cancel('Execution has no validated route')
            return
        now = time.monotonic()
        clock = self.node.get_clock().now().nanoseconds
        if clock != self.clock_seen:
            self.clock_seen, self.clock_changed = clock, now
        fresh = (self.pose is not None and now - self.pose_time < 1.0 and clock > 0
                 and self.clock_changed > 0 and now - self.clock_changed < 1.0)
        if self.state == 'waiting':
            if not fresh or not self.robot.ready():
                if now >= self.deadline:
                    self.cancel('Execution readiness timed out after 30 wall seconds')
                return
            if self.pose is None:
                raise RuntimeError('Fresh localization unexpectedly missing')
            if math.dist(self.pose, self.route.points[0]) > 0.03:
                self.cancel('Robot is not at the supplied start; restart the simulation')
                return
            try:
                if not edge_is_free(self.cspace, self.pose, self.route.points[0]):
                    raise ValueError('initial connection is obstructed')
            except ValueError as error:
                self.cancel(f'Unsafe initial connection: {error}')
                return
            self.state = 'executing'
            self.report('executing', f'Executing {len(self.route.points)} waypoints')
        if not fresh or not self.robot.ready():
            self.cancel('Movement helper, localization or simulation clock unavailable/stale')
            return
        if self.future is not None:
            if not self.future.done():
                return
            result = self.future.result()
            self.future = None
            if result is None or not result.success:
                detail = 'missing result' if result is None else result.message
                self.cancel(f'Waypoint {self.index + 1} failed: {detail}')
                return
            if self.pose is None or math.dist(self.pose, self.route.points[self.index]) > 0.03:
                self.cancel('Helper reported success without measured waypoint arrival')
                return
            self.index += 1
        if self.index == len(self.route.points):
            self.state = 'completed'
            self.report('completed', 'All waypoints reached; helper stopped the robot')
            return
        target = self.route.points[self.index]
        self.node.get_logger().info(
            f'Waypoint {self.index + 1}/{len(self.route.points)}: {target}')
        self.future = self.robot.go_to(*target)

    def shutdown(self) -> None:
        """Stop the timer and request cancellation while ROS is still alive."""
        self.timer.cancel()
        if self.active:
            self.cancel('Execution interrupted; pending movement canceled')
