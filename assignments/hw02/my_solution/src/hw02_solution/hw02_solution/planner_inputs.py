"""Receive and validate planner inputs without planning or commanding motion."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .grid import GridMap


INPUT_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


def read_grid(message: OccupancyGrid) -> GridMap:
    """Validate a map message and retain its raw, uninflated occupancy values."""
    if message.header.frame_id != 'map':
        raise ValueError('Map frame must be map')
    origin = message.info.origin
    q = origin.orientation
    values = (origin.position.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Map origin pose must be finite')
    if (any(abs(v) > 1e-6 for v in values[:-1])
            or not math.isclose(abs(q.w), 1.0, abs_tol=1e-6)):
        raise ValueError('Only a planar map with identity origin rotation is supported')
    return GridMap(
        message.info.width, message.info.height, message.info.resolution,
        origin.position.x, origin.position.y, tuple(message.data))


def read_target(message: PoseStamped) -> tuple[float, float]:
    """Validate a two-dimensional target in map; heading is not a planning input."""
    point = message.pose.position
    if message.header.frame_id != 'map':
        raise ValueError('Target frame must be map')
    if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
        raise ValueError('Target coordinates must be finite')
    if abs(point.z) > 1e-6:
        raise ValueError('Target z must be zero')
    return point.x, point.y


class PlannerInputs(Node):
    """Wait for a valid map, start and goal without requesting any movement."""

    def __init__(self) -> None:
        super().__init__('planner_inputs')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.grid: GridMap | None = None
        self.targets: dict[str, tuple[float, float]] = {}
        self.ready = False
        self.create_subscription(OccupancyGrid, '/map', self.on_map, INPUT_QOS)
        for label in ('start', 'goal'):
            self.create_subscription(
                PoseStamped, f'/hw02/{label}',
                lambda message, name=label: self.on_target(name, message), INPUT_QOS)
        self.get_logger().info('Waiting for map, start and goal; no motion is enabled.')

    def on_map(self, message: OccupancyGrid) -> None:
        """Clear an invalid map rather than retaining a previously accepted map."""
        previous = self.grid
        self.grid = None
        try:
            self.grid = read_grid(message)
        except ValueError as error:
            self.ready = False
            self.get_logger().error(f'Rejected map: {error}')
            return
        if self.grid != previous:
            self.ready = False
        self.check_ready()

    def on_target(self, label: str, message: PoseStamped) -> None:
        """Reject invalid target updates without reusing the old target."""
        previous = self.targets.pop(label, None)
        try:
            self.targets[label] = read_target(message)
        except ValueError as error:
            self.ready = False
            self.get_logger().error(f'Rejected {label}: {error}')
            return
        if self.targets[label] != previous:
            self.ready = False
        self.check_ready()

    def check_ready(self) -> None:
        """Check target bounds whenever any input changes."""
        was_ready = self.ready
        self.ready = False
        if self.grid is None or any(name not in self.targets for name in ('start', 'goal')):
            return
        try:
            cells = {name: self.grid.world_to_grid(*xy) for name, xy in self.targets.items()}
        except ValueError as error:
            self.get_logger().error(f'Inputs not ready: {error}')
            return
        self.ready = True
        if not was_ready:
            self.get_logger().info(
                f'Inputs ready: map={self.grid.width}x{self.grid.height}, '
                f'resolution={self.grid.resolution:.3f} m; '
                f'start={self.targets["start"]} cell={cells["start"]}; '
                f'goal={self.targets["goal"]} cell={cells["goal"]}. No motion.')


def main(args: list[str] | None = None) -> None:
    """Spin the input-only node until interrupted."""
    rclpy.init(args=args)
    node = PlannerInputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
