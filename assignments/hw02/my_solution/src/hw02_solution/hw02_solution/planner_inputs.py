"""Validate inputs and build a PRM, without route search or robot movement."""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from . import roadmap as prm
from .cspace import inflate, PLANNING_RADIUS, point_is_free
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
    """Build one roadmap for each valid changed input set, without movement."""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__('planner_inputs', parameter_overrides=parameter_overrides)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.prm_settings: dict[str, int] = {
            name: self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)
            ).get_parameter_value().integer_value
            for name, default in (
                ('seed', 0), ('samples', prm.DEFAULT_SAMPLES),
                ('neighbors', prm.DEFAULT_NEIGHBORS), ('max_draws', prm.DEFAULT_MAX_DRAWS))
        }
        self.grid: GridMap | None = None
        self.cspace: GridMap | None = None
        self.roadmap: prm.Roadmap | None = None
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
            self.cspace = None
            self.roadmap = None
            self.get_logger().error(f'Rejected map: {error}')
            return
        if self.grid != previous or self.cspace is None:
            self.ready = False
            self.roadmap = None
            self.cspace = inflate(self.grid)
            self.get_logger().info(
                f'Configuration space ready: radius={PLANNING_RADIUS:.2f} m; '
                f'{self.cspace.data.count(0)} free cells. No motion.')
        self.check_ready()

    def on_target(self, label: str, message: PoseStamped) -> None:
        """Reject invalid target updates without reusing the old target."""
        previous = self.targets.pop(label, None)
        try:
            self.targets[label] = read_target(message)
        except ValueError as error:
            self.ready = False
            self.roadmap = None
            self.get_logger().error(f'Rejected {label}: {error}')
            return
        if self.targets[label] != previous:
            self.ready = False
            self.roadmap = None
        self.check_ready()

    def check_ready(self) -> None:
        """Check target bounds and footprint clearance whenever inputs change."""
        was_ready = self.ready
        self.ready = False
        if self.cspace is None or any(name not in self.targets for name in ('start', 'goal')):
            self.roadmap = None
            return
        try:
            cells = {name: self.cspace.world_to_grid(*xy) for name, xy in self.targets.items()}
            for name, xy in self.targets.items():
                if not point_is_free(self.cspace, xy):
                    raise ValueError(f'{name} touches a configuration-space obstacle')
        except ValueError as error:
            self.roadmap = None
            self.get_logger().error(f'Inputs not ready: {error}')
            return
        self.ready = True
        if not was_ready:
            self.get_logger().info(
                f'Inputs ready: map={self.cspace.width}x{self.cspace.height}, '
                f'resolution={self.cspace.resolution:.3f} m; '
                f'start={self.targets["start"]} cell={cells["start"]}; '
                f'goal={self.targets["goal"]} cell={cells["goal"]}. No motion.')
            started = time.perf_counter()
            try:
                self.roadmap = prm.build_prm(
                    self.cspace, self.targets['start'], self.targets['goal'],
                    **self.prm_settings)
            except ValueError as error:
                self.get_logger().error(f'Roadmap construction failed: {error}')
                return
            self.get_logger().info(
                f'Roadmap built: seed={self.prm_settings["seed"]}; '
                f'{len(self.roadmap.points)} nodes, {self.roadmap.edge_count} edges; '
                f'{self.roadmap.draws} draws, {self.roadmap.candidate_edges} candidate edges; '
                f'{time.perf_counter() - started:.3f} s. No route search or motion.')


def main(args: list[str] | None = None) -> None:
    """Spin the roadmap-building node until interrupted."""
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
