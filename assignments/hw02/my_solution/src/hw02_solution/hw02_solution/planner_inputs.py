"""Plan a complete PRM/A* route without commanding robot movement."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from . import outputs
from . import roadmap as prm
from . import search
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
    """Plan once per valid changed input set, without requesting movement."""

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
        self.figure_path = self.declare_parameter(
            'figure_path', 'build/hw02/planner.png', ParameterDescriptor(read_only=True)
        ).get_parameter_value().string_value
        self.grid: GridMap | None = None
        self.cspace: GridMap | None = None
        self.map_info: MapMetaData | None = None
        self.roadmap: prm.Roadmap | None = None
        self.path: search.Route | None = None
        self.targets: dict[str, tuple[float, float]] = {}
        self.ready = False
        self.status = 'planning'
        self.cspace_pub = self.create_publisher(OccupancyGrid, '/hw02/cspace', INPUT_QOS)
        self.roadmap_pub = self.create_publisher(MarkerArray, '/hw02/roadmap', INPUT_QOS)
        self.path_pub = self.create_publisher(Path, '/hw02/planned_path', INPUT_QOS)
        self.status_pub = self.create_publisher(String, '/hw02/planner_status', INPUT_QOS)
        self.publish_outputs('planning')
        self.create_subscription(OccupancyGrid, '/map', self.on_map, INPUT_QOS)
        for label in ('start', 'goal'):
            self.create_subscription(
                PoseStamped, f'/hw02/{label}',
                lambda message, name=label: self.on_target(name, message), INPUT_QOS)
        self.get_logger().info('Waiting for map, start and goal; no motion is enabled.')

    def clear_plan(self) -> None:
        """Discard the roadmap and selected route together."""
        self.roadmap = None
        self.path = None

    def publish_outputs(self, status: str) -> None:
        """Publish one complete, reliable/transient-local planning snapshot."""
        self.status = status
        if status in ('planning', 'failed'):
            try:
                outputs.remove_figure(self.figure_path)
            except (OSError, ValueError) as error:
                self.get_logger().error(f'Cannot clear previous figure: {error}')
                self.status = 'failed'
                self.path = None
        occupancy, markers, path = outputs.make_messages(
            self.map_info, self.cspace, self.roadmap, self.path,
            self.get_clock().now().to_msg())
        self.cspace_pub.publish(occupancy)
        self.roadmap_pub.publish(markers)
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=self.status))

    def on_map(self, message: OccupancyGrid) -> None:
        """Clear an invalid map rather than retaining a previously accepted map."""
        previous = self.grid
        self.grid = None
        try:
            self.grid = read_grid(message)
        except ValueError as error:
            self.ready = False
            self.cspace = None
            self.clear_plan()
            self.get_logger().error(f'Rejected map: {error}')
            self.publish_outputs('failed')
            return
        info_changed = self.map_info != message.info
        self.map_info = deepcopy(message.info)
        if self.grid != previous or self.cspace is None:
            self.ready = False
            self.clear_plan()
            self.cspace = inflate(self.grid)
            self.get_logger().info(
                f'Configuration space ready: radius={PLANNING_RADIUS:.2f} m; '
                f'{self.cspace.data.count(0)} free cells. No motion.')
        elif info_changed:
            self.publish_outputs(self.status)
        self.check_ready()

    def on_target(self, label: str, message: PoseStamped) -> None:
        """Reject invalid target updates without reusing the old target."""
        previous = self.targets.pop(label, None)
        try:
            self.targets[label] = read_target(message)
        except ValueError as error:
            self.ready = False
            self.clear_plan()
            self.get_logger().error(f'Rejected {label}: {error}')
            self.publish_outputs('failed')
            return
        if self.targets[label] != previous:
            self.ready = False
            self.clear_plan()
        self.check_ready()

    def check_ready(self) -> None:
        """Check target bounds and footprint clearance whenever inputs change."""
        was_ready = self.ready
        self.ready = False
        if (self.grid is None or self.cspace is None
                or any(name not in self.targets for name in ('start', 'goal'))):
            self.clear_plan()
            self.publish_outputs('failed' if self.status == 'failed' else 'planning')
            return
        try:
            cells = {name: self.cspace.world_to_grid(*xy) for name, xy in self.targets.items()}
            for name, xy in self.targets.items():
                if not point_is_free(self.cspace, xy):
                    raise ValueError(f'{name} touches a configuration-space obstacle')
        except ValueError as error:
            self.clear_plan()
            self.get_logger().error(f'Inputs not ready: {error}')
            self.publish_outputs('failed')
            return
        self.ready = True
        if not was_ready:
            self.publish_outputs('planning')
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
                self.publish_outputs('failed')
                return
            self.get_logger().info(
                f'Roadmap built: seed={self.prm_settings["seed"]}; '
                f'{len(self.roadmap.points)} nodes, {self.roadmap.edge_count} edges; '
                f'{self.roadmap.draws} draws, {self.roadmap.candidate_edges} candidate edges; '
                f'{time.perf_counter() - started:.3f} s. No motion.')
            started = time.perf_counter()
            try:
                self.path = search.plan_route(self.cspace, self.roadmap)
            except ValueError as error:
                self.get_logger().error(f'Route planning failed: {error}')
                self.publish_outputs('failed')
                return
            if self.path is None:
                self.get_logger().warning(
                    'No roadmap path found in this bounded graph. No motion.')
            else:
                self.get_logger().info(
                    f'Path found: {len(self.path.points)} waypoints; '
                    f'length={self.path.length:.3f} m; search/validation='
                    f'{time.perf_counter() - started:.3f} s. No motion.')
            status = 'no_path_found' if self.path is None else 'path_found'
            try:
                if self.figure_path:
                    filename = outputs.save_figure(
                        self.figure_path, self.grid, self.cspace, self.roadmap, self.path,
                        self.prm_settings['seed'], status)
                    self.get_logger().info(f'Figure saved: {filename}')
            except (OSError, ValueError) as error:
                self.path = None
                self.get_logger().error(f'Figure export failed: {error}')
                self.publish_outputs('failed')
                return
            self.publish_outputs(status)


def main(args: list[str] | None = None) -> None:
    """Spin the planning-only node until interrupted."""
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
