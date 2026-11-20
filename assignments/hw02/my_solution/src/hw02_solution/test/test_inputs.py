"""Isolated ROS input tests; never run fake publishers in a simulation domain."""

import math
import time
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from hw02_solution import planner_inputs as inputs_module
from hw02_solution.planner_inputs import PlannerInputs, read_grid, read_target
from nav_msgs.msg import OccupancyGrid, Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def map_message():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width, message.info.height = 4, 3
    message.info.resolution = 0.5
    message.info.origin.position.x, message.info.origin.position.y = 1.0, -2.0
    message.info.origin.orientation.w = 1.0
    message.data = [0] * 12
    return message


def target(x=1.75, y=-1.25):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x, message.pose.position.y = x, y
    message.pose.orientation.w = 1.0
    return message


def spin_until(executor, condition):
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return
    pytest.fail('Input callback did not reach the expected state within five wall seconds')


@pytest.fixture
def rig():
    rclpy.init()
    source = Node('hw02_input_test_source')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {
        'map': source.create_publisher(OccupancyGrid, '/map', qos),
        'start': source.create_publisher(PoseStamped, '/hw02/start', qos),
        'goal': source.create_publisher(PoseStamped, '/hw02/goal', qos),
    }
    publishers['map'].publish(map_message())
    publishers['start'].publish(target())
    publishers['goal'].publish(target(2.25, -1.25))
    node = PlannerInputs(parameter_overrides=[
        Parameter('samples', value=20), Parameter('neighbors', value=5),
        Parameter('max_draws', value=500),
        Parameter('figure_path', value=''),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(node)
    try:
        spin_until(executor, lambda: node.ready)
        yield node, publishers, executor
    finally:
        executor.shutdown()
        node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_late_subscriber_receives_all_inputs_without_motion(rig):
    node, publishers, executor = rig
    assert node.ready
    assert node.get_parameter('use_sim_time').value is True
    for name in ('seed', 'samples', 'neighbors', 'max_draws', 'figure_path'):
        descriptor = node.describe_parameter(name)
        assert descriptor.name == name and descriptor.read_only
    assert node.grid is not None
    assert node.grid.data == tuple(map_message().data)
    assert node.targets == {'start': (1.75, -1.25), 'goal': (2.25, -1.25)}
    assert node.cspace is not None
    assert node.roadmap is not None
    assert node.path is not None
    assert node.path.points[0] == node.targets['start']
    assert node.path.points[-1] == node.targets['goal']
    assert node.path.length > 0
    assert node.roadmap.points[:2] == (node.targets['start'], node.targets['goal'])
    previous = node.roadmap
    previous_path = node.path
    publishers['map'].publish(map_message())
    for _ in range(5):
        executor.spin_once(timeout_sec=0.05)
    assert node.roadmap is previous
    assert node.path is previous_path
    for subscription in node.subscriptions:
        if subscription.topic_name in ('/map', '/hw02/start', '/hw02/goal'):
            assert subscription.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert subscription.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert node.count_publishers('/hw02/cmd_vel') == 0


def test_invalid_map_is_logged_and_clears_previous_input(rig, capfd):
    node, publishers, executor = rig
    invalid = map_message()
    invalid.header.frame_id = 'odom'
    publishers['map'].publish(invalid)
    spin_until(executor, lambda: node.grid is None)
    assert not node.ready
    assert node.cspace is None
    assert node.roadmap is None
    assert node.path is None
    captured = capfd.readouterr()
    assert 'Rejected map' in captured.out + captured.err
    publishers['map'].publish(map_message())
    spin_until(executor, lambda: node.ready)


@pytest.mark.parametrize('invalid_kind', ['frame', 'nan', 'height'])
def test_invalid_goal_is_logged_and_never_reuses_previous_goal(rig, capfd, invalid_kind):
    node, publishers, executor = rig
    invalid = target()
    if invalid_kind == 'frame':
        invalid.header.frame_id = 'odom'
    elif invalid_kind == 'nan':
        invalid.pose.position.x = math.nan
    else:
        invalid.pose.position.z = 1.0
    publishers['goal'].publish(invalid)
    spin_until(executor, lambda: 'goal' not in node.targets)
    assert not node.ready
    assert node.roadmap is None
    assert node.path is None
    captured = capfd.readouterr()
    assert 'Rejected goal' in captured.out + captured.err
    publishers['goal'].publish(target(2.25, -1.25))
    spin_until(executor, lambda: node.ready)


def test_new_map_rechecks_target_bounds(rig, capfd):
    node, publishers, executor = rig
    smaller = map_message()
    smaller.info.width = 2
    smaller.data = [0] * 6
    publishers['map'].publish(smaller)
    spin_until(executor, lambda: node.grid is not None and node.grid.width == 2)
    assert not node.ready
    assert node.roadmap is None
    assert node.path is None
    captured = capfd.readouterr()
    assert 'outside the map' in captured.out + captured.err


def test_rotated_or_nonplanar_map_is_rejected():
    message = map_message()
    message.info.origin.orientation.z = 0.5
    with pytest.raises(ValueError, match='identity'):
        read_grid(message)
    message = map_message()
    message.info.origin.position.z = math.nan
    with pytest.raises(ValueError, match='finite'):
        read_grid(message)


def test_nonfinite_target_height_is_rejected():
    message = target()
    message.pose.position.z = math.inf
    with pytest.raises(ValueError, match='finite'):
        read_target(message)


def test_target_in_inflated_wall_margin_is_rejected(rig, capfd):
    node, publishers, executor = rig
    publishers['goal'].publish(target(1.25, -1.25))
    spin_until(executor, lambda: not node.ready)
    assert node.roadmap is None
    assert node.path is None
    captured = capfd.readouterr()
    assert 'goal touches a configuration-space obstacle' in captured.out + captured.err


def test_changed_obstacles_rebuild_configuration_space(rig, capfd):
    node, publishers, executor = rig
    previous = node.cspace
    changed = map_message()
    changed.data[5] = 100
    publishers['map'].publish(changed)
    spin_until(executor, lambda: node.cspace != previous)
    assert not node.ready
    assert node.roadmap is None
    assert node.path is None
    captured = capfd.readouterr()
    assert 'touches a configuration-space obstacle' in captured.out + captured.err


def test_roadmap_failure_is_logged_once_without_reusing_old_graph(rig, monkeypatch, capfd):
    node, publishers, executor = rig
    builder = Mock(side_effect=ValueError('Sampling budget exhausted'))
    monkeypatch.setattr(inputs_module.prm, 'build_prm', builder)
    publishers['goal'].publish(target(2.3, -1.2))
    spin_until(executor, lambda: node.roadmap is None)
    assert builder.call_count == 1
    captured = capfd.readouterr()
    assert 'Roadmap construction failed: Sampling budget exhausted' in captured.out + captured.err
    publishers['map'].publish(map_message())
    for _ in range(5):
        executor.spin_once(timeout_sec=0.05)
    assert builder.call_count == 1
    assert node.roadmap is None
    assert node.path is None


def test_no_path_and_validation_errors_never_keep_an_old_route(rig, monkeypatch, capfd):
    node, publishers, executor = rig
    planner = Mock(return_value=None)
    monkeypatch.setattr(inputs_module.search, 'plan_route', planner)
    publishers['goal'].publish(target(2.3, -1.2))
    spin_until(executor, lambda: node.path is None)
    assert node.roadmap is not None and planner.call_count == 1
    captured = capfd.readouterr()
    assert 'No roadmap path found' in captured.out + captured.err
    publishers['map'].publish(map_message())
    for _ in range(5):
        executor.spin_once(timeout_sec=0.05)
    assert planner.call_count == 1 and node.path is None
    planner.side_effect = ValueError('Unsafe route')
    publishers['goal'].publish(target(2.2, -1.3))
    spin_until(executor, lambda: planner.call_count == 2)
    assert node.path is None
    captured = capfd.readouterr()
    assert 'Route planning failed: Unsafe route' in captured.out + captured.err


def test_late_outputs_and_failed_updates_clear_stale_visuals(rig, monkeypatch, tmp_path, capfd):
    node, publishers, executor = rig
    listener = Node('hw02_late_output_test')
    executor.add_node(listener)
    latest = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        for name, topic, kind in (
            ('map', '/hw02/cspace', OccupancyGrid),
            ('graph', '/hw02/roadmap', MarkerArray),
            ('path', '/hw02/planned_path', Path),
            ('status', '/hw02/planner_status', String),
        ):
            listener.create_subscription(
                kind, topic, lambda message, key=name: latest.update({key: message}), qos)
        spin_until(executor, lambda: len(latest) == 4)
        assert latest['status'].data == 'path_found'
        assert latest['map'].info == map_message().info
        assert tuple(latest['map'].data) == node.cspace.data
        assert len(latest['path'].poses) == len(node.path.points)
        for publisher in node.publishers:
            if publisher.topic_name.startswith('/hw02/'):
                assert publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                assert publisher.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        invalid = map_message()
        invalid.header.frame_id = 'odom'
        publishers['map'].publish(invalid)
        spin_until(executor, lambda: latest['status'].data == 'failed'
                   and not latest['path'].poses
                   and len(latest['graph'].markers) == 1
                   and set(latest['map'].data) == {100})
        assert latest['graph'].markers[0].action == Marker.DELETEALL
        assert latest['map'].info == map_message().info
        publishers['map'].publish(map_message())
        spin_until(executor, lambda: latest['status'].data == 'path_found')
        node.figure_path = str(tmp_path / 'failed.png')
        monkeypatch.setattr(
            inputs_module.outputs, 'save_figure',
            Mock(side_effect=OSError('disk unavailable')))
        publishers['goal'].publish(target(2.3, -1.2))
        spin_until(executor, lambda: latest['status'].data == 'failed'
                   and not latest['path'].poses and node.path is None)
        captured = capfd.readouterr()
        assert 'Figure export failed: disk unavailable' in captured.out + captured.err
    finally:
        executor.remove_node(listener)
        listener.destroy_node()
