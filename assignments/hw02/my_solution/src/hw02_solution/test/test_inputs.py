"""Isolated ROS input tests; never run fake publishers in a simulation domain."""

import math
import time

from geometry_msgs.msg import PoseStamped
from hw02_solution.planner_inputs import PlannerInputs, read_grid, read_target
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def map_message():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width, message.info.height = 4, 3
    message.info.resolution = 0.5
    message.info.origin.position.x, message.info.origin.position.y = 1.0, -2.0
    message.info.origin.orientation.w = 1.0
    message.data = [0, 100, -1, 0] * 3
    return message


def target(x=1.25, y=-1.75):
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
    publishers['goal'].publish(target(2.75, -0.75))
    node = PlannerInputs()
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
    node, _, _ = rig
    assert node.ready
    assert node.get_parameter('use_sim_time').value is True
    assert node.grid is not None
    assert node.grid.data == tuple(map_message().data)
    assert node.targets == {'start': (1.25, -1.75), 'goal': (2.75, -0.75)}
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
    captured = capfd.readouterr()
    assert 'Rejected goal' in captured.out + captured.err
    publishers['goal'].publish(target(2.75, -0.75))
    spin_until(executor, lambda: node.ready)


def test_new_map_rechecks_target_bounds(rig, capfd):
    node, publishers, executor = rig
    smaller = map_message()
    smaller.info.width = 2
    smaller.data = [0] * 6
    publishers['map'].publish(smaller)
    spin_until(executor, lambda: node.grid is not None and node.grid.width == 2)
    assert not node.ready
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
