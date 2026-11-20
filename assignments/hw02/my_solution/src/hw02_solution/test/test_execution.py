"""Focused sequencing/safety tests; no real movement or action server."""

import time
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from hw02_interfaces.action import GoTo
from hw02_solution import execution
from hw02_solution.cspace import inflate
from hw02_solution.grid import GridMap
from hw02_solution.search import Route
import pytest
import rclpy
from rclpy.node import Node
from rclpy.task import Future


class FakeRobot:
    def __init__(self):
        self.available = True
        self.goals = []
        self.futures = []
        self.canceled = 0

    def ready(self):
        return self.available

    def go_to(self, x, y):
        assert not self.futures or self.futures[-1].done()
        self.goals.append((x, y))
        future = Future()
        self.futures.append(future)
        return future

    def cancel(self):
        self.canceled += 1


def pose(point):
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x, message.pose.position.y = point
    message.pose.position.z = 0.14
    return message


@pytest.fixture
def rig(monkeypatch):
    rclpy.init()
    node = Node('hw02_execution_test')
    robot = FakeRobot()
    monkeypatch.setattr(execution, 'RobotClient', lambda node: robot)
    reports = []
    follower = execution.WaypointExecutor(node, lambda state, text: reports.append((state, text)))
    clock = Mock()
    clock.now.return_value.nanoseconds = 1000000000
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    space = inflate(GridMap(8, 8, 1.0, 0.0, 0.0, (0,) * 64))
    route = Route(((1.5, 1.5), (2.5, 1.5), (2.5, 2.5)), 2.0)
    follower.start(route, space)
    follower.on_pose(pose(route.points[0]))
    try:
        yield follower, robot, reports, clock, route
    finally:
        follower.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def advance(follower, clock):
    clock.now.return_value.nanoseconds += 100000000
    follower.tick()


def test_goals_are_sequential_and_completion_requires_every_success(rig):
    follower, robot, reports, clock, route = rig
    advance(follower, clock)
    assert robot.goals == [route.points[0]]
    advance(follower, clock)
    assert len(robot.goals) == 1
    for index, point in enumerate(route.points):
        follower.on_pose(pose(point))
        robot.futures[-1].set_result(GoTo.Result(success=True, message='stopped'))
        advance(follower, clock)
        assert len(robot.goals) == min(index + 2, len(route.points))
    assert follower.state == 'completed' and not follower.pending
    assert [state for state, _ in reports] == ['executing', 'completed']
    advance(follower, clock)
    assert len(robot.goals) == len(route.points)


@pytest.mark.parametrize('reason', ['helper_failure', 'cancel_before_reply'])
def test_failure_or_cancellation_never_sends_the_next_goal(rig, reason):
    follower, robot, reports, clock, _ = rig
    advance(follower, clock)
    if reason == 'helper_failure':
        robot.futures[-1].set_result(
            GoTo.Result(success=False, message='contact or lost feedback'))
        advance(follower, clock)
    else:
        follower.cancel('Inputs changed before acceptance')
        assert follower.pending
        robot.futures[-1].set_result(GoTo.Result(success=True, message='late reply'))
        advance(follower, clock)
    assert follower.state == 'failed' and robot.canceled == 1
    assert reports[-1][0] == 'failed'
    advance(follower, clock)
    assert len(robot.goals) == 1


@pytest.mark.parametrize('problem', [
    'wrong_start', 'readiness_timeout', 'stale_pose', 'clock_stall',
])
def test_unsafe_start_or_stale_monitoring_stops_execution(rig, problem):
    follower, robot, reports, clock, _ = rig
    if problem == 'wrong_start':
        follower.on_pose(pose((4.5, 4.5)))
        advance(follower, clock)
        assert not robot.goals
    elif problem == 'readiness_timeout':
        robot.available = False
        follower.deadline = time.monotonic() - 1
        advance(follower, clock)
        assert not robot.goals
    else:
        advance(follower, clock)
        if problem == 'stale_pose':
            follower.pose_time = time.monotonic() - 2
            advance(follower, clock)
        else:
            follower.clock_changed = time.monotonic() - 2
            follower.tick()
        assert len(robot.goals) == 1
    assert follower.state == 'failed' and reports[-1][0] == 'failed'


def test_success_without_measured_arrival_is_not_accepted(rig):
    follower, robot, reports, clock, route = rig
    advance(follower, clock)
    follower.on_pose(pose(route.points[0]))
    robot.futures[-1].set_result(GoTo.Result(success=True))
    advance(follower, clock)
    assert len(robot.goals) == 2
    robot.futures[-1].set_result(GoTo.Result(success=True))
    advance(follower, clock)
    assert follower.state == 'failed'
    assert 'without measured waypoint arrival' in reports[-1][1]
    assert len(robot.goals) == 2
