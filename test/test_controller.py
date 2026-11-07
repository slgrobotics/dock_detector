"""Controller convergence and freshness checks; no ROS graph or motion commands."""
import math
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType

from dock_detector.control import Limits, PoseController, GoalMemory, cached_reverse_command
from dock_detector.controller import DockController, parser, validate


def test_reverse_controller_converges_from_rough_staging():
    for y in (-.25, 0., .25):
        for yaw in (-.35, 0., .35):
            robot = [.8, y, yaw]
            law = PoseController(Limits())
            for _ in range(6000):
                v, w, mode = law.command(robot, (0., 0., 0.))
                assert -.035 <= v <= 0. and abs(w) <= .08
                if mode == 'settle':
                    break
                robot[0] += v*math.cos(robot[2])*.05
                robot[1] += v*math.sin(robot[2])*.05
                robot[2] += w*.05
            assert mode == 'settle'
            assert math.hypot(*robot[:2]) <= .05
            assert abs(robot[2]) <= math.radians(2)


def config():
    return parser().parse_args(['--detected-goal'])


def test_invalid_limits():
    args = config()
    args.speed = float('nan')
    with pytest.raises(ValueError):
        validate(args)


def test_frozen_tf_rejected_even_when_ros_clock_is_paused():
    tf = TransformStamped()
    tf.header.stamp.sec = 10
    tf.transform.rotation.w = 1.
    node = SimpleNamespace(args=config(), robot_stamp=None, robot_stamp_wall=None,
                           buffer=SimpleNamespace(lookup_transform=lambda *a: tf),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME)))
    with patch('dock_detector.controller.time.monotonic', return_value=100.):
        assert DockController.robot(node) == (0., 0., 0.)
    with patch('dock_detector.controller.time.monotonic', return_value=101.):
        with pytest.raises(RuntimeError, match='stopped updating'):
            DockController.robot(node)


def test_detection_acquisition_duplicate_loss_and_jump():
    # Exercise real callbacks with an identity timestamped transform.
    from collections import deque
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    node = SimpleNamespace(args=config(), samples=deque(maxlen=5),
                           last_pose_stamp=None, last_pose_wall=None, goal=None, goal_memory=GoalMemory(),
                           buffer=SimpleNamespace(lookup_transform=lambda *a: tf),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10, clock_type=ClockType.ROS_TIME)),
                           emit=lambda *a, **k: None)
    msg = PoseStamped()
    msg.header.frame_id = 'odom'
    # Inward Nav2 dock yaw pi corresponds to reverse parked robot yaw zero.
    msg.pose.orientation.z = 1.
    msg.pose.orientation.w = 0.
    with patch('dock_detector.controller.time.monotonic', return_value=100.):
        for i in range(5):
            msg.header.stamp = Time(seconds=9.8+i*.01).to_msg()
            DockController.on_pose(node, msg)
        assert node.goal == pytest.approx((0., 0., 0.))
        DockController.on_pose(node, msg)
        assert node.last_pose_stamp == Time.from_msg(msg.header.stamp).nanoseconds
        msg.header.stamp = Time(seconds=9.9).to_msg()
        msg.pose.position.x = .5
        DockController.on_pose(node, msg)
        assert node.goal is None and len(node.samples) == 1
        assert node.goal_memory.goal is None  # Jump revokes blind-motion goal.
    with patch('dock_detector.controller.time.monotonic', return_value=101.):
        assert DockController.detected_goal(node) is None
        assert len(node.samples) == 0


def test_cached_goal_deadline_uses_trusted_pose_not_isolated_packets():
    memory = GoalMemory()
    memory.accept((0., 0., 0.), 100., 10_000_000_000)
    assert memory.lookup(102., 12_000_000_000, .5, 3.) == (0., 0., 0.)
    assert memory.lookup(103.51, 13_510_000_000, .5, 3.) is None
    memory.accept((0., 0., 0.), 100., 10_000_000_000)
    assert memory.lookup(100.2, 9_000_000_000, .5, 3.) is None  # clock rollback


def test_cached_reverse_never_aligns_or_declares_success():
    law = PoseController(Limits())
    v, w, mode = cached_reverse_command(law, (.8, 0., 0.), (0., 0., 0.))
    assert v < 0 and mode == 'reverse_cached_goal'
    assert cached_reverse_command(law, (0., 0., .2), (0., 0., 0.)) == (0., 0., 'wait_for_detection')
    assert cached_reverse_command(law, (0., 0., 0.), (0., 0., 0.)) == (0., 0., 'wait_for_detection')


@pytest.mark.parametrize('value', [-1., float('inf'), float('nan')])
def test_reject_invalid_grace(value):
    args = config()
    args.detection_grace = value
    with pytest.raises(ValueError):
        validate(args)


@pytest.mark.parametrize('fault', ['none', 'tf', 'monitor', 'action', 'disabled', 'forward'])
def test_run_bridges_detection_only_and_limits_command_rate(fault):
    args = config()
    args.execute = True
    args.forward = fault == 'forward'
    if fault == 'disabled':
        args.detection_grace = 0.
    clock = [100.]
    published, events = [], []
    goal = (0., 0., 0.)
    memory = GoalMemory()
    memory.accept(goal, 101.2, 11_200_000_000)
    def robot():
        if fault == 'tf' and clock[0] > 101.3:
            raise RuntimeError('TF lost')
        return (.8, 0., 0.)
    node = SimpleNamespace(
        args=args, active_motion=False, actions={}, goal_memory=memory,
        emit=lambda event, **data: events.append((clock[0], event, data)),
        robot=robot, monitor_active=lambda: not (fault == 'monitor' and clock[0] > 101.3),
        detected_goal=lambda: goal if clock[0] <= 101.2 else None,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(
            nanoseconds=int((10+clock[0]-100)*1e9), clock_type=ClockType.ROS_TIME)),
        publish=lambda v=0., w=0.: published.append((clock[0], v, w)))
    def spin(*a, **k):
        clock[0] += .01  # 100 Hz callback traffic must not become 100 Hz commands.
        if fault == 'action' and clock[0] > 101.3:
            node.actions['navigate_to_pose'] = True
    with patch('dock_detector.controller.time.monotonic', side_effect=lambda: clock[0]), \
         patch('dock_detector.controller.rclpy.spin_once', side_effect=spin), \
         patch('dock_detector.controller.rclpy.ok', return_value=True):
        with pytest.raises(RuntimeError):
            DockController.run(node, lambda: clock[0] >= 106.)
    assert published
    assert all(b[0]-a[0] >= .049 for a, b in zip(published, published[1:]))
    if fault == 'none':
        assert any(t > 102 and v < 0 for t, v, w in published)
        assert all(v == 0 and w == 0 for t, v, w in published if t > 104.71)
        assert any(e == 'detection_grace_started' for _, e, _ in events)
    else:
        assert all(v == 0 and w == 0 for t, v, w in published if t > 101.36)


def test_isolated_pose_after_gap_does_not_refresh_memory():
    from collections import deque
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    now_ros = [10.]
    node = SimpleNamespace(args=config(), samples=deque(maxlen=5),
                           last_pose_stamp=None, last_pose_wall=None, goal=None,
                           goal_memory=GoalMemory(),
                           buffer=SimpleNamespace(lookup_transform=lambda *a: tf),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(
                               seconds=now_ros[0], clock_type=ClockType.ROS_TIME)),
                           emit=lambda *a, **k: None)
    msg = PoseStamped()
    msg.header.frame_id = 'odom'
    # Inward Nav2 dock yaw pi corresponds to reverse parked robot yaw zero.
    msg.pose.orientation.z = 1.
    msg.pose.orientation.w = 0.
    with patch('dock_detector.controller.time.monotonic', return_value=100.):
        for i in range(5):
            msg.header.stamp = Time(seconds=9.8+i*.01).to_msg()
            DockController.on_pose(node, msg)
    assert node.goal_memory.wall == 100.
    now_ros[0] = 11.
    with patch('dock_detector.controller.time.monotonic', return_value=101.):
        msg.header.stamp = Time(seconds=10.9).to_msg()
        DockController.on_pose(node, msg)
    assert node.goal is None and len(node.samples) == 1
    assert node.goal_memory.wall == 100.
    assert node.goal_memory.lookup(102., 12_000_000_000, .5, 3.) == pytest.approx((0., 0., 0.))
    assert node.goal_memory.lookup(103.6, 13_600_000_000, .5, 3.) is None
