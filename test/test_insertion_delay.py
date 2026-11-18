"""Verify charging-triggered stop timing using a simulated insertion clock.

Run from the workspace root after sourcing ROS and the workspace:
    PYTHONPATH=src/dock_detector:$PYTHONPATH python3 -m pytest -q src/dock_detector/test/test_insertion_delay.py

Expect immediate default stopping, delayed stopping, and earlier travel-limit
stopping. The harness supplies odometry and battery status without robot motion.
"""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from geometry_msgs.msg import Twist
from nav2_msgs.action import DockRobot
from rclpy.time import Time
from sensor_msgs.msg import BatteryState

from dock_detector.docking_alignment import AlignmentSample, DockingAlignment


def simulate_insertion(delay, *, flicker=False, near_limit=False, dock_id='real_v_bay'):
    clock = [0.0]
    commands = []
    wrapper = Mock()
    wrapper.settings = {
        'insertion_timeout': 5.0,
        'insertion_stop_margin': .01, 'insertion_speed': .01, 'insertion_min_speed': 0.0,
        'docking_threshold_yaw': 5.0,
    }
    wrapper.charging_stop_delays = {'real_v_bay': delay, 'other_bay': 0.0}
    wrapper.ensure_idle.return_value = True
    wrapper.current_twist.return_value = Twist()
    wrapper.robot_pose.side_effect = lambda: (
        .095 if near_limit and clock[0] >= .1 else 0., 0., 0.)
    wrapper.battery_status.side_effect = lambda: (
        BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if flicker and .1 <= clock[0] < .3
        else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    wrapper.get_clock.return_value.now.return_value = Time()
    wrapper.publish.side_effect = lambda linear=0.: commands.append((clock[0], linear))
    aligned = AlignmentSample(
        robot=(0., 0.), detected=(0., 0.), filtered=(0., 0.),
        scan_stamp=Time(), robot_yaw=0., detected_yaw_error=0., yaw_error=0.)
    goal = SimpleNamespace(request=DockRobot.Goal(dock_id=dock_id),
                           publish_feedback=Mock())

    def advance(seconds):
        clock[0] += seconds

    with patch('dock_detector.docking_alignment.monotonic', side_effect=lambda: clock[0]), \
            patch('dock_detector.docking_alignment.sleep', side_effect=advance):
        DockingAlignment.insert(wrapper, goal, Time(), DockRobot.Result(), Mock(), aligned, .1)
    return commands


def test_default_stops_immediately_on_charging():
    assert all(speed == 0. for _, speed in simulate_insertion(0.))


@pytest.mark.parametrize('flicker', [False, True])
def test_delay_starts_on_first_charging_report(flicker):
    commands = simulate_insertion(.5, flicker=flicker)
    assert all(speed > 0. for when, speed in commands if when < .5)
    first_stop = next(when for when, speed in commands if speed == 0.)
    assert .5 <= first_stop <= .55
    assert all(speed == 0. for when, speed in commands if when >= first_stop)


def test_travel_limit_stops_before_charging_delay():
    commands = simulate_insertion(1., near_limit=True)
    first_stop = next(when for when, speed in commands if speed == 0.)
    assert first_stop == pytest.approx(.1)


@pytest.mark.parametrize('delay', [-.1, float('nan'), float('inf')])
def test_delay_must_be_finite_and_nonnegative(delay):
    with patch('rclpy.node.Node.__init__', return_value=None), \
            patch.object(DockingAlignment, 'declare_parameter',
                         side_effect=lambda name, value: SimpleNamespace(
                             value=['real_v_bay'] if name == 'dock_ids'
                             else delay if name == 'docks.real_v_bay.charging_to_stop_delay'
                             else value)):
        with pytest.raises(ValueError, match='charging_to_stop_delay'):
            DockingAlignment()


@pytest.mark.parametrize('dock_id', ['other_bay', 'unconfigured_bay'])
def test_other_bays_use_their_own_delay_or_zero_default(dock_id):
    assert all(speed == 0. for _, speed in simulate_insertion(.5, dock_id=dock_id))
