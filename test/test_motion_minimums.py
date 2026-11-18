"""Verify docking speed floors preserve explicit stops and speed ceilings.

Run from the workspace root without a robot:
    PYTHONPATH=src/dock_detector python3 -m pytest -q src/dock_detector/test/test_motion_minimums.py

Expect signed corrections below the floor to use the minimum, commands above
it to pass unchanged up to the maximum, and explicit zero to remain zero.
"""
import pytest
from dock_detector.core.pose_controller import moving_speed


@pytest.mark.parametrize('requested,minimum,maximum,expected', [
    (0., .0406, .0406, 0.),
    (.001, .0406, .0406, .0406),
    (-.001, .1715, .1716, -.1715),
    (.2, .1715, .5, .2),
    (-1., .1715, .5, -.5),
    (.001, 0., .05, .001),
])
def test_moving_speed(requested, minimum, maximum, expected):
    assert moving_speed(requested, minimum, maximum) == pytest.approx(expected)
