import math
import numpy as np
from dock_detector.geometry import detect_v, angle_error


def target(apex=(0., 0.), yaw=0., noise=0., gap=0.02):
    rng = np.random.default_rng(9)
    points = []
    for angle in (yaw-math.pi/4, yaw+math.pi/4):
        d = np.array([math.cos(angle), math.sin(angle)])
        arm = np.asarray(apex)+np.linspace(gap, .45, 40)[:, None]*d
        points.extend(arm+rng.normal(0, noise, arm.shape))
    return np.array(points)


def test_noisy_rotated_v_with_outliers():
    points = target((2., -1.), 1.2, .006)
    points = np.vstack([points, np.random.default_rng(7).uniform([-1,-3], [4,2], (25,2))])
    result = detect_v(points, (2.05, -.97), 1.15)
    assert result is not None
    assert np.linalg.norm(result.apex-[2,-1]) < .025
    assert abs(angle_error(result.yaw, 1.2)) < .04


def test_rear_scan_gap_and_unordered_points():
    # Missing vertex observations, as occurs across Seggy's rear blind sector.
    points = target((0., 0.), 0., .005, gap=.16)
    result = detect_v(points[::-1], (.01, 0.), 0.)
    assert result is not None
    assert np.linalg.norm(result.apex) < .025


def test_reject_single_wall_empty_and_invalid():
    for points in (np.empty((0,2)), np.full((20,2), np.nan), target()[:40]):
        assert detect_v(points, (0.,0.), 0.) is None


def test_reject_wrong_location_and_facing():
    assert detect_v(target((2,2)), (0.,0.), 0.) is None
    assert detect_v(target(yaw=math.pi), (0.,0.), 0.) is None


def test_reject_wrong_opening_and_too_short():
    r = np.linspace(.02, .4, 40)
    points = np.vstack([np.column_stack([r, r*.2]), np.column_stack([r, -r*.2])])
    assert detect_v(points, (0.,0.), 0.) is None
    assert detect_v(target()*.1, (0.,0.), 0.) is None


def test_partial_face_lost():
    assert detect_v(target()[:43], (0.,0.), 0.) is None
