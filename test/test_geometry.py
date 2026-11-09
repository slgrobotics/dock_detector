import math
import numpy as np
from dock_detector.core.geometry import detect_v, angle_error


def target(apex=(0., 0.), yaw=0., noise=0., gap=0.02, flare=math.radians(55)):
    rng = np.random.default_rng(9)
    points = []
    for angle in (yaw-flare, yaw+flare):
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
        assert detect_v(points, (0.,0.), 0., expected_angle=math.radians(65)) is None


def test_reject_wrong_location_and_facing():
    assert detect_v(target((2,2)), (0.,0.), 0.) is None
    assert detect_v(target(yaw=math.pi), (0.,0.), 0.) is None


def test_reject_wrong_opening_and_too_short():
    r = np.linspace(.02, .4, 40)
    points = np.vstack([np.column_stack([r, r*.2]), np.column_stack([r, -r*.2])])
    assert detect_v(points, (0.,0.), 0., expected_angle=math.radians(65)) is None
    assert detect_v(target()*.1, (0.,0.), 0.) is None


def test_partial_face_lost():
    assert detect_v(target()[:43], (0.,0.), 0.) is None


def test_sixty_five_degree_v_supported_when_configured():
    result = detect_v(target(flare=math.radians(32.5)), (0., 0.), 0.,
                      expected_angle=math.radians(65))
    assert result is not None
    assert np.linalg.norm(result.apex) < .01


def test_v_does_not_accept_displaced_trapezoid_virtual_apex():
    from dock_detector.core.refinement import template
    faces = template(.5, .5*math.cos(math.radians(32.5)), math.radians(32.5))
    points = np.vstack([a+(b-a)*np.linspace(.02,.98,40)[:,None] for a,b in faces])
    assert detect_v(points, (0.,0.), 0., expected_angle=math.radians(65)) is None


def test_v_panel_width_allows_partial_support_but_rejects_excess_length():
    points = target()
    assert detect_v(points, (0., 0.), 0., panel_width=.7) is not None
    assert detect_v(points, (0., 0.), 0., panel_width=.2) is None
