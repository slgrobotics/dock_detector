"""Joint fitting improves a perturbed pose without changing bay dimensions."""
import math
import numpy as np
import pytest
from dock_detector.geometry import Detection, detect_trapezoid, angle_error
from dock_detector.refinement import template, transform, refine_bay

WIDTH = .5
DEPTH = .5*math.cos(math.radians(32.5))
FLARE = math.radians(32.5)


def observations(pose, noise=.004, flare=FLARE):
    rng = np.random.default_rng(87)
    segments = transform(template(WIDTH, .5*math.cos(flare), flare), np.array(pose))
    pts = np.vstack([a+(b-a)*np.linspace(.03, .97, n)[:, None]
                     for (a, b), n in zip(segments, (30, 45, 18))])
    return pts+rng.normal(0, noise, pts.shape)


@pytest.mark.parametrize('flare, max_position_error', [(math.pi/6, .006), (math.pi/4, .010)])
def test_joint_fit_corrects_pose_with_clutter_and_preserves_lengths(flare, max_position_error):
    truth = np.array([2., -1., .6])
    initial = truth+np.array([.018, -.012, .04])
    points = observations(truth, flare=flare)
    # A few near-panel outliers plus unrelated distant clutter.
    points = np.vstack([points, points[:3]+[.025, .015], [[4., 4.], [-3., -2.]]])
    seed = Detection(initial[:2], initial[2], (), .02)
    fit, metrics = refine_bay(points, seed, width=WIDTH, depth=.5*math.cos(flare), flare=flare)
    assert fit is not None, metrics
    assert np.linalg.norm(fit.apex-truth[:2]) < max_position_error
    assert abs(angle_error(fit.yaw, truth[2])) < abs(initial[2]-truth[2])/2
    assert metrics['final_rms_m'] < metrics['initial_rms_m']
    assert 0 < metrics['confidence'] <= 1
    for a, b in fit.arms:
        assert abs(np.linalg.norm(b-a)-.5) < 1e-10


def test_refinement_rejects_missing_panels():
    pts = observations((0., 0., 0.))[:30]
    seed = Detection(np.zeros(2), 0., (), 0.)
    fit, metrics = refine_bay(pts, seed, width=WIDTH, depth=DEPTH, flare=FLARE)
    assert fit is None
    assert metrics['reason'] == 'refinement_insufficient_panel_support'


def test_match_diagnostics_and_baseline_switch():
    points = observations((0., 0., 0.))
    stats = {}
    fit = detect_trapezoid(points, (0., 0.), 0., diagnostics=stats)
    assert fit is not None, stats
    assert stats['reason'] == 'matched'
    assert stats['refinement'] == 'accepted'
    assert len(stats['panels']) == 3
    seed = detect_trapezoid(points, (0., 0.), 0., joint_refinement=False, diagnostics=stats)
    assert seed is not None
    assert stats['refinement'] == 'disabled'
    assert stats['confidence'] is None
    assert detect_trapezoid(points[:30], (0., 0.), 0., diagnostics=stats) is None
    assert stats['reason'] == 'insufficient_lines'
    assert stats['confidence'] == 0.
    assert stats['line_count'] < 3
