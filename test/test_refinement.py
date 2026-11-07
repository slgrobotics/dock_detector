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


def test_v_joint_fit_corrects_pose_and_keeps_equal_panels():
    from dock_detector.refinement import v_template, refine_v
    truth=np.array([1., -2., .7])
    local=v_template(.7,math.radians(55))
    panels=transform(local,truth)
    rng=np.random.default_rng(4)
    points=np.vstack([a+(b-a)*np.linspace(.08,.9,n)[:,None] for (a,b),n in zip(panels,(35,18))])
    points+=rng.normal(0,.004,points.shape)
    seed=Detection(truth[:2]+[.01,-.012],truth[2]+.025,(),.02)
    result,stats=refine_v(points,seed,panel_width=.7,flare=math.radians(55))
    assert result is not None,stats
    assert np.linalg.norm(result.apex-truth[:2])<.007
    assert abs(angle_error(result.yaw,truth[2]))<.01
    assert len(stats['panels'])==2 and 0<stats['confidence']<=1
    assert stats['correction_m']<=.05 and abs(stats['correction_yaw_rad'])<=.10
    for a,b in result.arms:assert np.linalg.norm(b-a)==pytest.approx(.7)
    assert refine_v(points[:35],seed,panel_width=.7,flare=math.radians(55))[0] is None


def test_v_detector_refinement_switch_and_prior_guard():
    from dock_detector.geometry import detect_v
    from dock_detector.refinement import v_template
    rng=np.random.default_rng(8)
    points=np.vstack([a+(b-a)*np.linspace(.04,.95,40)[:,None] for a,b in v_template(.7,math.radians(55))])
    points+=rng.normal(0,.003,points.shape)
    stats={}
    assert detect_v(points,(0,0),0,panel_width=.7,joint_refinement=True,diagnostics=stats) is not None
    assert stats['refinement']=='accepted' and len(stats['panels'])==2
    assert detect_v(points,(0,0),0,panel_width=.7,joint_refinement=False,diagnostics=stats) is not None
    assert stats['refinement']=='disabled'
    assert detect_v(points[:40],(0,0),0,panel_width=.7,joint_refinement=True) is None
    assert detect_v(points,(0,0),math.pi,panel_width=.7,joint_refinement=True) is None
