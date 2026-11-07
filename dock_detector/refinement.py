"""Bounded robust SE(2) fitting of three finite bay faces using NumPy only."""
import math
import numpy as np
from .geometry import Detection, angle_error


def template(width, depth, flare):
    rear = np.array([[0., -width/2], [0., width/2]])
    return np.array([rear, [rear[0], [depth, -width/2-depth*math.tan(flare)]],
                     [rear[1], [depth, width/2+depth*math.tan(flare)]]])


def transform(segments, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return segments @ np.array([[c, s], [-s, c]]) + pose[:2]


def projections(points, segments):
    edge = segments[:, 1]-segments[:, 0]
    delta = points[:, None, :]-segments[None, :, 0, :]
    t = np.clip(np.sum(delta*edge, axis=2)/np.sum(edge*edge, axis=1), 0., 1.)
    closest = segments[None, :, 0, :]+t[:, :, None]*edge
    return closest, t


def refine_bay(points, seed, *, width, depth, flare, threshold=.025,
               min_points=5, min_length=.12, max_translation=.05,
               max_yaw=.10, max_iterations=20):
    """Fit x/y/yaw jointly; dimensions fixed, initial panel assignments fixed.

    Point-to-finite-segment residuals, Huber weights and equal panel weighting
    limit clutter influence. No interpolation, history, or synthesized returns.
    Returns (detection or None, diagnostic metrics).
    """
    local = template(width, depth, flare)
    initial = np.array([*seed.apex, seed.yaw], dtype=float)
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    closest, _ = projections(points, transform(local, initial))
    distances = np.linalg.norm(points[:, None, :]-closest, axis=2)
    labels = np.argmin(distances, axis=1)
    keep = distances[np.arange(len(points)), labels] <= 2*threshold
    pts, labels = points[keep], labels[keep]
    counts = np.bincount(labels, minlength=3)
    metrics = {'refinement': 'rejected', 'selected_points': len(pts)}
    if np.any(counts < min_points):
        return None, {**metrics, 'reason': 'refinement_insufficient_panel_support'}
    # Freeze assignments and selected points so optimization cannot lower cost
    # simply by discarding measurements or switching walls.
    panel_weight = 1./counts[labels]
    huber = threshold/2

    def residual(pose):
        closest, _ = projections(pts, transform(local, pose))
        return pts-closest[np.arange(len(pts)), labels]

    def cost(r):
        norm = np.linalg.norm(r, axis=1)
        return float(np.sum(panel_weight*np.where(norm <= huber,
                         .5*norm**2, huber*(norm-.5*huber))))

    def jacobian(pose):
        eps = 1e-5
        return np.stack([(residual(pose+np.eye(3)[j]*eps)-
                          residual(pose-np.eye(3)[j]*eps))/(2*eps)
                         for j in range(3)], axis=2).reshape(-1, 3)

    pose = initial.copy()
    r = residual(pose)
    before = float(np.sqrt(np.mean(np.sum(r*r, axis=1))))
    converged = False
    iterations = 0
    for iterations in range(1, max_iterations+1):
        norm = np.linalg.norm(r, axis=1)
        weights = np.repeat(np.sqrt(panel_weight*np.minimum(1., huber/np.maximum(norm, 1e-12))), 2)
        jac = jacobian(pose)
        step = np.linalg.lstsq(jac*weights[:, None], -r.ravel()*weights, rcond=None)[0]
        if np.linalg.norm(step) < 1e-6:
            converged = True
            break
        improved = False
        for scale in (1., .5, .25, .125, .0625):
            candidate = pose+scale*step
            delta = candidate-initial
            if np.linalg.norm(delta[:2]) > max_translation or abs(delta[2]) > max_yaw:
                continue
            new_r = residual(candidate)
            if cost(new_r) < cost(r):
                pose, r = candidate, new_r
                improved = True
                break
        if not improved:
            # A stalled fit is usable only when the remaining step is tiny.
            converged = np.linalg.norm(step) < 1e-4
            break
    segments = transform(local, pose)
    closest, along = projections(points, segments)
    distance = np.linalg.norm(points[:, None, :]-closest, axis=2)
    nearest = np.argmin(distance, axis=1)
    inlier = distance[np.arange(len(points)), nearest] <= threshold
    support = []
    for panel in range(3):
        mask = inlier & (nearest == panel)
        n = int(mask.sum())
        length = float(np.linalg.norm(local[panel, 1]-local[panel, 0]))
        span = float(np.ptp(along[mask, panel])*length) if n > 1 else 0.
        support.append({'points': n, 'span_m': span, 'coverage': span/length})
    rms = float(np.sqrt(np.mean(distance[np.arange(len(points))[inlier], nearest[inlier]]**2))) if inlier.any() else None
    # Metre-equivalent heading column uses characteristic panel length.
    scaled_jac = jacobian(pose)*np.array([1., 1., 1./max(width, depth)])
    singular = np.linalg.svd(scaled_jac, compute_uv=False)
    condition = float(singular[0]/max(singular[-1], 1e-12))
    metrics.update(initial_rms_m=before, final_rms_m=rms, iterations=iterations,
                   converged=converged, condition=condition, panels=support,
                   inlier_fraction=float(inlier.mean()) if len(points) else 0.,
                   correction_m=float(np.linalg.norm(pose[:2]-initial[:2])),
                   correction_yaw_rad=float(angle_error(pose[2], initial[2])))
    reason = None
    if not converged:
        reason = 'refinement_not_converged'
    elif condition > 100.:
        reason = 'refinement_degenerate'
    elif any(p['points'] < min_points or p['span_m'] < min_length for p in support):
        reason = 'refinement_insufficient_panel_support'
    elif rms is None or rms > threshold*.75:
        reason = 'refinement_high_residual'
    if reason:
        return None, {**metrics, 'reason': reason}
    confidence = math.exp(-.5*(rms/(threshold/2))**2)
    confidence *= min(1., min(p['points'] for p in support)/(2*min_points))
    confidence *= min(p['coverage'] for p in support)
    metrics.update(refinement='accepted', reason='matched', confidence=float(confidence))
    return Detection(pose[:2], float(angle_error(pose[2], 0.)), tuple(segments), rms), metrics
