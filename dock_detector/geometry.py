"""Planar dock landmark matching independent of ROS."""
from dataclasses import dataclass
from itertools import combinations
import math
import numpy as np


@dataclass
class Detection:
    apex: np.ndarray
    yaw: float
    arms: tuple
    rms: float


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def fit_line(points):
    center = points.mean(axis=0)
    _, _, axes = np.linalg.svd(points-center, full_matrices=False)
    direction = axes[0]
    normal = np.array([-direction[1], direction[0]])
    return center, direction, normal


def extract_lines(points, *, distance_threshold=0.025, min_points=5,
                  min_length=0.12, max_length=1.3, max_lines=7, max_line_gap=0.10):
    """Extract supported finite segments, without depending on scan ordering."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    remaining = points.copy()
    lines = []
    rng = np.random.default_rng(0)
    for _ in range(max_lines):
        if len(remaining) < min_points:
            break
        best = None
        for _ in range(160):
            a, b = remaining[rng.choice(len(remaining), 2, replace=False)]
            d = b-a
            length = np.linalg.norm(d)
            if length < min_length/2:
                continue
            normal = np.array([-d[1], d[0]]) / length
            mask = np.abs((remaining-a) @ normal) <= distance_threshold
            count = int(mask.sum())
            if best is None or count > best[0]:
                best = count, mask
        if best is None or best[0] < min_points:
            break
        mask = best[1]
        center, direction, normal = fit_line(remaining[mask])
        mask = np.abs((remaining-center) @ normal) <= distance_threshold
        # A distant collinear clutter point must not enlarge a short dock arm.
        indices = np.flatnonzero(mask)
        order = indices[np.argsort((remaining[indices]-center) @ direction)]
        along = (remaining[order]-center) @ direction
        groups = np.split(order, np.flatnonzero(np.diff(along) > max_line_gap)+1)
        selected = max(groups, key=len)
        mask = np.zeros(len(remaining), dtype=bool)
        mask[selected] = True
        support = remaining[mask]
        if len(support) < min_points:
            break
        center, direction, normal = fit_line(support)
        projection = (support-center) @ direction
        span = np.ptp(projection)
        rms = float(np.sqrt(np.mean(((support-center) @ normal)**2)))
        if min_length <= span <= max_length:
            lines.append((center, direction, normal, support, rms))
        remaining = remaining[~mask]
    return lines


def detect_v(points, expected_apex, expected_yaw, *, distance_threshold=0.025,
             min_points=5, min_length=0.12, max_length=1.30,
             expected_angle=math.radians(110), angle_tolerance=0.20,
             position_tolerance=0.45, yaw_tolerance=0.45,
             max_apex_gap=0.25, max_lines=5):
    """RANSAC + total least squares; reject unsupported/wrong-facing pairs.

    Returned yaw points from the V vertex toward its open side. Input order does
    not matter, so a rear-facing V may straddle a scan boundary or blind sector.
    """
    lines = extract_lines(points, distance_threshold=distance_threshold,
                          min_points=min_points, min_length=min_length,
                          max_length=max_length, max_lines=max_lines)
    matches = []
    for a, b in combinations(lines, 2):
        matrix = np.stack([a[2], b[2]])
        if abs(np.linalg.det(matrix)) < 0.2:
            continue
        apex = np.linalg.solve(matrix, np.array([a[2] @ a[0], b[2] @ b[0]]))
        error = np.linalg.norm(apex-np.asarray(expected_apex))
        if error > position_tolerance:
            continue
        rays = []
        valid = True
        endpoints = []
        for center, direction, normal, support, rms in (a, b):
            if np.dot(center-apex, direction) < 0:
                direction = -direction
            along = (support-apex) @ direction
            # Both arms must extend outward from a nearby common vertex.
            if along.min() < -distance_threshold*2 or along.min() > max_apex_gap:
                valid = False
                break
            rays.append(direction)
            endpoints.append((apex+direction*along.min(), apex+direction*along.max()))
        if not valid:
            continue
        opening = math.acos(float(np.clip(np.dot(*rays), -1, 1)))
        if abs(opening-expected_angle) > angle_tolerance:
            continue
        bisector = rays[0]+rays[1]
        yaw = math.atan2(bisector[1], bisector[0])
        heading_error = abs(angle_error(yaw, expected_yaw))
        if heading_error > yaw_tolerance:
            continue
        rms = max(a[4], b[4])
        matches.append((error+heading_error*0.2+rms, Detection(apex, yaw, tuple(endpoints), rms)))
    return min(matches, key=lambda x: x[0])[1] if matches else None


def _detect_trapezoid_seed(points, expected_center, expected_yaw, *, width, depth, flare_angle,
                     dimension_tolerance=0.10,
                     distance_threshold=0.025, min_points=5, min_length=0.12,
                     max_length=1.30, angle_tolerance=0.20,
                     position_tolerance=0.45, yaw_tolerance=0.45,
                     max_corner_gap=0.18, max_line_gap=0.30, diagnostics=None):
    """Three-sided trapezoid seed matcher; require back AND both side faces.

    Detection.apex is the midpoint of the two inner rear corners.
    The yaw points from the back wall toward the open side. A lone rear wall or
    L-shaped room corner is deliberately insufficient for a detection.
    """
    if not math.isfinite(flare_angle) or not 0 < flare_angle < math.pi/2:
        raise ValueError('flare_angle must be in (0, pi/2)')
    lines = extract_lines(points, distance_threshold=distance_threshold,
                          min_points=min_points, min_length=min_length,
                          max_length=max_length, max_line_gap=max_line_gap)
    if diagnostics is not None:
        diagnostics.update(reason='no_geometric_candidate', line_count=len(lines),
                           roi_points=len(points), candidates=0, rejections={})
    def reject(reason):
        if diagnostics is not None:
            counts = diagnostics['rejections']
            counts[reason] = counts.get(reason, 0)+1
    matches = []
    for back_index, back in enumerate(lines):
        center, tangent, normal, support, _ = back
        forward = normal.copy()
        if np.dot(forward, [math.cos(expected_yaw), math.sin(expected_yaw)]) < 0:
            forward = -forward
        yaw = math.atan2(forward[1], forward[0])
        if abs(angle_error(yaw, expected_yaw)) > yaw_tolerance:
            reject('rear_heading')
            continue
        lateral = np.array([-forward[1], forward[0]])
        back_span = np.ptp((support-center) @ lateral)
        if not width-2*max_corner_gap <= back_span <= width+dimension_tolerance:
            reject('rear_span')
            continue
        sides = []
        for i, side in enumerate(lines):
            if i == back_index:
                continue
            c, d, n, pts, rms = side
            if abs(math.acos(float(np.clip(abs(d @ forward), 0, 1)))-flare_angle) > angle_tolerance:
                reject('side_angle')
                continue
            matrix = np.stack([normal, n])
            if abs(np.linalg.det(matrix)) < .1:
                reject('parallel_faces')
                continue
            corner = np.linalg.solve(matrix, [normal @ center, n @ c])
            along = (pts-corner) @ forward
            if (along.min() < -2*distance_threshold or along.min() > max_corner_gap
                    or along.max() > depth+dimension_tolerance
                    or along.max() < depth-max_corner_gap):
                reject('side_extent_or_corner_gap')
                continue
            sides.append((side, corner, along))
        for a, b in combinations(sides, 2):
            measured_width = abs(float((a[1]-b[1]) @ lateral))
            if abs(measured_width-width) > dimension_tolerance:
                reject('rear_width')
                continue
            landmark = (a[1]+b[1])/2
            # Orient each fitted side toward the opening, then require its
            # lateral slope to point OUT from its own rear corner.
            outward = True
            for side, corner, along in (a, b):
                direction = side[1] if side[1] @ forward > 0 else -side[1]
                sign = 1. if (corner-landmark) @ lateral > 0 else -1.
                measured_flare = math.atan2(sign*float(direction @ lateral),
                                            float(direction @ forward))
                if abs(measured_flare-flare_angle) > angle_tolerance:
                    outward = False
            if not outward:
                reject('inward_or_wrong_flare')
                continue
            error = float(np.linalg.norm(landmark-np.asarray(expected_center)))
            if error > position_tolerance:
                reject('association_position')
                continue
            rear_along = (support-landmark) @ lateral
            if (rear_along.min() < -width/2-dimension_tolerance
                    or rear_along.max() > width/2+dimension_tolerance
                    or rear_along.min() > -width/2+max_corner_gap
                    or rear_along.max() < width/2-max_corner_gap):
                reject('rear_corner_support')
                continue
            segments = [(landmark+lateral*rear_along.min(),
                         landmark+lateral*rear_along.max())]
            for side, corner, along in (a, b):
                direction = side[1] / float(side[1] @ forward)
                segments.append((corner+direction*along.min(), corner+direction*along.max()))
            rms = max(back[4], a[0][4], b[0][4])
            score = error+abs(measured_width-width)+rms
            matches.append((score, Detection(landmark, yaw, tuple(segments), rms)))
    if diagnostics is not None:
        diagnostics['candidates'] = len(matches)
        diagnostics['reason'] = 'geometric_match' if matches else (
            'insufficient_lines' if len(lines) < 3 else 'no_geometric_candidate')
    return min(matches, key=lambda x: x[0])[1] if matches else None


def detect_trapezoid(points, expected_center, expected_yaw, *, width=0.5,
                     depth=0.5*math.cos(math.radians(32.5)), flare_angle=math.radians(32.5), max_line_gap=0.30,
                     joint_refinement=True, diagnostics=None, **kwargs):
    """Match rear wall and two outward-flared sides.

    Width is inner rear width, depth is projected forward distance (not side
    length), and flare_angle is radians outward from the forward axis per side.
    Landmark is the rear-face midpoint; yaw points toward the opening.
    """
    stats = diagnostics if diagnostics is not None else {}
    stats.clear()
    stats['confidence'] = 0.
    seed = _detect_trapezoid_seed(points, expected_center, expected_yaw, width=width,
                       depth=depth, flare_angle=flare_angle, max_line_gap=max_line_gap,
                       diagnostics=stats, **kwargs)
    if seed is None:
        return None
    stats.update(seed_xy=seed.apex.tolist(), seed_yaw=seed.yaw, seed_rms_m=seed.rms)
    if not joint_refinement:
        stats.update(reason='matched', refinement='disabled', confidence=None)
        return seed
    from .refinement import refine_bay
    result, metrics = refine_bay(points, seed, width=width, depth=depth, flare=flare_angle,
                                threshold=kwargs.get('distance_threshold', .025),
                                min_points=kwargs.get('min_points', 5),
                                min_length=kwargs.get('min_length', .12))
    stats.update(metrics)
    if result is not None and (
            np.linalg.norm(result.apex-np.asarray(expected_center)) > kwargs.get('position_tolerance', .45)
            or abs(angle_error(result.yaw, expected_yaw)) > kwargs.get('yaw_tolerance', .45)):
        stats.update(reason='refined_pose_outside_prior', confidence=0.)
        return None
    return result
