"""Trapezoidal template tests, including angularly sampled world geometry."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from dock_detector.core.geometry import detect_trapezoid, angle_error


def bay(width=.5, depth=.5*math.cos(math.radians(32.5)), flare=math.radians(32.5), yaw=0., center=(0., 0.), noise=.003):
    x = np.linspace(.01, depth, 65)
    faces = [np.column_stack([np.zeros(100), np.linspace(-width/2, width/2, 100)])]
    for sign in (-1, 1):
        faces.append(np.column_stack([x, sign*(width/2+x*math.tan(flare))]))
    r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    rng = np.random.default_rng(51)
    return [f@r.T+center+rng.normal(0, noise, f.shape) for f in faces]


def test_rotated_noisy_shuffled_with_clutter():
    points = np.vstack(bay(center=(2., -1.), yaw=.7)+[
        np.random.default_rng(6).uniform([-1, -3], [4, 3], (30, 2))])
    np.random.default_rng(7).shuffle(points)
    result = detect_trapezoid(points, (2.04, -1.03), .72)
    assert result is not None
    assert np.linalg.norm(result.apex-[2, -1]) < .025
    assert abs(angle_error(result.yaw, .7)) < .03
    assert len(result.arms) == 3
    # Debug segments must follow the sloped faces, not parallel forward lines.
    for start, end in result.arms[1:]:
        d = end-start
        assert abs(abs(angle_error(math.atan2(d[1], d[0]), result.yaw))-math.radians(32.5)) < .04


def test_reject_missing_faces_and_wrong_templates():
    faces = bay()
    for subset in ([], faces[:1], faces[:2], faces[1:]):
        points = np.vstack(subset) if subset else np.empty((0, 2))
        assert detect_trapezoid(points, (0, 0), 0) is None
    for kw in ({'flare': 0.}, {'flare': -math.radians(32.5)}, {'flare': math.pi/3},
               {'width': 1.4}, {'depth': .8}, {'yaw': math.pi}, {'center': (2, 2)}):
        assert detect_trapezoid(np.vstack(bay(**kw)), (0, 0), 0) is None


def world_inner_faces():
    world = Path(__file__).resolve().parents[2]/'articubot_one/assets/worlds/test_robot_world.sdf'
    root = ET.parse(world)
    faces = []
    for number in (2, 1, 3):
        model = root.find(f".//model[@name='dock-bay-wall-{number}']")
        pose = list(map(float, model.findtext('pose').split()))
        size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
        yaw = pose[5]
        tangent = np.array([math.cos(yaw), math.sin(yaw)])
        normal = np.array([-math.sin(yaw), math.cos(yaw)])
        # Back/bottom use +normal inner face; upper uses -normal.
        sign = -1 if number == 3 else 1
        # Back tangent is +Y, so inner face is on -normal (+X).
        if number == 2:
            sign = -1
        center = np.array(pose[:2])+sign*size[1]/2*normal
        faces.append((center-size[0]/2*tangent, center+size[0]/2*tangent))
    return faces


def cross(a, b):
    return a[0]*b[1]-a[1]*b[0]


@pytest.mark.parametrize('robot, expected_match', [
    ((0., 3.), True), ((.8, 3.), True),
    ((.8, 2.8), False), ((.8, 3.2), False), ((1.3, 3.), True)])
def test_sdf_faces_with_angular_lidar_sampling(robot, expected_match):
    # Nearest-hit 0.5-degree rays, 0.3 m minimum range, 5-degree rear gap.
    origin = np.array(robot)
    hits = []
    faces = world_inner_faces()
    for theta in np.arange(-math.pi+math.radians(2.5), math.pi-math.radians(2.5), math.radians(.5)):
        ray = np.array([math.cos(theta), math.sin(theta)])
        distances = []
        for start, end in faces:
            edge = end-start
            denom = cross(ray, edge)
            if abs(denom) < 1e-9:
                continue
            distance = cross(start-origin, edge)/denom
            u = cross(start-origin, ray)/denom
            if distance >= .3 and 0 <= u <= 1:
                distances.append(distance)
        if distances:
            hits.append(origin+min(distances)*ray)
    points = np.array(hits)
    # Match the node's configured region of interest.
    points = points[np.linalg.norm(points-[-1.425, 3.], axis=1) <= .95]
    result = detect_trapezoid(points, (-1.425, 3.), 0.,
                              dimension_tolerance=.05, max_corner_gap=.10)
    # At the offset poses the rear blind sector removes corner support.
    # Preserve rejection of those scans.
    assert (result is not None) == expected_match
    if not expected_match:
        return
    assert np.linalg.norm(result.apex-[-1.425, 3.]) < .025
    assert abs(result.yaw) < .03


def test_sdf_inner_dimensions():
    rear, lower, upper = world_inner_faces()
    rear_x = float(np.mean(np.array(rear)[:, 0]))
    assert abs(rear_x+1.425) < 1e-9
    corners = []
    ends = []
    for start, end in (lower, upper):
        direction = end-start
        if direction[0] < 0:
            start, end = end, start
            direction = -direction
        assert abs(abs(math.atan2(direction[1], direction[0]))-math.radians(32.5)) < 1e-9
        corners.append(start+(rear_x-start[0])/direction[0]*direction)
        ends.append(end)
    assert abs(abs(corners[1][1]-corners[0][1])-.5) < 1e-9
    assert abs(abs(ends[1][1]-ends[0][1])-(.5+math.sin(math.radians(32.5)))) < 1e-9
    assert abs(ends[0][0]-rear_x-.5*math.cos(math.radians(32.5))) < 1e-9


def test_world_uses_three_half_metre_panels():
    world = Path(__file__).resolve().parents[2]/'articubot_one/assets/worlds/test_robot_world.sdf'
    root = ET.parse(world)
    for number in (1, 2, 3):
        model = root.find(f".//model[@name='dock-bay-wall-{number}']")
        for kind in ('collision', 'visual'):
            size = list(map(float, model.findtext(f'link/{kind}/geometry/box/size').split()))
            assert size == [.5, .05, .5]
