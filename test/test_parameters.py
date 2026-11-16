"""
Verify the detector’s public ROS YAML geometry parameters.

The tests cover equal panel dimensions, degree conversion, invalid geometry,
and the ROS parser’s flattening of shape-specific parameter groups.

From the workspace root, source install/setup.bash in a built ROS workspace
and run:

     PYTHONPATH=src/dock_detector:$PYTHONPATH python3 -m pytest -q src/dock_detector/test/test_parameters.py

Expect pytest to pass when valid profiles retain their geometry and invalid
values raise the expected errors.
"""

import math
from pathlib import Path
import pytest
from rclpy.parameter import parameter_dict_from_yaml_file
from dock_detector.shape_parameters import SHAPE_DEFAULTS, geometry_parameters


def defaults():
    return dict(SHAPE_DEFAULTS, expected_dock_yaw_deg=180., angle_tolerance_deg=12., yaw_tolerance_deg=25.)


def test_equal_panel_dimensions_and_degree_conversion():
    c=geometry_parameters(defaults())
    assert c['trapezoid_width']==.5
    assert c['trapezoid_depth']==pytest.approx(.421695722906443)
    assert c['expected_angle']==pytest.approx(math.radians(110))
    assert c['expected_dock_yaw']==math.pi


@pytest.mark.parametrize('key,value', [('v_shaped_bay.panel_width',0.),('trapezoid_bay.panel_width',float('nan')),
    ('v_shaped_bay.flare_angle_deg',90.),('trapezoid_bay.flare_angle_deg',-1.),('v_shaped_bay.parking_offset',-.1)])
def test_invalid_geometry(key,value):
    c=defaults();c[key]=value
    with pytest.raises(ValueError):geometry_parameters(c)


@pytest.mark.parametrize('profile',['seggy_t.yaml','seggy_v.yaml'])
def test_ros_yaml_parser_flattens_shape_subtrees(profile):
    params=parameter_dict_from_yaml_file(str(Path(__file__).parents[1]/'config'/profile))
    assert params['v_shaped_bay.panel_width'].value.double_value==.7
    assert params['trapezoid_bay.flare_angle_deg'].value.double_value==32.5
