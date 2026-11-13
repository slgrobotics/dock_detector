"""Public YAML schema preserves measured bays and rejects invalid geometry."""
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
