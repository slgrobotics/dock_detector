"""Human-facing ROS parameter schema and conversion to geometric units."""
import math

SHAPE_DEFAULTS = {
    'trapezoid_bay.panel_width': .5, 'trapezoid_bay.flare_angle_deg': 32.5,
    'trapezoid_bay.parking_offset': .425, 'trapezoid_bay.max_line_gap': .30,
    'trapezoid_bay.dimension_tolerance': .10, 'trapezoid_bay.max_corner_gap': .18,
    'trapezoid_bay.joint_refinement': True,
    'v_shaped_bay.joint_refinement': True, 'v_shaped_bay.panel_width': .70, 'v_shaped_bay.flare_angle_deg': 55.,
    'v_shaped_bay.parking_offset': .60, 'v_shaped_bay.max_apex_gap': .25,
}

def geometry_parameters(values):
    """Return internal parameters; public angles are degrees, lengths metres."""
    c = dict(values)
    for k in SHAPE_DEFAULTS:
        if k.endswith('joint_refinement'):
            continue
        v = c[k]
        if not math.isfinite(v) or v < 0 or (v == 0 and not k.endswith('parking_offset')):
            raise ValueError(f'{k} must be finite and ' + ('nonnegative' if k.endswith('parking_offset') else 'positive'))
    for shape in ('trapezoid_bay', 'v_shaped_bay'):
        if not 0 < c[f'{shape}.flare_angle_deg'] < 90:
            raise ValueError(f'{shape}.flare_angle_deg must be between 0 and 90 degrees')
    c.update(trapezoid_width=c['trapezoid_bay.panel_width'],
             trapezoid_depth=c['trapezoid_bay.panel_width']*math.cos(math.radians(c['trapezoid_bay.flare_angle_deg'])),
             trapezoid_flare_angle=math.radians(c['trapezoid_bay.flare_angle_deg']),
             expected_angle=2*math.radians(c['v_shaped_bay.flare_angle_deg']))
    for old, new in {'v_offset':'v_shaped_bay.parking_offset', 'trapezoid_offset':'trapezoid_bay.parking_offset',
                     'trapezoid_max_line_gap':'trapezoid_bay.max_line_gap',
                     'dimension_tolerance':'trapezoid_bay.dimension_tolerance',
                     'max_corner_gap':'trapezoid_bay.max_corner_gap',
                     'joint_refinement':'trapezoid_bay.joint_refinement',
                     'max_apex_gap':'v_shaped_bay.max_apex_gap'}.items():
        c[old] = c[new]
    for key in ('expected_dock_yaw', 'angle_tolerance', 'yaw_tolerance'):
        c[key] = math.radians(c[key+'_deg'])
    return c
