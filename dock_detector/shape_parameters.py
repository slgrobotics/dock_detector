"""Human-facing ROS parameter schema and conversion to geometric units."""
import math

SHAPE_DEFAULTS = {
    'trapezoid.panel_width': .5, 'trapezoid.flare_angle_deg': 32.5,
    'trapezoid.parking_offset': .425, 'trapezoid.max_line_gap': .30,
    'trapezoid.dimension_tolerance': .10, 'trapezoid.max_corner_gap': .18,
    'trapezoid.joint_refinement': True,
    'v.joint_refinement': True, 'v.panel_width': .70, 'v.flare_angle_deg': 55.,
    'v.parking_offset': .60, 'v.max_apex_gap': .25,
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
    for shape in ('trapezoid', 'v'):
        if not 0 < c[f'{shape}.flare_angle_deg'] < 90:
            raise ValueError(f'{shape}.flare_angle_deg must be between 0 and 90 degrees')
    c.update(trapezoid_width=c['trapezoid.panel_width'],
             trapezoid_depth=c['trapezoid.panel_width']*math.cos(math.radians(c['trapezoid.flare_angle_deg'])),
             trapezoid_flare_angle=math.radians(c['trapezoid.flare_angle_deg']),
             expected_angle=2*math.radians(c['v.flare_angle_deg']))
    for old, new in {'v_offset':'v.parking_offset', 'trapezoid_offset':'trapezoid.parking_offset',
                     'trapezoid_max_line_gap':'trapezoid.max_line_gap',
                     'dimension_tolerance':'trapezoid.dimension_tolerance',
                     'max_corner_gap':'trapezoid.max_corner_gap',
                     'joint_refinement':'trapezoid.joint_refinement',
                     'max_apex_gap':'v.max_apex_gap'}.items():
        c[old] = c[new]
    for key in ('expected_dock_yaw', 'angle_tolerance', 'yaw_tolerance'):
        c[key] = math.radians(c[key+'_deg'])
    return c
