"""World panels, detector profiles, markings and parking clearances agree."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from dock_detector.simulation import panel_polygons,polygons_intersect,polygon_clearance

PACKAGE=Path(__file__).resolve().parents[1]
WORLD=ET.parse(PACKAGE.parent/'articubot_one/assets/worlds/test_robot_world.sdf')

def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])

def model_polygon(name):
    model=WORLD.find(f".//model[@name='{name}']")
    pose=list(map(float,model.findtext('pose').split()))
    size=list(map(float,model.findtext('link/collision/geometry/box/size').split()))
    polygon=np.array([[-size[0]/2,-size[1]/2],[size[0]/2,-size[1]/2],
                      [size[0]/2,size[1]/2],[-size[0]/2,size[1]/2]])
    return polygon@rotation(pose[5]).T+pose[:2]


def test_perpendicular_v_world_matches_70cm_55degree_template_and_offset():
    cfg=yaml.safe_load((PACKAGE/'config/seggy_v.yaml').read_text())['dock_detector']['ros__parameters']
    assert cfg['target_shape']=='v'
    assert cfg['expected_angle']==math.radians(110)
    final=np.array([cfg['expected_dock_x'],cfg['expected_dock_y']])
    yaw=cfg['expected_dock_yaw']-math.pi
    assert yaw==-math.pi/2
    apex=final-cfg['v_offset']*np.array([math.cos(yaw),math.sin(yaw)])
    assert np.allclose(apex,[0.,6.6])
    for i,poly in enumerate(panel_polygons(math.radians(55),length=.7,shape='v'),1):
        assert np.allclose(model_polygon(f'v-dock-wall-{i}'),poly@rotation(yaw).T+apex,atol=1e-10)
        model=WORLD.find(f".//model[@name='v-dock-wall-{i}']")
        assert model.findtext('link/visual/geometry/box/size')=='0.70 0.05 0.50'
    markings=WORLD.find(".//model[@name='seggy_v_docking_area']")
    pose=list(map(float,markings.findtext('pose').split()))
    assert np.allclose(pose[:2],final) and abs(pose[5]-yaw)<1e-10
    assert markings.find("link/visual[@name='rear_boundary']") is None
    assert markings.find("link/visual[@name='apex_reference']") is not None


def test_two_docks_are_separate_and_both_centerline_approaches_clear():
    trap=[model_polygon(f'dock-bay-wall-{i}') for i in (1,2,3)]
    v=[model_polygon(f'v-dock-wall-{i}') for i in (1,2)]
    assert not any(polygons_intersect(a,b) for a in trap for b in v)
    nav=yaml.safe_load((PACKAGE.parent/'articubot_one/robots/seggy/config/nav2_params.yaml').read_text())
    body=np.array(__import__('json').loads(nav['local_costmap']['local_costmap']['ros__parameters']['footprint']))
    for profile,marking in [('seggy.yaml','seggy_docking_area'),('seggy_v.yaml','seggy_v_docking_area')]:
        cfg=yaml.safe_load((PACKAGE/'config'/profile).read_text())['dock_detector']['ros__parameters']
        final=np.array([cfg['expected_dock_x'],cfg['expected_dock_y']]);yaw=cfg['expected_dock_yaw']-math.pi
        mark_pose=list(map(float,WORLD.find(f".//model[@name='{marking}']/pose").text.split()))
        assert np.allclose(mark_pose[:2],final) and abs(mark_pose[5]-yaw)<1e-10
        for d in np.linspace(0,.8,17):
            footprint=body@rotation(yaw).T+final+d*np.array([math.cos(yaw),math.sin(yaw)])
            assert not any(polygons_intersect(footprint,p) for p in trap+v)
        if cfg['target_shape']=='v':
            footprint=body@rotation(yaw).T+final
            assert min(polygon_clearance(footprint,p) for p in v)>.16
