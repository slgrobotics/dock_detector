import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from dock_detector.core.simulation import panel_polygons, polygons_intersect, simulate_scan


def test_offline_boxes_match_current_sdf():
    root=ET.parse(Path(__file__).resolve().parents[2]/'articubot_one/assets/worlds/test_robot_world.sdf')
    for polygon,number in zip(panel_polygons(math.radians(32.5)),(2,1,3)):
        model=root.find(f".//model[@name='dock-bay-wall-{number}']")
        pose=list(map(float,model.findtext('pose').split()))
        size=list(map(float,model.findtext('link/collision/geometry/box/size').split()))
        angle=pose[5];d=np.array([math.cos(angle),math.sin(angle)])*size[0]/2
        n=np.array([-math.sin(angle),math.cos(angle)])*size[1]/2
        center=np.array(pose[:2])-[-1.425,3.]
        actual=np.array([center-d-n,center+d-n,center+d+n,center-d+n])
        assert np.allclose(polygon,actual,atol=1e-10)


def test_minimum_range_does_not_reveal_wall_behind():
    pts,support=simulate_scan(.1,0.,0.,math.pi/6,np.random.default_rng(1),noise=0.)
    assert support[0][0]==0  # All rear-face first hits are inside minimum range.
    assert np.isfinite(pts).all()


def test_convex_clearance_detects_contact_and_separation():
    square=np.array([[0,0],[1,0],[1,1],[0,1]],dtype=float)
    assert polygons_intersect(square,square+[.5,.5])
    assert polygons_intersect(square,square+[1.,0.])
    assert not polygons_intersect(square,square+[1.01,0.])


def test_v_scan_and_match_across_requested_angles():
    from dock_detector.core.geometry import detect_v
    for degree in (20,25,30,35,40,45):
        flare=math.radians(degree)
        points,support=simulate_scan(1.225,0.,0.,flare,np.random.default_rng(20261106),shape='v')
        assert len(support)==2
        assert all(n>=5 and span>=.12 for n,span in support)
        result=detect_v(points,(0.,0.),0.,expected_angle=2*flare,max_length=1.3)
        assert result is not None,degree
        assert np.linalg.norm(result.apex)<.03


def test_v_is_narrower_and_changes_parking_clearance():
    from dock_detector.core.simulation import footprint_free
    body=np.array([[.28,0.],[.235,-.235],[0.,-.28],[-.235,-.235],
                   [-.3,-.14],[-.3,.14],[-.235,.235],[0.,.28],[.235,.235]])
    assert footprint_free(.425,0.,0.,math.radians(32.5),body)
    assert not footprint_free(.425,0.,0.,math.radians(32.5),body,shape='v')
    assert footprint_free(.8,0.,0.,math.radians(32.5),body,shape='v')


def test_seventy_cm_v_panels_and_sensor_support():
    from dock_detector.core.geometry import detect_v
    length=.7
    for degree in (40,45,50,55,60):
        flare=math.radians(degree)
        panels=panel_polygons(flare,length=length,shape='v')
        assert len(panels)==2
        for panel in panels:
            assert abs(np.linalg.norm(panel[1]-panel[0])-length)<1e-12
        points,support=simulate_scan(1.225,0.,0.,flare,np.random.default_rng(5),shape='v',length=length)
        assert len(support)==2
        assert all(n>=5 and span>.12 for n,span in support)
        assert all(span<=length+1e-12 for n,span in support)
        assert detect_v(points,(0.,0.),0.,expected_angle=flare*2,max_length=1.3) is not None


def test_euclidean_polygon_clearance():
    from dock_detector.core.simulation import polygon_clearance
    square=np.array([[0,0],[1,0],[1,1],[0,1]],dtype=float)
    assert polygon_clearance(square,square+[.5,0])==0
    assert abs(polygon_clearance(square,square+[2.,0])-1.)<1e-12
    assert abs(polygon_clearance(square,square+[2.,2.])-math.sqrt(2))<1e-12
