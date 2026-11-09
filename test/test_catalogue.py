from dock_detector.catalogue import Selection


def test_selection_unknown_and_conditional_release():
    s=Selection({'a':{},'b':{}})
    assert s.select(True,'a','first')[0]
    assert not s.select(True,'missing','second')[0]
    assert (s.dock_id,s.acquisition_id)==('a','first')
    assert s.select(True,'b','second')[0]
    assert not s.select(False,'a','first')[0]
    assert s.dock_id=='b'
    assert s.select(False,'b','second')[0]
    assert s.dock_id==s.acquisition_id==''
    assert not s.select(True,'a','')[0]


def test_catalogue_generates_nav2_poses_without_mutating_original():
    import math
    from pathlib import Path
    import yaml
    from dock_detector.nav2_config_merger import merge_nav2
    c=yaml.safe_load((Path(__file__).parents[1]/'config/seggy_docks.yaml').read_text())
    base={'docking_server':{'ros__parameters':{'dock_backwards':False}}}
    params=merge_nav2(base,c)['docking_server']['ros__parameters']
    assert 'dock_backwards' not in params
    assert base['docking_server']['ros__parameters']['dock_backwards'] is False
    assert params['trapezoid_bay']['pose']==[-1.,3.,math.pi]
    assert params['v_bay']['pose']==[0.,6.,math.pi/2]
    assert params['trapezoid_bay']['id']=='trapezoid_bay'
    assert params['v_bay']['id']=='v_bay'
    assert params['lidar_dock']['plugin']=='dock_detector_nav2::LidarDock'
    assert params['controller']['use_collision_detection']
