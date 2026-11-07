"""Check shape-specific scalar offsets in both prior inversion and output pose."""
import math,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest,yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.node import Node
from dock_detector.geometry import Detection
from dock_detector.node import DockDetector


@pytest.mark.parametrize('shape,distance', [('v',.6),('trapezoid',.425)])
def test_scalar_offset_applies_in_landmark_direction_for_both_shapes(shape,distance):
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/seggy.yaml').read_text())['dock_detector']['ros__parameters']
    cfg.update(target_shape=shape,expected_dock_x=1.,expected_dock_y=2.,expected_dock_yaw=0.)
    prior_tf=TransformStamped()
    prior_tf.transform.translation.x=10.;prior_tf.transform.translation.y=20.
    prior_tf.transform.rotation.z=math.sin(math.pi/4)
    prior_tf.transform.rotation.w=math.cos(math.pi/4)
    laser_tf=TransformStamped();laser_tf.transform.rotation.w=1.
    scan=LaserScan();scan.header.frame_id='laser_frame';scan.header.stamp.sec=10
    scan.ranges=[1.,1.2,1.4];scan.range_min=.3;scan.range_max=24.;scan.angle_increment=.1
    poses=[];statuses=[];markers=[]
    now=time.monotonic()
    node=SimpleNamespace(cfg=cfg,pending=scan,last_detection=None,last_scan_wall=now,
        received_at=now,last_processed_stamp=None,processed_count=0,match_count=0,
        buffer=SimpleNamespace(lookup_transform=lambda target,source,stamp:
            prior_tf if source==cfg['expected_frame'] else laser_tf),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.1,clock_type=ClockType.ROS_TIME)),
        invalidate=lambda:None,report=lambda *a,**k:None,
        pub=SimpleNamespace(publish=poses.append),status=SimpleNamespace(publish=statuses.append),
        markers=SimpleNamespace(publish=markers.append))
    detected=Detection(np.array([4.,5.]),.7,(),.001)
    matcher='dock_detector.node.detect_v' if shape=='v' else 'dock_detector.node.detect_trapezoid'
    with patch(matcher,return_value=detected) as match:
        DockDetector.process(node)
    # Map (1,2) rotated +90 degrees then translated (10,20) = odom (8,21).
    # Inverse offset must follow the expected landmark heading, not world X.
    assert match.call_args.args[1] == pytest.approx([8.,21.-distance])
    assert match.call_args.args[2] == pytest.approx(math.pi/2)
    assert len(poses)==1 and statuses[-1].data
    pose=poses[0]
    assert pose.pose.position.x == pytest.approx(4.+distance*math.cos(.7))
    assert pose.pose.position.y == pytest.approx(5.+distance*math.sin(.7))
    assert 2*math.atan2(pose.pose.orientation.z,pose.pose.orientation.w)==pytest.approx(.7)
    assert pose.header.stamp.sec==10 and pose.header.frame_id==cfg['fixed_frame']


@pytest.mark.parametrize('name', ['v_offset','trapezoid_offset'])
@pytest.mark.parametrize('value', [-.1,float('nan'),float('inf')])
def test_invalid_scalar_offsets_fail_before_ros_graph_creation(name,value):
    def parameter(key,default):return SimpleNamespace(value=value if key==name else default)
    with patch.object(Node,'__init__',return_value=None), \
         patch.object(DockDetector,'declare_parameter',side_effect=parameter):
        with pytest.raises(ValueError,match=f'{name} must be finite and nonnegative'):
            DockDetector()
