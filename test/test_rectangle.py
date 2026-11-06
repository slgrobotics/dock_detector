import math
import numpy as np
from dock_detector.geometry import detect_rectangle, angle_error


def rectangle(center=(0.,0.), yaw=0., width=1.05, depth=.4, noise=.006):
    rng=np.random.default_rng(12)
    segments=[np.column_stack([np.zeros(80),np.linspace(-width/2,width/2,80)]),
              np.column_stack([np.linspace(.01,depth,30),np.full(30,-width/2)]),
              np.column_stack([np.linspace(.01,depth,30),np.full(30,width/2)])]
    rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    return [s@rot.T+center+rng.normal(0,noise,s.shape) for s in segments]


def test_rotated_noisy_rectangle_with_clutter():
    segments=rectangle((2,-1),.7)
    points=np.vstack(segments+[np.random.default_rng(7).uniform([-1,-3],[4,3],(30,2))])
    result=detect_rectangle(points,(2.04,-1.03),.72)
    assert result is not None
    assert np.linalg.norm(result.apex-[2,-1]) < .025
    assert abs(angle_error(result.yaw,.7)) < .03
    assert len(result.arms)==3


def test_reject_missing_faces_and_room_corner():
    segments=rectangle()
    for points in [np.empty((0,2)),segments[0],np.vstack(segments[:2]),np.vstack(segments[1:])]:
        assert detect_rectangle(points,(0,0),0) is None


def test_reject_wrong_width_depth_and_association():
    assert detect_rectangle(np.vstack(rectangle(width=1.4)),(0,0),0) is None
    assert detect_rectangle(np.vstack(rectangle(depth=.8)),(0,0),0) is None
    assert detect_rectangle(np.vstack(rectangle(center=(2,2))),(0,0),0) is None
    assert detect_rectangle(np.vstack(rectangle(yaw=math.pi)),(0,0),0) is None


def test_rear_blind_gap_and_shuffled_points():
    segments=rectangle()
    # Five-degree blind sector removes a short central rear-wall interval.
    segments[0]=segments[0][abs(segments[0][:,1])>.035]
    points=np.vstack(segments);np.random.default_rng(3).shuffle(points)
    assert detect_rectangle(points,(0,0),0) is not None
