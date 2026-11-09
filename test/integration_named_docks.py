"""Explicit no-motion ROS test: pytest test/integration_named_docks.py in isolated domain."""
import time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from dock_detector_interfaces.srv import SelectDock
from dock_detector_interfaces.msg import DockDetection
from dock_detector.dock_detector_node import DockDetector
from dock_detector.core.geometry import Detection


def test_named_detector_selection_and_identified_measurement():
    config=Path(__file__).parents[1]/'config/seggy_docks.yaml'
    rclpy.init(args=['--ros-args','--params-file',str(config),'-p','use_sim_time:=false'])
    detector=DockDetector();peer=Node('selection_test')
    executor=SingleThreadedExecutor();executor.add_node(detector);executor.add_node(peer)
    client=peer.create_client(SelectDock,'dock_detector/select_dock')
    observations=[]
    sub=peer.create_subscription(DockDetection,'dock_detector/detection',observations.append,10)
    def spin(duration=.1):
        end=time.monotonic()+duration
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    def select(dock,token,activate=True):
        f=client.call_async(SelectDock.Request(activate=activate,dock_id=dock,acquisition_id=token))
        executor.spin_until_future_complete(f,timeout_sec=2.)
        assert f.done();return f.result()
    try:
        assert client.wait_for_service(timeout_sec=2.)
        assert not detector.selection.dock_id
        assert select('trapezoid_bay','one').success
        assert detector.cfg['target_shape']=='trapezoid'
        detector.pending=LaserScan()
        assert select('v_bay','two').success
        assert detector.pending is None and detector.last_detection is None
        assert detector.cfg['target_shape']=='v'
        assert not select('trapezoid_bay','one',False).success
        assert not select('unknown','bad').success
        assert detector.selection.acquisition_id=='two'
        tf=TransformStamped();tf.transform.rotation.w=1.
        scan=LaserScan();scan.header.frame_id='laser';scan.range_min=.3;scan.range_max=24.
        scan.angle_increment=.1;scan.ranges=[1.,1.2,1.4]
        scan.header.stamp=detector.get_clock().now().to_msg()
        with patch.object(detector.buffer,'lookup_transform',return_value=tf), \
             patch('dock_detector.dock_detector_node.detect_v',return_value=Detection(np.array([0.,6.6]),-np.pi/2,(),.001)):
            detector.on_scan(scan)
        spin()
        assert len(observations)==1
        assert observations[0].dock_id=='v_bay' and observations[0].acquisition_id=='two'
        assert observations[0].pose.header.stamp==scan.header.stamp
        assert abs(observations[0].pose.pose.position.y-6.)<1e-8
        assert select('v_bay','three').success
        # Queued old scan cannot acquire the new session.
        detector.on_scan(scan);spin()
        assert len(observations)==1
        assert select('v_bay','three',False).success
        detector.on_scan(scan);spin();assert len(observations)==1
    finally:
        executor.remove_node(detector);executor.remove_node(peer)
        detector.destroy_node();peer.destroy_node();executor.shutdown();rclpy.shutdown()


def test_legacy_single_profile_remains_available():
    config=Path(__file__).parents[1]/'config/seggy_t.yaml'
    rclpy.init(args=['--ros-args','--params-file',str(config),'-p','use_sim_time:=false'])
    detector=DockDetector()
    try:
        assert not detector.catalogue_mode
        assert detector.cfg['target_shape']=='trapezoid'
        response=detector.select_dock(SelectDock.Request(activate=True,dock_id='any',acquisition_id='test'),SelectDock.Response())
        assert not response.success
    finally:
        detector.destroy_node();rclpy.shutdown()
