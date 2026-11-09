"""Unsupported legacy shape selection must fail before a ROS graph is created."""
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from rclpy.node import Node
from dock_detector.dock_detector_node import DockDetector


def test_rectangle_mode_is_rejected():
    def parameter(name, default):
        return SimpleNamespace(value='rectangle' if name=='target_shape' else default)
    with patch.object(Node,'__init__',return_value=None), \
         patch.object(DockDetector,'declare_parameter',side_effect=parameter):
        with pytest.raises(ValueError,match='target_shape must be v or trapezoid'):
            DockDetector()
