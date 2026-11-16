"""
Verify that the detector rejects the unsupported rectangle mode.

The test supplies the legacy rectangle setting and checks that initialization
fails before creating the ROS graph.

From the workspace root, source install/setup.bash in a built ROS workspace
and run:

     PYTHONPATH=src/dock_detector:$PYTHONPATH python3 -m pytest -q src/dock_detector/test/test_node_modes.py

Expect pytest to pass when the detector raises the expected error.
"""

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
        with pytest.raises(ValueError,match='target_shape must be v_shaped_bay or trapezoid_bay'):
            DockDetector()
