#!/usr/bin/env python3
"""
Preview an experimental docking approach and check the detected target pose.

The preview uses the detector's fresh dock poses and the controller's robot
transform. With the workspace sourced and the detector running, use:

     ros2 run dock_detector dock_controller --detected-goal

A successful preview logs preview_complete and exits without publishing motion.
Add --use-sim-time when the ROS graph uses /clock. The --execute option enables
motion for short approach tests. For routine Seggy docking, use Nav2's
/dock_robot for staging and named dock selection.
"""

from dock_detector.docking_controller import main

if __name__ == '__main__':
    raise SystemExit(main())
