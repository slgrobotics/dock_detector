#!/usr/bin/env python3
"""
ROS-only experimental docking controller. Preview by default; --execute moves.

dock_controller.py is a thin entry point for the same controller that runs with ros2 run dock_detector dock_controller.

It’s useful for previewing detected poses and testing short approaches from a nearby position.

For routine Seggy docking, use Nav2’s /dock_robot, which handles staging and named dock selection.
"""

from dock_detector.docking_controller import main

if __name__ == '__main__':
    raise SystemExit(main())
