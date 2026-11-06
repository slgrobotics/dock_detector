Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------

# LiDAR dock detector

Standalone, perception-only ROS 2 node for detecting a relative position a known three-sided rectangular bay
(or a V landmark via `target_shape: v`) using only LiDAR `scan` topic messages.

Assists Nav2 Docking by publishing a `Dock` message that contains the dock pose.

> In the ROS 2 Nav2 Docking Framework, the dock detector package is responsible for detecting the target charging dock or station
> (using sensors like LiDAR, cameras, or AR tags) and publishing a `nav2_msgs/msg/Dock` message (or equivalent dock pose topic),
> which provides the framework with the precise position and orientation of the dock so the robot can perform its final alignment and charging maneuver.

<img alt="LiDAR dock detector - Seggy SIM" src="https://github.com/user-attachments/assets/0738a09a-0b89-4a54-bf5b-31ea4736a8ff" />


## Run

```bash
cd ~/robot_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select dock_detector
source install/setup.bash
ros2 launch dock_detector detector.launch.py
```

Override `params_file`, `use_sim_time`, or `namespace` as launch arguments. Parameters
are read at startup; restart after editing them. Defaults target Seggy's current
simulation and must be calibrated for other worlds/hardware. Do not run two
publishers on the docking pose topic.

## Interface

- Input: `scan` (`sensor_msgs/msg/LaserScan`, sensor-data QoS) and timestamped TF.
- Output: `detected_dock_pose` (`geometry_msgs/msg/PoseStamped`, volatile QoS).
- Status: `dock_detector/detected` (`std_msgs/msg/Bool`).
- Debug: `dock_detector/markers` (`visualization_msgs/msg/MarkerArray`).

In RViz add MarkerArray on the debug topic and Pose on the detected pose topic.
Green segments show fitted wall faces; the blue arrow shows the parking pose.

The output is the desired **base_link parking XY**, projected onto z=0 in
`fixed_frame` (default `odom`). Its +X axis points toward the bay opening, matching
Seggy's parked forward heading for reverse docking. It is not the rear-wall centre or
a camera/tag pose. Configure downstream plugin detection transforms as identity
if this convention is used; do not apply the parking offset twice. Verify the
installed backward-docking direction convention before enabling motion.

`expected_dock_{x,y,yaw}` in `expected_frame` is a coarse association prior, not a
substitute detection. `target_to_dock_{x,y}` offsets the fitted rear-face centre
in the target's axes to the desired robot parking pose. The current inner rear
face is at world x=-1.425, y=3.0; its opening points +X. An x offset of **0.425 m**
therefore places the desired robot XY at (-1, 3). The inner bay width is 1.05 m,
and side-wall depth from the visible rear face to the opening is 0.40 m.
`rectangle_width`, `rectangle_depth`, `dimension_tolerance`, and `max_corner_gap`
control shape acceptance. This template is measured from the current SDF, not
from a runtime simulator pose query. Changing the bay or walls independently requires recalibration.

## Algorithm and limitations

Filter valid ranges; transform points at the scan timestamp; select a circular
region around the expected target; extract contiguous RANSAC/TLS line segments;
require a supported rear wall plus two perpendicular side walls. Check width,
depth coverage, corner proximity, association position, and opening direction.
The rectangle landmark is the midpoint of the inner rear corners; its heading is
the rear-wall normal pointing out of the bay. A lone wall, missing face, V, or
ordinary L corner produces no new detection. Input ordering is irrelevant, including across scan endpoints.
The opening direction must agree with the observed side-wall extensions and prior.
There is no additional temporal smoothing; downstream plugin filtering can be used.
A geometrically identical nearby bay can still be ambiguous: this is a single
known-dock detector, not arbitrary dock recognition.

Seggy uses a 355-degree scan, leaving a 5-degree rear blind sector. Minimum range is 0.3 m. All three faces must retain enough returns;
missing a face produces no new pose. No stale pose is republished. A steady-clock
watchdog clears detection status after `detection_timeout`, even when simulation
pauses; scan age is also checked against ROS time. Markers expire and are cleared
on detection loss. Scan timestamps and availability of exact-time TF are required.
The node briefly retries the newest scan when TF arrives late. Motion distortion
within a scan is not compensated; begin stationary, then validate at slow speed.

## Verify

```bash
PYTHONPATH=src/dock_detector python3 -m pytest -q src/dock_detector/test
ros2 topic echo /detected_dock_pose
ros2 topic echo /dock_detector/detected
ros2 topic hz /detected_dock_pose
```

Check measured pose against Gazebo ground truth at several positions/headings,
including the rear blind sector, partial occlusion, noise, false corners, and target
loss. No docking precision is established by unit tests alone. Full experiment
history is maintained in `~/articubot_one.wiki/Docking-in-ROS2.md`.

## Recorded stationary check (2026-11-05)

With the new rectangular bay and the restarted 5-degree-gap LiDAR, a 20-second
perception-only run received 118 scans and published 113 detected poses (includes
TF startup). Map-frame mean parking pose was approximately
(-0.955253, 3.013661, yaw=0.000750). Standard deviations were
(0.001254 m, 0.003378 m, 0.003843 rad). These are stationary repeatability figures,
not global accuracy; map/world alignment is not calibrated. No docking action or
robot motion was commanded during this check. The detector exits after the test;
use the launch command above to run it continuously.

--------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
