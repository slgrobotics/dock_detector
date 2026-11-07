Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# LiDAR dock detector

**Contents:**

- [What is this package for?](#what-is-this-package-for)
- [Technical overview](#technical-overview)
- [Run](#run)
- [Human-friendly geometry configuration](#human-friendly-geometry-configuration)
- [V joint refinement](#v-joint-refinement)
- [Multiple docks and robots](#multiple-docks-and-robots)
- [Interface](#interface)
- [Algorithm and limitations](#algorithm-and-limitations)
- [V landmark mode](#v-landmark-mode)
- [Two docks in the test world](#two-docks-in-the-test-world)
- [Fixed V placement recommendation](#fixed-v-placement-recommendation)
- [Verify](#verify)
- [Recorded stationary check (2026-11-05)](#recorded-stationary-check-2026-11-05)
- [Offline flare-angle sweep](#offline-flare-angle-sweep)
- [Joint fitting and confidence diagnostics](#joint-fitting-and-confidence-diagnostics)
- [Reusable live-test controller](#reusable-live-test-controller)

-------------

## What is this package for?

A robot needs to know exactly where a docking bay is before it can park there.
This package helps it find the bay using LiDAR, a sensor that measures distances
with laser light. It looks for the bay's wall shapes and reports where the robot
should park and which way the dock points.

For example, Seggy can look behind itself, find a V-shaped bay, and send its
measured position to the software that steers the robot. If there are several
named bays, the detector looks for the one it has been asked to find. The detector
itself does not drive the robot.

## Technical overview

Standalone, perception-only ROS 2 node for detecting a known trapezoidal bay
or a two-arm V landmark using LiDAR `scan` messages. Supported `target_shape`
values are `trapezoid` and `v`. The rectangular bay option has been removed.

Assists Nav2 Docking by publishing a `geometry_msgs/msg/PoseStamped` parking pose.

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

## Human-friendly geometry configuration

Lengths are metres and all public angles are degrees. ROS 2 flattens nested YAML
keys into dotted parameter names, for example `trapezoid.panel_width`.
All panels within a shape have equal width (measured along the panel).

```yaml
trapezoid:
  panel_width: 0.50
  flare_angle_deg: 32.5
  parking_offset: 0.425
  max_line_gap: 0.30
  dimension_tolerance: 0.05
  max_corner_gap: 0.10
  joint_refinement: true
v:
  joint_refinement: true
  panel_width: 0.70
  flare_angle_deg: 55.0
  parking_offset: 0.60
  max_apex_gap: 0.25
```

Flare is each side panel's angle from the outward opening centreline. V included
opening is twice the flare. Trapezoid rear width equals panel width; its depth is
computed as `panel_width * cos(flare_angle_deg)` (converting degrees internally).
V detection permits partial panel observations, but rejects observed support
extending beyond the configured panel width plus two line-distance tolerances.
It does not require both outer endpoints to be visible.

Common settings remain outside the shape subtrees. Use `expected_dock_yaw_deg`
(inward Nav2 heading), `angle_tolerance_deg`, and `yaw_tolerance_deg`.
The supplied profiles retain their measured geometry and tolerances.

This schema replaces the old flat geometry keys: `trapezoid_width`,
`trapezoid_depth`, `trapezoid_flare_angle`, `expected_angle`, `v_offset`,
`trapezoid_offset`, and shape-specific fit controls. Flat radian yaw/tolerance
parameters are also replaced by their `_deg` counterparts. Migrate external
parameter files before restarting; old overrides are not compatibility aliases.
Historical result snapshots and examples below retain their original schema;
use the current files in `config/` for runnable parameters.

## V joint refinement

`v.joint_refinement: true` enables bounded joint fitting after the existing V
line-pair seed. Both finite, equal-width panels are fit simultaneously with their
configured opening angle fixed; only landmark X/Y/yaw change. The shared robust
solver uses fixed point assignments, equal panel weighting and Huber residuals.
Corrections are bounded to 5 cm / 0.10 rad from the seed. Fits must converge,
remain within the association prior, be well conditioned, and retain adequate
support on both panels. Rejected refinement produces no pose, rather than silently
falling back to the seed. It cannot recover a V with only one supported panel.

Diagnostics include refinement state, per-panel support, residual, correction,
conditioning and confidence, using the same schema as trapezoid fitting.
`v.joint_refinement: false` retains the independent-line baseline. This switch is
separate from `trapezoid.joint_refinement`. Geometric Python API callers must pass
`joint_refinement=True` and `panel_width` explicitly to opt in; node defaults enable it.

A paired 135-scan offline simulation comparison at 55 degrees per arm and 0.70 m
panel width found identical coverage (132/135) and improved p95 errors:
position 11.1 -> 6.5 mm; heading 1.08 -> 0.49 degrees. P95 total detection runtime
was approximately 30 -> 36 ms on this machine. These synthetic results do not
establish hardware accuracy or closed-loop docking performance. See
[comparison and reproduction script](results/2026-11-07-v-refinement/README.md).

## Multiple docks and robots

Named-dock selection is implemented through `dock_detector_interfaces` and the
thin `dock_detector_nav2::LidarDock` plugin. Use `config/seggy_docks.yaml` for a
catalogue of per-dock priors, shapes and geometry. A detector with nonempty
`dock_ids` starts idle until its `~/select_dock` service selects a target. It then
publishes `~/detection` (DockDetection) with dock ID and acquisition token, alongside
the legacy PoseStamped/debug topics. Selection clears prior acquisition state;
unknown IDs are rejected and releases require a matching token.

[Build, configuration generation, Nav2 actions and testing](../dock_detector_nav2/README.md).
The standard DockRobot dock_id selects the detector automatically through the
plugin; no custom BT node or coordinator is needed. One catalogue generates Nav2's
database, avoiding duplicate dock poses. Profiles without dock_ids retain continuous
single-dock mode; identified Nav2 integration requires catalogue mode.

Each robot needs its own namespace, isolated scan/pose/velocity/TF topics and
explicit frame configuration. Shared bay reservations remain outside this package.
Geometrically identical nearby bays may still be ambiguous within the association
prior; the reported dock ID does not constitute unique physical identification.
Multi-robot operation and full Nav2 action docking are not yet validated.

The trapezoid profile is `config/seggy_t.yaml`; `config/seggy_v.yaml` remains the V
profile. Historical result snapshots retain original filenames.

## Interface

- Input: `scan` (`sensor_msgs/msg/LaserScan`, sensor-data QoS) and timestamped TF.
- Output: `detected_dock_pose` (`geometry_msgs/msg/PoseStamped`, volatile QoS).
- Status: `dock_detector/detected` (`std_msgs/msg/Bool`).
- Debug: `dock_detector/markers` (`visualization_msgs/msg/MarkerArray`).

In RViz add MarkerArray on the debug topic and Pose on the detected pose topic.
Green segments show fitted wall faces; the blue arrow shows the parking pose.

The output is the desired **base_link parking XY**, projected onto z=0 in
`fixed_frame` (default `odom`). Its +X axis points **into the bay**, following
Nav2's dock convention. For backward docking the parked robot faces the opposite
direction. Configure Nav2's plugin with `dock_direction: backward`,
`rotate_to_dock: false`, and identity external detection transforms (zero
translation and zero roll/pitch/yaw). Remove the deprecated server-level
`dock_backwards` override. Do not apply the parking offset twice.
The blue RViz arrow now points into the bay, not along the parked robot heading.

**Convention change (2026-11-07):** published yaw and `expected_dock_yaw` are now
inward. Existing external profiles must add pi to their old outward prior yaw;
XY and scalar offsets stay unchanged. Seggy's supplied profiles are updated:
trapezoid dock yaw pi; V dock yaw pi/2. Internal geometric fits still use the
outward opening axis. Historical results retain their original convention.
The experimental controller converts detected dock yaw to desired robot yaw
(adds pi for reverse, unchanged for `--forward`); explicit `--goal` continues to
mean robot pose. Restart detector and controller together after rebuilding.

`expected_dock_{x,y,yaw}` in `expected_frame` uses inward dock yaw and is a coarse association prior, not a
substitute detection. `v_offset` and `trapezoid_offset` are separate scalar distances in metres.
Each places its landmark behind the final point along the staging-to-final
(reverse-docking) direction. The node selects the parameter for `target_shape`.
`v_offset` measures final point to V apex; `trapezoid_offset` measures final point
to inner rear-face midpoint. The output parking point is that distance ahead of
the landmark along its opening axis. Published heading is opposite the geometric opening axis; there are no lateral
or yaw offset parameters. The current inner rear
face is at world x=-1.425, y=3.0; its opening points +X. An x offset of **0.425 m**
therefore places the desired robot XY at (-1, 3). `trapezoid_width`,
`trapezoid_depth`, `dimension_tolerance`, and `max_corner_gap` control shape acceptance. This template is measured from the current SDF, not
from a runtime simulator pose query. Changing the bay or walls independently requires recalibration.

Current defaults:

```yaml
v_offset: 0.60
trapezoid_offset: 0.425
```

For final world (-1,3,yaw=0), these imply V apex (-1.6,3) and trapezoid rear-face
centre (-1.425,3). The same selected scalar is applied inversely to the coarse
parking prior and forward to the measured landmark; switching shapes selects its
own offset automatically. Values must be finite and nonnegative. The old
`target_to_dock_x` and `target_to_dock_y` parameters have been removed; update
external parameter files and restart the detector. Changing offsets does not
move physical/Gazebo walls, so the geometry and parameters must agree.

## Algorithm and limitations

Filter valid ranges; transform points at the scan timestamp; select a circular
region around the expected target; extract RANSAC/TLS line segments with bounded gaps.
In trapezoid mode require a supported rear wall plus both outward-flared side walls. Check width,
depth coverage, corner proximity, association position, and opening direction.
The bay landmark is the midpoint of the inner rear corners; its heading is
the rear-wall normal pointing out of the bay. For trapezoid mode, a lone wall, missing face, V, or ordinary L corner produces
no new detection. Input ordering is irrelevant, including across scan endpoints.
The opening direction must agree with the observed side-wall extensions and prior.
There is no temporal smoothing; trapezoid mode uses joint spatial refinement as
described below. Downstream plugin filtering can also be used.
A geometrically identical nearby bay can still be ambiguous: this is a single
known-dock detector, not arbitrary dock recognition.

The current Seggy world uses `target_shape: trapezoid`: inner rear width **0.50 m**,
forward depth **0.4217 m**, and each side flared **32.5 degrees outward** from the
opening axis. Each side wall is 0.50 m long (as is the rear panel); opening width is
1.0373 m. Rear-face centre remains (-1.425, 3), so the parking offset
and final cross remain unchanged. The rear width equals Seggy’s nominal 0.50 m width; assess the full robot
footprint along the taper before docking motion. These dimensions alone do not
establish clearance. `trapezoid_width`, `trapezoid_depth`, and
`trapezoid_flare_angle` (radians) describe these inner faces. V mode remains available; no rectangular mode or rectangular parameters remain.

The trapezoid matcher intersects fitted sides with the rear line, checks rear
width, forward extent, corner support, and **signed outward flare on both sides**.
It rejects inward-tapering bays and missing faces. `trapezoid_max_line_gap: 0.30`
allows collinear rear-wall returns across the scanner blind sector; the V extractor retains its previous 0.10 m gap. The search radius is 0.95 m to include the sides and association margin. Larger gaps and search regions can admit
more clutter, so false-positive testing in the full world is still necessary.

Restart Gazebo with the edited `test_robot_world.sdf` and restart the detector to
load these changes. An already running world retains its old walls. Existing maps
may contain the previous bay; update the map before relying on Nav2 planning near
the changed walls. Historical rectangular-bay measurements below are not results
for the new geometry. The resized matcher passes synthetic noise/clutter/rejection checks and SDF
inner-dimension checks. In ideal 0.5-degree angular scans with a 5-degree rear gap,
matching succeeds from centered world X=0.0, X=0.8, and X=1.3 (Y=3); it rejects
the sampled X=0.8, Y=3±0.2 positions because too little corner support remains. These are sampled visibility limits, not a measured acquisition boundary.
Live Gazebo coverage and detector-driven docking remain unmeasured for this size.

Seggy uses a 355-degree scan, leaving a 5-degree rear blind sector. Minimum range is 0.3 m. All required faces (three for trapezoid, two for V) must retain enough returns;
missing a required face produces no new pose. No stale pose is republished. A steady-clock
watchdog clears detection status after `detection_timeout`, even when simulation
pauses; scan age is also checked against ROS time. Markers expire and are cleared
on detection loss. Scan timestamps and availability of exact-time TF are required.
The node briefly retries the newest scan when TF arrives late. Motion distortion
within a scan is not compensated; begin stationary, then validate at slow speed.

## V landmark mode

`target_shape: v` matches two straight arms meeting at an apex. The internal fit yaw is their
outward bisector; the published dock yaw is opposite and `v_offset` sets the distance from apex to the desired
parking point along that bisector. The selected V dock uses two 0.70 m arms. **Per-arm flare** is measured from
the opening axis, so `expected_angle` is twice the flare: 20–45° flare means a
40–90° included V opening. The selected V default is **110° included opening (55° per arm)**, with 70 cm
panels. Angle sweeps are historical evaluations; this is the fixed V geometry
for subsequent placement and docking tests.
The V matcher uses its existing RANSAC/TLS two-line fit, endpoint/apex support and
association checks; it does not use the three-panel joint refinement.

V regression checks include noisy/rotated targets, missing arms, wrong opening,
wrong-facing targets, the original 90° V, and raycast scans across 20–45° flare.
A trapezoid's displaced virtual apex must not be mistaken for the configured V.
V diagnostics expose match status and fitted RMS. Confidence is unavailable
(`null` in diagnostics, zero on the scalar confidence topic), not a calibrated
low score. The real detector must be pointed at an actual V geometry before
switching this mode on.

A two-arm V has no 0.50 m rear-width offset: entrance width is
`2 * arm_length * sin(flare)`, considerably narrower than the trapezoid. The earlier 0.425 m parking offset had inadequate V footprint clearance. The
new perpendicular V dock uses `v_offset: 0.6` and the recommended apex placement.
Historical offline sweeps retain their original offsets for comparison.

## Two docks in the test world

`articubot_one/assets/worlds/test_robot_world.sdf` contains both selected docks:

| Dock | Panels / flare | Final world pose (X,Y,yaw) | Staging world XY | Landmark world XY | Profile |
|---|---|---|---|---|---|
| Trapezoid | 3 × 0.50 m; 32.5° sides | (-1, 3, 0°) | (-0.2, 3) | Rear midpoint (-1.425, 3) | `config/seggy_t.yaml` |
| V | 2 × 0.70 m; 55° per arm | (0, 6, 270°) | (0, 5.2) | Apex (0, 6.6) | `config/seggy_v.yaml` |

The V is rotated 90° relative to the trapezoid. Its robot faces world -Y and
reverses north from staging to final; the trapezoid robot faces +X and reverses
west. Dark blue/green floor areas distinguish them, green crosses mark final
base_link points, cyan crosses mark staging, yellow guides follow inner faces,
and white arrows show reverse travel. The V has an apex marker rather than a
rear-wall guide. Floor markings are visual only; all five wall panels have
collisions. Panel height is 0.50 m and thickness 0.05 m.

Rebuild both packages and restart Gazebo to load the two-dock world. Run one
detector at a time, selecting the corresponding profile. The default launch uses
the trapezoid profile; for the V:

```bash
ros2 launch dock_detector detector.launch.py \
  params_file:=/home/sergei/robot_ws/src/dock_detector/config/seggy_v.yaml
```

Profiles contain coarse map-frame parking priors. The table gives Gazebo world
references; verify map/world alignment before commanding Nav2 goals. Existing
maps may contain old walls. No charging behavior is modeled.

## Fixed V placement recommendation

The V angle is fixed at 55° per arm (110° included), with two 70 cm panels.
The offline placement study recommends centering the V on the final heading and
putting its apex **0.60 m behind the final base_link point**, rather than 0.425 m.
For final world (-1,3,yaw=0), this proposes apex (-1.6,3). The corresponding
V parking transform is `v_offset: 0.6`.
This placement is applied to the new perpendicular V dock in the saved test world;
the trapezoid retains its own location and `trapezoid_offset: 0.425`.

The proposed placement gives 16.4 cm nominal footprint clearance and 6.1 cm
minimum clearance over sampled ±5 cm fore/aft, ±10 cm lateral and ±10° yaw errors.
Useful coverage over the modeled 10 cm approach corridor grid is 97.3%, including
footprint feasibility. This is an offline, sampled result with a perfect prior;
it needs validation in Gazebo and does not guarantee continuous tolerance-box
clearance. See `results/2026-11-06-v55-placement/run_01/README.md` and
`placement_recommendation.png` for dimensions, selection rule and raw data.

## Verify

```bash
PYTHONPATH=src/dock_detector:$PYTHONPATH python3 -m pytest -q src/dock_detector/test
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


## Offline flare-angle sweep

Run a fixed-panel geometry comparison without ROS or Gazebo:

```bash
PYTHONPATH=src/dock_detector:$PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 src/dock_detector/scripts/sweep_flare.py \
  --output /tmp/seggy-flare-sweep --workers 4
```

Add `--shape v` to sweep the two-arm V and automatically include the fixed 32.5°
trapezoid as a baseline. This mode ranks exactly the placements collision-free
for every V angle and the fixed trapezoid; its denominator differs from the
historical trapezoid-only sweep. Both use the same grid, headings, sensor and noise
seeds. The V report is in `results/2026-11-06-v-flare-sweep/run_01/`.

For the 70 cm V-panel comparison over 40–60° per arm:

```bash
PYTHONPATH=src/dock_detector:$PYTHONPATH OPENBLAS_NUM_THREADS=1 \
  python3 src/dock_detector/scripts/sweep_flare.py --shape v \
  --v-panel-length 0.7 --angles 40 45 50 55 60 \
  --output /tmp/seggy-v70-sweep --workers 4
MPLCONFIGDIR=/tmp/dock-sweep-mpl python3 src/dock_detector/scripts/report_flare_sweep.py /tmp/seggy-v70-sweep
```

These options affect offline geometry only. The trapezoid comparison keeps three
50 cm panels at 32.5°. The completed run and the paired 50/70 cm comparison are in
`results/2026-11-06-v70-flare-sweep/run_01/`.

Choose a new output directory. The sweep tests 20–45° in 5° steps on a 10 cm
world-coordinate grid spanning the marked practice area and staging approach,
five headings and three noise repetitions. It uses Seggy's 360-ray, 355° view,
0.3 m minimum range, 1 cm Gaussian noise and the actual Nav2 footprint. Geometry
checks verify that offline wall boxes match the SDF. Worker processes are CPU-only;
no detector, world, or motion setting is changed. Results are JSONL/CSV plus
metadata with source hashes. Archived report scripts in the respective result directories generate plots and rankings.

The main comparison uses placements collision-free for every angle. It separates
raw three-face visibility from actual detector success and scores the published
parking point. Three additional ideal 6 Hz reverse paths illustrate detection
gaps. This simplified model omits unrelated clutter, reflectance, additional
self-occlusion, and real-time TF/control effects; validate promising angles with
identical Gazebo poses before choosing physical geometry.

## Joint fitting and confidence diagnostics

Trapezoid mode now uses `joint_refinement: true`. The existing three-line geometry
matcher supplies an initial pose. A bounded robust fit then adjusts **only X, Y,
and yaw** against all three finite template segments. Panel dimensions and flare
remain fixed. Huber weighting reduces outlier influence and panel balancing
prevents a densely sampled face from overwhelming a sparse one. Refinement is
bounded to 5 cm translation and 0.10 rad heading change from its seed. Fits must
converge, retain at least `min_points` and `min_length` support on each panel,
have RMS at most 0.75 times `distance_threshold`, and satisfy the original prior.
This does not interpolate missing returns, smooth across scans, or acquire a bay
when the original geometric seed fails. Rejected refinements publish no pose.

New topics (under the detector namespace):

- `/dock_detector/diagnostics`: `diagnostic_msgs/msg/DiagnosticArray`, with the
  source scan timestamp for scan results. Reports `matched`, `insufficient_lines`,
  `no_geometric_candidate`, refinement rejection reasons, `waiting_for_tf`,
  `stale_scan`, or `no_fresh_scan`. Rejection counters identify geometric gates
  such as `rear_span`, `side_extent_or_corner_gap`, and `rear_width`; they count
  candidate attempts, not scans, and can be nonzero even when a match succeeds.
- `/dock_detector/confidence`: `std_msgs/msg/Float32`, a **heuristic 0–1 quality
  score**, not a calibrated probability or accuracy bound. Combines fit RMS,
  weakest panel point count, and weakest panel span coverage. Zero on rejection,
  lost scan freshness, or when refinement is disabled. Diagnostics distinguish
  disabled confidence (`null`) from rejected fits.

Successful refinement diagnostics include `initial_rms_m`, `final_rms_m`,
`correction_m`, `correction_yaw_rad`, `iterations`, `converged`, `condition`,
`inlier_fraction`, and `panels` (rear, lower side, upper side in template axes).
Each panel reports point count, supported span in metres, and coverage fraction.
Initial RMS uses all seed-selected points; final RMS uses final inliers, so these
are useful fit summaries but are not an identical-population accuracy comparison.
`condition` describes the scaled fit Jacobian, not pose covariance. Counters
`scans_received`, `scans_processed`, `matches`, and `match_fraction` cover the
current process lifetime; TF/stale input failures do not increment processed scans.
`processing_ms` measures matching and refinement time.

For the simulation smoke test, rebuild/source and run **one detector**:

```bash
ros2 launch dock_detector detector.launch.py
# In separate sourced terminals:
ros2 topic echo /dock_detector/diagnostics
ros2 topic echo /dock_detector/confidence
ros2 topic hz /detected_dock_pose
```

Move Seggy between poses and inspect match fraction, rejection reasons, the three
panel support values, RMS, confidence, and pose stability. A stable wrong match
can still score well; compare against the known simulated bay when assessing
accuracy. The RViz green segments now show the complete fitted template; measured
support is reported in diagnostics, so a drawn segment does not imply returns
along its whole length. To compare the previous line-only result at the same pose:

```bash
# Stop the other detector first. This runs the baseline with the same config.
ros2 run dock_detector dock_detector --ros-args \
  --params-file ~/robot_ws/src/dock_detector/config/seggy_t.yaml \
  -p joint_refinement:=false
```

## Reusable live-test controller

The experimental controller is available as `scripts/dock_controller.py` and as
`ros2 run dock_detector dock_controller` after rebuilding and sourcing the workspace.
It uses ROS TF and optionally detected parking poses; it never queries Gazebo.
The detector node itself remains perception-only. This controller is a bounded
positioning experiment, not a Nav2 docking action server or charging controller.

Start Seggy's normal navigation stack so TF, the velocity smoother, and collision
monitor are running. Cancel navigation/docking actions and stop other autonomous
command producers before using it. Keep manual stop control available. Defaults
use stamped commands on `cmd_vel_raw`, through Seggy's existing chain:
`cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor ->
cmd_vel_nav -> twist_mux`. Other robots need a verified equivalent route; do not
remap the output directly to the motor controller. Action-status checks cover the
listed standard Nav2 actions, but do not provide exclusive ownership of motion.

For a physical robot, first preview a nearby, surveyed parking pose in **odom**
(metres and radians; these coordinates are only an example):

```bash
ros2 run dock_detector dock_controller --goal -1.0 3.0 0.0
# After checking the preview and the physical approach:
ros2 run dock_detector dock_controller --goal -1.0 3.0 0.0 --execute
```

Alternatively start the detector with hardware-calibrated parameters, verify that
its output represents the desired base_link parking pose, and preview tracking:

```bash
ros2 run dock_detector dock_controller --detected-goal
ros2 run dock_detector dock_controller --detected-goal --execute --log /tmp/dock-live-01.jsonl
```

Logs use exclusive creation, so choose a new filename for each run. The source
wrapper can also be run after sourcing the built workspace:
`python3 src/dock_detector/scripts/dock_controller.py --detected-goal`.
Use `--help` for all settings; `--ros-args -r __ns:=/robot` supports namespaced nodes
and topics. Frames themselves are set with `--fixed-frame` and `--base-frame`.
For simulation add `--use-sim-time`; omit it on hardware. Fixed goals must use the
selected fixed frame, not uncalibrated Gazebo world coordinates.

Preview creates no command publisher. Execution defaults to reverse motion at
at most 0.035 m/s and 0.08 rad/s; `--forward` supports forward positioning.
Initial range and cumulative XY travel are each limited to 1.5 m; total runtime
is limited to 120 seconds. Success requires sustained alignment for 2 seconds.
The controller checks active collision monitoring, competing Nav2 actions, TF
freshness and jumps, detection freshness, pose jumps, and motion stalls. Control
and command publication are capped at 20 Hz independently of callback traffic.
Stale TF, inactive collision monitoring, or a competing action stops motion on
the next control iteration (at most approximately 50 ms, subject to scheduling).
Other missing inputs permit only the detection grace behavior below, followed
by a stopped reacquisition interval and abort. Shutdown sends zero commands at
20 Hz for 2 seconds;
downstream smoothing and arbitration still determine physical stopping behavior.

Detected mode requires five fresh, consistent poses before initial motion.
For **reverse detected-goal mode**, `--detection-grace` now defaults to **3 seconds**:
after the normal 0.5-second detection freshness window, the controller may continue
reversing toward its last fully acquired goal using live odometry/TF. It continues
to steer toward that fixed goal; it does not replay a frozen velocity command.
At the default speed cap, the extra grace corresponds to at most about 10.5 cm
of requested reverse travel. Existing overall time, travel, and safety checks
still apply. There is no blind initial acquisition or forward-mode grace.

Only a fully reacquired five-pose goal refreshes the grace deadline. An isolated
pose cannot keep blind motion alive indefinitely. A detected pose jump revokes
the cached goal immediately and requires reacquisition. Wall time and ROS stamp
age both bound the cache, so a clock rollback/paused simulation cannot indefinitely
extend it. If the cached goal is reached or would require in-place alignment,
the controller stops and waits for fresh detection; it cannot declare success
using a cached goal alone. If grace expires, it stops, waits up to `--loss-timeout`
(default 3 s) for reacquisition, then aborts. `--detection-grace 0` restores immediate
stopping when detection freshness expires. Events `detection_grace_started`,
`detection_grace_ended`, and `stopped_waiting_for_fresh_input` appear in JSONL logs.

```bash
ros2 run dock_detector dock_controller --detected-goal --execute --use-sim-time \
  --detection-grace 3 --log /tmp/dock-grace-01.jsonl
```

Omit `--use-sim-time` on hardware. Earlier tests found poor acquisition from rough
staging, so preposition where the detector already tracks reliably. The new grace
behavior has automated tests but has not yet been exercised in a motion trial.
Detector-driven steering on hardware has not yet been validated.

Verification includes planar convergence, detection acquisition/loss/jumps, frozen
TF with paused ROS time, and the detector geometry tests. These checks command no
robot motion; they do not establish physical docking accuracy.

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
