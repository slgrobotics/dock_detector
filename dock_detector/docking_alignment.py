"""Complete a named Nav2 docking goal with a stationary yaw correction."""
from collections import deque
from dataclasses import dataclass, replace
import math
from statistics import median
from threading import Lock
from time import monotonic, sleep
from uuid import uuid4

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import DockRobot, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformException, TransformListener

from dock_detector_interfaces.msg import DockDetection
from dock_detector_interfaces.srv import SelectDock
from .core.pose_controller import wrap
from .core.transforms import rotation


@dataclass
class AlignmentSample:
    robot: tuple[float, float]
    detected: tuple[float, float]
    filtered: tuple[float, float]
    scan_stamp: Time
    robot_yaw: float
    detected_yaw_error: float
    yaw_error: float

    @property
    def distance(self):
        return math.dist(self.robot, self.filtered)


class AlignmentTrace:
    """Log each phase change immediately and repeated status at most twice a second."""

    def __init__(self, logger, dock):
        self.logger = logger
        self.dock = dock
        self.phase = None
        self.last_status = 0.0

    def report(self, phase, details=''):
        now = monotonic()
        if phase != self.phase or now - self.last_status >= .5:
            self.logger.info(f'ALIGN: {phase} id={self.dock} {details}'.rstrip())
            self.phase = phase
            self.last_status = now


class DockingFailure(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class DockingAlignment(Node):
    def __init__(self):
        super().__init__('docking_alignment')
        defaults = {
            'fixed_frame': 'odom', 'base_frame': 'base_link', 'dock_direction': 'forward',
            'docking_threshold': .025, 'docking_threshold_yaw': 5.0,
            'alignment_position_margin': 0.02,
            'max_insertion_travel': 0.0, 'insertion_speed': .01,
            'insertion_timeout': 15.0, 'battery_topic': 'battery/battery_state',
            'angular_speed': .05, 'alignment_timeout': 12.0,
            'external_detection_timeout': .5, 'selection_timeout': 3.0,
            'trace_enabled': False,
            'restaging_goal_checker': '', 'max_restaging_retries': 2,
            'nav2_timeout': 1500.0, 'odom_topic': 'odometry/local',
            'selection_service': 'dock_detector/select_dock',
            'detection_topic': 'dock_detector/detection',
        }
        self.settings = {
            name: self.declare_parameter(name, value).value for name, value in defaults.items()
        }
        for name, default in defaults.items():
            if name in ('alignment_position_margin', 'max_insertion_travel'):
                value = self.settings[name]
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f'{name} must be finite and nonnegative')
            elif isinstance(default, float):
                value = self.settings[name]
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{name} must be finite and positive')
        if self.settings['max_restaging_retries'] < 0:
            raise ValueError('max_restaging_retries must be nonnegative')
        self.alignment_position_limit = (
            self.settings['docking_threshold'] + self.settings['alignment_position_margin']
        )
        if not math.isfinite(self.alignment_position_limit):
            raise ValueError('Combined alignment position limit must be finite')
        if self.settings['dock_direction'] not in ('forward', 'backward'):
            raise ValueError('dock_direction must be forward or backward')
        if self.settings['docking_threshold_yaw'] > 180:
            raise ValueError('docking_threshold_yaw must be at most 180 degrees')

        dock_ids = self.declare_parameter('dock_ids', Parameter.Type.STRING_ARRAY).value or []
        self.insertion_travel = {
            dock: self.declare_parameter(
                f'docks.{dock}.max_insertion_travel', self.settings['max_insertion_travel']
            ).value for dock in dock_ids
        }
        if any(not math.isfinite(value) or value < 0 for value in self.insertion_travel.values()):
            raise ValueError('Per-dock max_insertion_travel must be finite and nonnegative')
        if (self.settings['dock_direction'] != 'forward'
                and (self.settings['max_insertion_travel'] > 0
                     or any(self.insertion_travel.values()))):
            raise ValueError('Forward insertion requires dock_direction=forward')

        # Match the logger name in the existing two-line analyzer format.
        self.plot_logger = get_logger('docking_server')
        callbacks = ReentrantCallbackGroup()
        self.lock = Lock()
        self.busy = False
        self.nav2_uncertain = False
        self.selection = None
        self.detections = deque(maxlen=3)
        self.odom = None
        self.battery = None
        self.active_actions = {}
        self.tf = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf, self)
        self.velocity = self.create_publisher(TwistStamped, 'cmd_vel_raw', 10)
        self.selector = self.create_client(
            SelectDock, self.settings['selection_service'], callback_group=callbacks)
        self.pipeline = [
            self.create_client(GetState, name + '/get_state', callback_group=callbacks)
            for name in ('velocity_smoother', 'collision_monitor')
        ]
        self.create_subscription(DockDetection, self.settings['detection_topic'],
                                 self.receive_detection, 10)
        self.create_subscription(Odometry, self.settings['odom_topic'],
                                 self.receive_odom, qos_profile_sensor_data)
        if self.settings['max_insertion_travel'] or any(self.insertion_travel.values()):
            self.create_subscription(BatteryState, self.settings['battery_topic'],
                                     self.receive_battery, qos_profile_sensor_data)
        for name in ('dock_robot_position', 'undock_robot', 'navigate_to_pose',
                     'navigate_through_poses', 'follow_waypoints', 'follow_path',
                     'spin', 'backup', 'drive_on_heading', 'assisted_teleop'):
            self.create_subscription(
                GoalStatusArray, name + '/_action/status',
                lambda msg, action=name: self.receive_status(action, msg),
                qos_profile_action_status_default,
            )
        self.nav2 = ActionClient(self, DockRobot, 'dock_robot_position', callback_group=callbacks)
        self.controller_parameters = AsyncParameterClient(
            self, 'controller_server', callback_group=callbacks)
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                      callback_group=callbacks)
        self.server = ActionServer(
            self, DockRobot, 'dock_robot', self.execute, goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=callbacks,
        )

    def accept(self, request):
        with self.lock:
            if (self.busy or self.nav2_uncertain or any(self.active_actions.values())
                    or not request.use_dock_id
                    or not request.dock_id or not self.nav2.server_is_ready()):
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def receive_status(self, action, msg):
        with self.lock:
            self.active_actions[action] = any(
                status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                  GoalStatus.STATUS_CANCELING)
                for status in msg.status_list
            )

    def receive_odom(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        with self.lock:
            if self.odom is None or stamp > self.odom[0]:
                self.odom = (stamp, monotonic(), msg.twist.twist)

    def receive_battery(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        with self.lock:
            if self.battery is None or stamp > self.battery[0]:
                status = (msg.power_supply_status if msg.present
                          else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
                self.battery = (stamp, monotonic(), status)

    def battery_status(self):
        with self.lock:
            battery = self.battery
        if battery is None or not self.fresh(battery[0], battery[1], 2.5):
            return BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        return battery[2]

    def receive_detection(self, msg):
        stamp = Time.from_msg(msg.pose.header.stamp)
        with self.lock:
            if (self.selection is None
                    or (msg.dock_id, msg.acquisition_id) != self.selection[:2]
                    or stamp <= self.selection[2]
                    or (self.detections and stamp <= self.detections[-1][0])):
                return
            self.detections.append((stamp, monotonic(), msg.pose))

    def publish(self, angular=0.0, *, linear=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.settings['base_frame']
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.velocity.publish(msg)

    def check_cancel(self, goal):
        if goal.is_cancel_requested or not self.context.ok():
            raise DockingFailure(DockRobot.Result.NONE, 'Docking canceled')

    def wait_for(self, future, seconds, goal=None):
        deadline = monotonic() + seconds
        while self.context.ok() and not future.done() and monotonic() < deadline:
            if goal is not None:
                self.check_cancel(goal)
            sleep(.02)
        if not future.done():
            raise TimeoutError('ROS request timed out')
        return future.result()

    def forward(self, goal, trace, *, already_staged=False):
        self.check_cancel(goal)
        trace.report('nav2_position', 'requesting position-only docking')
        states = {
            DockRobot.Feedback.NAV_TO_STAGING_POSE: 'staging',
            DockRobot.Feedback.INITIAL_PERCEPTION: 'perception',
            DockRobot.Feedback.CONTROLLING: 'approach',
            DockRobot.Feedback.WAIT_FOR_CHARGE: 'charge_wait',
            DockRobot.Feedback.RETRY: 'retry',
        }

        def feedback(message):
            status = message.feedback
            phase = states.get(status.state, str(status.state))
            trace.report(f'nav2_{phase}', f'retries={status.num_retries}')
            goal.publish_feedback(status)

        request = goal.request
        if already_staged:
            request = DockRobot.Goal(
                use_dock_id=True, dock_id=goal.request.dock_id,
                navigate_to_staging_pose=False, max_staging_time=goal.request.max_staging_time,
            )
        return self.run_inner_action(goal, trace, self.nav2, request,
                                     self.settings['nav2_timeout'], feedback)

    def run_inner_action(self, goal, trace, client, request, timeout, feedback=None):
        self.check_cancel(goal)
        future = client.send_goal_async(request, feedback_callback=feedback)
        try:
            inner = self.wait_for(future, 3.0)
        except Exception:
            # A delayed acceptance can still start the robot. Cancel it and
            # reject new wrapper goals until the server's state can be checked.
            with self.lock:
                self.nav2_uncertain = True
            future.add_done_callback(self.cancel_late_goal)
            raise
        if not inner.accepted:
            raise DockingFailure(DockRobot.Result.GOAL_REJECTED, 'Nav2 rejected the inner docking or staging goal')
        result = inner.get_result_async()
        try:
            return self.wait_for(result, timeout, goal)
        except (DockingFailure, TimeoutError):
            trace.report('nav2_cancel', 'requesting Nav2 stop')
            try:
                inner.cancel_goal_async()
                self.wait_for(result, 3.0)
            except Exception:
                with self.lock:
                    self.nav2_uncertain = True
                self.get_logger().error('Could not confirm Nav2 stopped. Restart after checking Nav2.')
            raise

    def restage(self, goal, trace, correction):
        try:
            frame, *coordinates = correction.split()
            x, y, yaw = map(float, coordinates)
            if frame != self.settings['fixed_frame'] or not all(
                    math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError('invalid frame or coordinates')
        except ValueError as exc:
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE,
                                 'Invalid LiDAR staging correction') from exc

        deadline = monotonic() + 5.0
        still_since = None
        while monotonic() < deadline:
            self.check_cancel(goal)
            twist = self.current_twist()
            stopped = (self.ensure_idle() and twist is not None
                       and math.hypot(twist.linear.x, twist.linear.y) <= .005
                       and abs(twist.angular.z) <= .01)
            now = monotonic()
            still_since = (still_since or now) if stopped else None
            if still_since is not None and now - still_since >= .3:
                break
            sleep(.05)
        else:
            raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                 'Robot did not stop before LiDAR re-staging')

        self.check_pipeline(goal)
        if not self.navigator.server_is_ready():
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE, 'Nav2 navigation unavailable')
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = frame
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = x, y
        request.pose.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.pose.orientation.w = math.cos(yaw / 2.0)
        trace.report('restage', f'frame={frame} x={x:.3f} y={y:.3f} yaw_deg={math.degrees(yaw):+.2f}')
        timeout = goal.request.max_staging_time
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE, 'Invalid re-staging time limit')
        # Scope tighter tolerances to the correction goal. Seggy uses one goal
        # checker, which also serves callers that omit a goal checker ID.
        checker = self.settings['restaging_goal_checker']
        tolerances = {
            'xy_goal_tolerance': .025, 'yaw_goal_tolerance': .05,
            'stateful': False,
        }
        parameters = [Parameter(f'{checker}.{key}', value=value)
                      for key, value in tolerances.items()]
        if not self.controller_parameters.services_are_ready():
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE,
                                 'Controller parameter services unavailable for re-staging')
        saved = self.wait_for(self.controller_parameters.get_parameters(
            [param.name for param in parameters]), 3.0, goal).values
        if len(saved) != len(parameters) or any(
                value.type != param.type_.value for param, value in zip(parameters, saved)):
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE,
                                 'Could not read the active staging goal checker settings')
        original = [
            Parameter(param.name, value=(value.bool_value
                      if param.type_ == Parameter.Type.BOOL else value.double_value))
            for param, value in zip(parameters, saved)
        ]
        try:
            applied = self.wait_for(
                self.controller_parameters.set_parameters_atomically(parameters), 3.0).result
            if not applied.successful:
                raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE,
                                     'Could not set precise re-staging tolerances: ' + applied.reason)
            response = self.run_inner_action(goal, trace, self.navigator, request, timeout)
        finally:
            try:
                restored = self.wait_for(
                    self.controller_parameters.set_parameters_atomically(original), 3.0).result
                if not restored.successful:
                    raise RuntimeError(restored.reason)
            except Exception as exc:
                with self.lock:
                    self.nav2_uncertain = True
                raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                     'Could not restore navigation goal tolerances') from exc
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code:
            raise DockingFailure(DockRobot.Result.FAILED_TO_STAGE,
                                 'LiDAR re-staging failed: ' + response.result.error_msg)

    def cancel_late_goal(self, future):
        try:
            inner = future.result()
            if inner.accepted:
                inner.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f'Could not cancel late Nav2 goal: {exc}')

    def request_selection(self, activate, dock, token):
        return self.selector.call_async(SelectDock.Request(
            activate=activate, dock_id=dock, acquisition_id=token))

    def ensure_idle(self):
        with self.lock:
            positioning = self.active_actions.get('dock_robot_position', False)
            active = [name for name, busy in self.active_actions.items()
                      if busy and name != 'dock_robot_position']
        if active:
            raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                 f'Competing motion action: {", ".join(active)}')
        return not positioning

    def check_pipeline(self, goal):
        for client in self.pipeline:
            if not client.service_is_ready():
                raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                     f'{client.srv_name} unavailable')
            try:
                state = self.wait_for(client.call_async(GetState.Request()), .25, goal)
            except TimeoutError as exc:
                raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                     f'{client.srv_name} timed out') from exc
            if state.current_state.id != State.PRIMARY_STATE_ACTIVE:
                raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                     f'{client.srv_name} reports an inactive node')

    def fresh(self, stamp, received, timeout):
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return stamp.nanoseconds > 0 and -.1 <= age <= timeout and monotonic() - received <= timeout

    def current_twist(self):
        with self.lock:
            odom = self.odom
        if odom is None or not self.fresh(odom[0], odom[1], .5):
            return None
        twist = odom[2]
        if not all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z)):
            return None
        return twist

    def robot_pose(self):
        tf = self.tf.lookup_transform(
            self.settings['fixed_frame'], self.settings['base_frame'], Time())
        stamp = Time.from_msg(tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if stamp.nanoseconds <= 0 or not -.1 <= age <= self.settings['external_detection_timeout']:
            return None
        robot = tf.transform.translation
        heading = rotation(tf.transform.rotation)
        yaw = math.atan2(heading[1, 0], heading[0, 0])
        if not all(math.isfinite(value) for value in (robot.x, robot.y, yaw)):
            return None
        return robot.x, robot.y, yaw

    def measure_alignment(self):
        with self.lock:
            detections = tuple(self.detections)
        timeout = self.settings['external_detection_timeout']
        fixed = self.settings['fixed_frame']
        poses = []
        scan_stamp = None
        for stamp, received, pose in detections:
            if not self.fresh(stamp, received, timeout) or not pose.header.frame_id:
                continue
            p, q = pose.pose.position, pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                continue
            if abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) > .01:
                continue
            xyz = np.array([p.x, p.y, p.z])
            orientation = rotation(q)
            if pose.header.frame_id != fixed:
                transform = self.tf.lookup_transform(fixed, pose.header.frame_id, stamp).transform
                r = rotation(transform.rotation)
                t = transform.translation
                xyz = r @ xyz + [t.x, t.y, t.z]
                orientation = r @ orientation
            poses.append((xyz[0], xyz[1], math.atan2(orientation[1, 0], orientation[0, 0])))
            scan_stamp = stamp
        if len(poses) < 2:
            return None

        robot_pose = self.robot_pose()
        if robot_pose is None:
            return None
        x, y, robot_yaw = robot_pose
        yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
        offset = math.pi if self.settings['dock_direction'] == 'backward' else 0.0
        raw_x, raw_y, raw_yaw = poses[-1]
        return AlignmentSample(
            robot=(x, y), detected=(raw_x, raw_y),
            filtered=(median(p[0] for p in poses), median(p[1] for p in poses)),
            scan_stamp=scan_stamp, robot_yaw=robot_yaw,
            detected_yaw_error=wrap(raw_yaw + offset - robot_yaw),
            yaw_error=wrap(yaw + offset - robot_yaw),
        )

    def trace_sample(self, dock, sample, phase, *, stage='alignment'):
        if not self.settings['trace_enabled']:
            return
        rx, ry = sample.robot
        dx, dy = sample.detected
        fx, fy = sample.filtered
        distance_limit = 100 * self.alignment_position_limit
        yaw_limit = self.settings['docking_threshold_yaw']
        scan_age_ms = (self.get_clock().now() - sample.scan_stamp).nanoseconds / 1e6
        self.plot_logger.info(
            f'DOCK: {phase} id={dock} frame={self.settings["fixed_frame"]} '
            f'scan_age_ms={scan_age_ms:.1f} robot=({rx:.3f},{ry:.3f}) '
            f'detected=({dx:.3f},{dy:.3f}) filtered=({fx:.3f},{fy:.3f}) '
            f'threshold_cm={distance_limit:.2f} threshold_yaw_deg={yaw_limit:.2f} phase={stage}'
        )
        # DOCK records use robot minus target. The controller uses target minus robot.
        self.plot_logger.info(
            f'      xy_cm={100 * math.dist(sample.robot, sample.detected):.2f} '
            f'yaw_deg={math.degrees(-sample.detected_yaw_error):+.2f} '
            f'filtered_xy_cm={100 * sample.distance:.2f} '
            f'dx_cm={100 * (rx - dx):+.2f} dy_cm={100 * (ry - dy):+.2f} '
            f'filtered_yaw_deg={math.degrees(-sample.yaw_error):+.2f}'
        )

    def align(self, goal, started, result, trace):
        dock, token = goal.request.dock_id, uuid4().hex
        trace.report('reacquire', 'Nav2 position complete; requesting the same dock')
        owns_commands = False
        with self.lock:
            self.selection = (dock, token, self.get_clock().now())
            self.detections.clear()
        try:
            if not self.selector.service_is_ready():
                raise DockingFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK,
                                     'Detector selection service unavailable')
            try:
                response = self.wait_for(self.request_selection(True, dock, token),
                                         self.settings['selection_timeout'], goal)
            except TimeoutError as exc:
                raise DockingFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK,
                                     'Detector selection timed out') from exc
            if not response.success or (response.dock_id, response.acquisition_id) != (dock, token):
                raise DockingFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK,
                                     'Detector refused yaw alignment selection')

            trace.report('detector_selected', 'waiting for fresh detections and stationary odometry')
            deadline = monotonic() + self.settings['alignment_timeout']
            next_check = next_sample = still_since = 0.0
            settled_since = None
            tolerance = math.radians(self.settings['docking_threshold_yaw'])
            maximum = self.settings['angular_speed']
            distance_limit = self.alignment_position_limit
            while monotonic() < deadline:
                self.check_cancel(goal)
                if not self.ensure_idle():
                    if owns_commands:
                        raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                             'Nav2 positioning became active during alignment')
                    trace.report('wait_nav2_idle')
                    sleep(.05)
                    continue
                twist = self.current_twist()
                # We could check for unexpected translation here, but the robot may drift slightly during yaw alignment.
                # The odometry check is sufficient to ensure the robot is stationary before claiming alignment.
                #if owns_commands and twist is not None and math.hypot(twist.linear.x, twist.linear.y) > .01:
                #    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                #                         'Unexpected translation during yaw alignment')
                if twist is None and owns_commands:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         'Odometry unavailable during yaw alignment')
                measured = 'odom=unavailable'
                stopped = False
                if twist is not None:
                    speed = math.hypot(twist.linear.x, twist.linear.y)
                    stopped = speed <= .005 and abs(twist.angular.z) <= .01
                    measured = f'odom_linear_mps={speed:.3f} odom_angular_radps={twist.angular.z:+.3f}'
                now = monotonic()
                if now >= next_check:
                    self.check_pipeline(goal)
                    next_check = now + .5
                try:
                    sample = self.measure_alignment()
                except TransformException:
                    sample = None

                if not owns_commands:
                    still_since = (still_since or now) if stopped else 0.0
                    if not still_since or now - still_since < .3 or sample is None:
                        trace.report(
                            'wait_handoff',
                            f'{measured} stationary={stopped} dock_pose_ready={sample is not None}',
                        )
                        sleep(.05)
                        continue
                    trace.report('handoff_ready', 'odometry stationary and dock pose fresh')
                    owns_commands = True
                if sample is None:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK,
                                         'Dock pose or robot TF unavailable during alignment')
                self.check_cancel(goal)
                if now >= next_sample:
                    self.trace_sample(dock, sample, 'approach')
                    next_sample = now + .5
                distance, yaw = sample.distance, sample.yaw_error
                status = (
                    f'xy_cm={distance * 100:.2f} threshold_cm={distance_limit * 100:.2f} '
                    f'yaw_deg={math.degrees(yaw):+.2f} threshold_yaw_deg={math.degrees(tolerance):.2f}'
                )
                if not math.isfinite(yaw) or not distance <= distance_limit:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         f'Outside parking tolerance during yaw alignment ({status})')
                if abs(yaw) <= tolerance:
                    self.publish()
                    trace.report('settle', f'{status} cmd_linear_mps=0.000 '
                                 f'cmd_angular_radps=0.000 {measured}')
                    settled_since = (settled_since or now) if stopped else None
                    if settled_since is not None and now - settled_since >= .3:
                        trace.report('aligned', status)
                        return sample
                else:
                    settled_since = None
                    angular = max(-maximum, min(maximum, 1.2 * yaw))
                    self.publish(angular)
                    trace.report('turn', f'{status} cmd_linear_mps=0.000 '
                                 f'cmd_angular_radps={angular:+.3f} {measured}')
                feedback = DockRobot.Feedback(state=DockRobot.Feedback.CONTROLLING,
                                              num_retries=result.num_retries)
                feedback.docking_time = (self.get_clock().now() - started).to_msg()
                goal.publish_feedback(feedback)
                sleep(.05)
            raise DockingFailure(DockRobot.Result.TIMEOUT, 'Yaw alignment did not settle before timeout')
        finally:
            if owns_commands and self.context.ok():
                self.publish()
                trace.report('stop', 'published zero velocity')
            with self.lock:
                self.selection = None
                self.detections.clear()
            if self.context.ok() and self.selector.service_is_ready():
                try:
                    trace.report('release', 'releasing detector selection')
                    self.wait_for(self.request_selection(False, dock, token), .5)
                except Exception as exc:
                    self.get_logger().warning(f'Detector release could not be confirmed: {exc}')

    def insert(self, goal, started, result, trace, aligned, limit):
        dock = goal.request.dock_id
        trace.report('insertion_start', f'max_travel_cm={100 * limit:.2f}')
        deadline = monotonic() + self.settings['insertion_timeout']
        next_check = next_sample = 0.0
        origin = None
        stopped_since = None
        stop_reason = None
        try:
            while monotonic() < deadline:
                self.check_cancel(goal)
                if not self.ensure_idle():
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         'Nav2 positioning became active during insertion')
                now = monotonic()
                if now >= next_check:
                    self.check_pipeline(goal)
                    next_check = now + .5

                twist = self.current_twist()
                try:
                    pose = self.robot_pose()
                except TransformException:
                    pose = None
                if twist is None or pose is None:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         'Odometry or robot TF unavailable during insertion')
                x, y, yaw = pose
                if origin is None:
                    origin = pose
                dx, dy = x - origin[0], y - origin[1]
                forward = dx * math.cos(origin[2]) + dy * math.sin(origin[2])
                lateral = -dx * math.sin(origin[2]) + dy * math.cos(origin[2])
                travel = math.hypot(dx, dy)
                heading_drift = wrap(yaw - aligned.robot_yaw)
                sample = replace(
                    aligned, robot=(x, y), robot_yaw=yaw,
                    detected_yaw_error=wrap(aligned.detected_yaw_error - heading_drift),
                    yaw_error=wrap(aligned.yaw_error - heading_drift),
                )
                if (forward < -.005 or abs(lateral) > .01
                        or abs(sample.yaw_error) > math.radians(self.settings['docking_threshold_yaw'])):
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         'Robot drifted from the aligned insertion path')
                if travel > limit:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CONTROL,
                                         f'Exceeded insertion travel limit ({100 * travel:.2f} cm)')

                battery = self.battery_status()
                if battery == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN:
                    raise DockingFailure(DockRobot.Result.FAILED_TO_CHARGE,
                                         'Fresh battery charging status unavailable')
                charging = battery == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                remaining = limit - travel
                # Allow for the 6 Hz smoother and a control cycle before the stop takes effect.
                braking_room = .001 + .25 * max(0.0, twist.linear.x)
                if charging:
                    stop_reason = 'charging'
                elif remaining <= braking_room and stop_reason is None:
                    stop_reason = 'travel_limit'

                self.check_cancel(goal)
                max_speed = self.settings['insertion_speed']
                speed = 0.0
                if stop_reason is None:
                    speed = min(max_speed, max(.3 * max_speed, 2.0 * (remaining - braking_room)))
                self.publish(linear=speed)
                if now >= next_sample:
                    self.trace_sample(dock, sample, 'approach', stage='insertion')
                    next_sample = now + .5
                status = (f'travel_cm={100 * forward:.2f} max_travel_cm={100 * limit:.2f} '
                          f'cmd_linear_mps={speed:.3f} battery_status={battery}')
                if stop_reason:
                    trace.report('insertion_wait_charge', f'reason={stop_reason} {status}')
                    still = (math.hypot(twist.linear.x, twist.linear.y) <= .005
                             and abs(twist.angular.z) <= .01)
                    stopped_since = (stopped_since or now) if still else None
                    if stopped_since is not None and now - stopped_since >= .3 and charging:
                        trace.report('insertion_charging', status)
                        return sample
                    if stopped_since is not None and now - stopped_since >= 2.5:
                        raise DockingFailure(DockRobot.Result.FAILED_TO_CHARGE,
                                             'Charging not confirmed after stopping insertion')
                else:
                    trace.report('insertion_advance', status)
                feedback = DockRobot.Feedback(
                    state=DockRobot.Feedback.WAIT_FOR_CHARGE if stop_reason else DockRobot.Feedback.CONTROLLING,
                    num_retries=result.num_retries,
                )
                feedback.docking_time = (self.get_clock().now() - started).to_msg()
                goal.publish_feedback(feedback)
                sleep(.05)
            raise DockingFailure(DockRobot.Result.FAILED_TO_CHARGE, 'Insertion timed out without confirmed charging')
        finally:
            if self.context.ok():
                self.publish()
                trace.report('insertion_stop', 'published zero velocity')

    def execute(self, goal):
        result = DockRobot.Result(success=False)
        trace = AlignmentTrace(self.get_logger(), goal.request.dock_id)
        try:
            started = self.get_clock().now()
            retries = 0
            while True:
                response = self.forward(goal, trace, already_staged=retries > 0)
                self.check_cancel(goal)
                result = response.result
                prefix = 'LiDAR staging rejected: '
                if (response.status != GoalStatus.STATUS_ABORTED
                        or result.error_code != DockRobot.Result.FAILED_TO_STAGE
                        or not result.error_msg.startswith(prefix)
                        or not goal.request.navigate_to_staging_pose
                        or not self.settings['restaging_goal_checker']
                        or retries >= self.settings['max_restaging_retries']):
                    break
                retries += 1
                self.restage(goal, trace, result.error_msg[len(prefix):])
            result.num_retries = min(65535, result.num_retries + retries)
            if response.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
                result.success = False
                goal.abort()
                return result
            result.success = False
            sample = self.align(goal, started, result, trace)
            dock = goal.request.dock_id
            limit = self.insertion_travel.get(dock, self.settings['max_insertion_travel'])
            if limit > 0:
                sample = self.insert(goal, started, result, trace, sample, limit)
            self.check_cancel(goal)
            result.success = True
            goal.succeed()
            self.trace_sample(dock, sample, 'arrival', stage='insertion' if limit else 'alignment')
        except Exception as exc:
            result.success = False
            result.error_code = (
                exc.code if isinstance(exc, DockingFailure) else
                DockRobot.Result.TIMEOUT if isinstance(exc, TimeoutError) else
                DockRobot.Result.UNKNOWN
            )
            result.error_msg = str(exc)
            if goal.is_cancel_requested:
                goal.canceled()
            elif self.context.ok():
                self.get_logger().warning(result.error_msg)
                goal.abort()
        finally:
            trace.report('result', f'success={result.success} error_code={result.error_code} '
                         f'reason={result.error_msg or "-"}')
            with self.lock:
                self.busy = False
        return result


def main():
    rclpy.init()
    node = DockingAlignment()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
