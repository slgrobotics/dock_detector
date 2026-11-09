"""Bounded ROS TF/detected-pose controller; no Gazebo or hardware-specific code."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.utilities import remove_ros_args
from rclpy.parameter import Parameter
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener, TransformException
from .core.pose_controller import Limits, PoseController, GoalMemory, cached_reverse_command, wrap
from .core.transforms import rotation


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='Fixed parking pose in --fixed-frame (metres/radians)')
    source.add_argument('--detected-goal', action='store_true', help='Use fresh detected dock poses')
    p.add_argument('--execute', action='store_true', help='Publish motion; default is preview only')
    p.add_argument('--use-sim-time', action='store_true', help='Use /clock; omit for physical robot')
    p.add_argument('--forward', action='store_true', help='Forward positioning instead of reverse docking')
    p.add_argument('--fixed-frame', default='odom')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--pose-topic', default='detected_dock_pose')
    p.add_argument('--cmd-vel-topic', default='cmd_vel_raw', help='Upstream safety-pipeline input')
    p.add_argument('--collision-monitor-node', default='collision_monitor')
    p.add_argument('--speed', type=float, default=.035)
    p.add_argument('--angular-speed', type=float, default=.08)
    p.add_argument('--xy-tolerance', type=float, default=.025)
    p.add_argument('--yaw-tolerance', type=float, default=math.radians(2))
    p.add_argument('--settled-xy-tolerance', type=float, default=.04)
    p.add_argument('--settled-yaw-tolerance', type=float, default=math.radians(3))
    p.add_argument('--settle-time', type=float, default=2.)
    p.add_argument('--timeout', type=float, default=120., help='Overall wall-clock limit')
    p.add_argument('--max-distance', type=float, default=1.5, help='Initial goal range AND travel budget (m)')
    p.add_argument('--tf-timeout', type=float, default=.4)
    p.add_argument('--detection-timeout', type=float, default=.5)
    p.add_argument('--detection-grace', type=float, default=3.,
                   help='Extra seconds to reverse toward the last fully acquired goal after detection expires; 0 disables')
    p.add_argument('--acquire-timeout', type=float, default=10.)
    p.add_argument('--loss-timeout', type=float, default=3., help='Stopped reacquisition interval before abort')
    p.add_argument('--acquire-samples', type=int, default=5)
    p.add_argument('--max-goal-jump', type=float, default=.10)
    p.add_argument('--max-goal-yaw-jump', type=float, default=.15)
    p.add_argument('--stall-timeout', type=float, default=15.)
    p.add_argument('--preview-duration', type=float, default=3.)
    p.add_argument('--log', type=Path, help='JSONL output; must be a new path')
    return p


def validate(args):
    for name in ('speed', 'angular_speed', 'xy_tolerance', 'yaw_tolerance',
                 'settled_xy_tolerance', 'settled_yaw_tolerance', 'settle_time',
                 'timeout', 'max_distance', 'tf_timeout', 'detection_timeout',
                 'acquire_timeout', 'loss_timeout', 'max_goal_jump',
                 'max_goal_yaw_jump', 'stall_timeout', 'preview_duration'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if not math.isfinite(args.detection_grace) or args.detection_grace < 0:
        raise ValueError('detection_grace must be finite and nonnegative')
    if args.acquire_samples < 1:
        raise ValueError('acquire_samples must be positive')
    if args.goal and not all(math.isfinite(v) for v in args.goal):
        raise ValueError('goal must be finite')
    if args.settled_xy_tolerance < args.xy_tolerance or args.settled_yaw_tolerance < args.yaw_tolerance:
        raise ValueError('settled tolerances must be at least control tolerances')


class DockController(Node):
    def __init__(self, args):
        super().__init__('dock_test_controller', parameter_overrides=[
            Parameter('use_sim_time', value=args.use_sim_time)])
        self.args = args
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(TwistStamped, args.cmd_vel_topic, 10) if args.execute else None
        self.samples = deque(maxlen=args.acquire_samples)
        self.goal_memory = GoalMemory()
        self.last_pose_stamp = None
        self.last_pose_wall = None
        self.goal = tuple(args.goal) if args.goal else None
        self.actions = {}
        self.monitor_state = None
        self.monitor_wall = None
        self.monitor_future = None
        self.next_monitor_check = 0.
        self.active_motion = False
        self.robot_stamp = None
        self.robot_stamp_wall = None
        self.logfile = args.log.open('x') if args.log else None
        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 10)
        for action in ('navigate_to_pose', 'navigate_through_poses', 'dock_robot',
                       'undock_robot', 'follow_path', 'spin', 'backup', 'drive_on_heading'):
            self.create_subscription(GoalStatusArray, action+'/_action/status',
                                     lambda m, name=action: self.actions.update({name: any(
                                         s.status in (1, 2, 3) for s in m.status_list)}),
                                     qos_profile_action_status_default)
        service = args.collision_monitor_node.rstrip('/')+'/get_state'
        self.monitor = self.create_client(GetState, service)

    def emit(self, event, **data):
        row = {'wall_time': time.monotonic(), 'event': event, **data}
        self.get_logger().info(json.dumps(row))
        if self.logfile:
            self.logfile.write(json.dumps(row)+'\n')
            self.logfile.flush()

    def on_pose(self, msg):
        if not self.args.detected_goal:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds <= 0 or not msg.header.frame_id:
            return
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if age < -.1 or age > self.args.detection_timeout:
            return
        if self.last_pose_stamp is not None and stamp.nanoseconds <= self.last_pose_stamp:
            return  # Duplicate/old packets must not refresh freshness or acquisition.
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not all(math.isfinite(v) for v in values) or abs(norm-1.) > .01:
            return
        try:
            tf = self.buffer.lookup_transform(self.args.fixed_frame, msg.header.frame_id, stamp).transform
        except TransformException:
            return
        r = rotation(tf.rotation)
        xyz = r @ np.array([p.x, p.y, p.z])+[tf.translation.x, tf.translation.y, tf.translation.z]
        orientation = r @ rotation(q)
        goal = (float(xyz[0]), float(xyz[1]), wrap(math.atan2(orientation[1, 0], orientation[0, 0]) + (0. if self.args.forward else math.pi)))
        if self.samples or self.goal_memory.goal is not None:
            old = self.samples[-1] if self.samples else self.goal_memory.goal
            if math.hypot(goal[0]-old[0], goal[1]-old[1]) > self.args.max_goal_jump or abs(wrap(goal[2]-old[2])) > self.args.max_goal_yaw_jump:
                self.samples.clear()
                self.goal = None
                self.goal_memory.clear()
                self.emit('goal_jump_reacquire')
        if self.last_pose_wall is not None and time.monotonic()-self.last_pose_wall > self.args.detection_timeout:
            self.samples.clear()
            self.goal = None
        self.samples.append(goal)
        self.last_pose_stamp = stamp.nanoseconds
        self.last_pose_wall = time.monotonic()
        if len(self.samples) == self.args.acquire_samples:
            self.goal = (float(np.median([v[0] for v in self.samples])),
                         float(np.median([v[1] for v in self.samples])),
                         math.atan2(sum(math.sin(v[2]) for v in self.samples),
                                    sum(math.cos(v[2]) for v in self.samples)))
            self.goal_memory.accept(self.goal, self.last_pose_wall, self.last_pose_stamp)

    def detected_goal(self):
        if not self.args.detected_goal:
            return self.goal
        if self.last_pose_wall is None:
            return None
        age = (self.get_clock().now().nanoseconds-self.last_pose_stamp)/1e9
        if time.monotonic()-self.last_pose_wall > self.args.detection_timeout or not -.1 <= age <= self.args.detection_timeout:
            self.samples.clear()
            self.goal = None
        return self.goal

    def robot(self):
        tf = self.buffer.lookup_transform(self.args.fixed_frame, self.args.base_frame, Time())
        age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        if not -.1 <= age <= self.args.tf_timeout:
            raise RuntimeError('Robot TF is stale')
        stamp = Time.from_msg(tf.header.stamp).nanoseconds
        now = time.monotonic()
        if stamp != self.robot_stamp:
            self.robot_stamp, self.robot_stamp_wall = stamp, now
        if now-self.robot_stamp_wall > self.args.tf_timeout:
            raise RuntimeError('Robot TF stopped updating')
        t, q = tf.transform.translation, tf.transform.rotation
        if not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
            raise RuntimeError('Robot TF contains nonfinite values')
        if abs(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.) > .01:
            raise RuntimeError('Robot TF quaternion is invalid')
        return (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def monitor_active(self):
        now = time.monotonic()
        if self.monitor_future is not None and self.monitor_future.done():
            try:
                self.monitor_state = self.monitor_future.result().current_state.id
                self.monitor_wall = now
            except Exception:
                self.monitor_state = None
            self.monitor_future = None
        if now >= self.next_monitor_check and self.monitor_future is None and self.monitor.service_is_ready():
            self.monitor_future = self.monitor.call_async(GetState.Request())
            self.next_monitor_check = now+1.
        return self.monitor_state == 3 and self.monitor_wall is not None and now-self.monitor_wall < 2.

    def publish(self, v=0., w=0.):
        if self.publisher is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.base_frame
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.publisher.publish(msg)

    def run(self, stopped):
        a = self.args
        law = PoseController(Limits(a.speed, a.angular_speed, a.xy_tolerance,
                                    a.yaw_tolerance, not a.forward))
        started = time.monotonic()
        missing_since = started
        initialized = False
        previous = progress_pose = None
        travelled = 0.
        last_progress = started
        settled_since = preview_since = None
        last_report = 0.
        next_control = started
        using_cached = False
        self.emit('start', execute=a.execute, fixed_frame=a.fixed_frame,
                  goal_source='detected' if a.detected_goal else 'fixed', command_topic=a.cmd_vel_topic,
                  detection_grace=a.detection_grace, control_rate_hz=20.)
        while rclpy.ok() and not stopped():
            rclpy.spin_once(self, timeout_sec=.05)
            now = time.monotonic()
            # Callbacks can arrive much faster than 20 Hz. Bound control and
            # command publication independently of incoming TF/message traffic.
            if now < next_control:
                continue
            next_control = now+.05
            if now-started > a.timeout:
                raise RuntimeError('Overall wall-clock timeout')
            if any(self.actions.values()):
                raise RuntimeError('A competing Nav2 motion action is active')
            ready = self.monitor_active()
            try:
                robot = self.robot()
            except (TransformException, RuntimeError):
                robot = None
            goal = self.detected_goal()
            cached = False
            if (goal is None and initialized and a.detected_goal and not a.forward
                    and a.detection_grace > 0 and ready and robot is not None):
                goal = self.goal_memory.lookup(now, self.get_clock().now().nanoseconds,
                                               a.detection_timeout, a.detection_grace)
                cached = goal is not None
            if cached != using_cached:
                self.emit('detection_grace_started' if cached else 'detection_grace_ended',
                          goal=goal, detection_grace=a.detection_grace)
                using_cached = cached
            # Allow action-status discovery before the first possible command.
            if now-started < 1. or not ready or robot is None or goal is None:
                if self.active_motion:
                    self.publish()
                if initialized:
                    settled_since = None
                    law.reset()
                    if missing_since is None:
                        missing_since = now
                        self.emit('stopped_waiting_for_fresh_input', tf_ready=robot is not None,
                                  monitor_ready=ready, goal_ready=goal is not None)
                    if now-missing_since > a.loss_timeout:
                        raise RuntimeError('Fresh TF/detection or active collision monitor unavailable')
                elif now-started > a.acquire_timeout:
                    raise RuntimeError('Readiness/acquisition timeout; no motion started')
                continue
            if not initialized:
                if math.hypot(goal[0]-robot[0], goal[1]-robot[1]) > a.max_distance:
                    raise RuntimeError('Initial goal exceeds max-distance')
                initialized = True
                previous = progress_pose = robot
                preview_since = last_progress = now
                self.emit('ready', robot=robot, goal=goal)
            missing_since = None
            displacement = math.hypot(robot[0]-previous[0], robot[1]-previous[1])
            if displacement > .2 or abs(wrap(robot[2]-previous[2])) > .3:
                raise RuntimeError('Robot TF jumped')
            travelled += displacement
            previous = robot
            if travelled > a.max_distance:
                raise RuntimeError('Travel budget exceeded')
            if math.hypot(goal[0]-robot[0], goal[1]-robot[1]) > a.max_distance:
                raise RuntimeError('Goal moved outside allowed range')
            if math.hypot(robot[0]-progress_pose[0], robot[1]-progress_pose[1]) > .01 or abs(wrap(robot[2]-progress_pose[2])) > .05:
                progress_pose = robot
                last_progress = now
            if cached:
                v, w, mode = cached_reverse_command(law, robot, goal)
            else:
                v, w, mode = law.command(robot, goal)
            if mode == 'settle':
                settled_since = settled_since or now
                if now-settled_since >= a.settle_time:
                    if math.hypot(goal[0]-robot[0], goal[1]-robot[1]) <= a.settled_xy_tolerance and abs(wrap(goal[2]-robot[2])) <= a.settled_yaw_tolerance:
                        self.emit('succeeded' if a.execute else 'preview_complete', robot=robot, goal=goal, travelled=travelled)
                        return 0
                    law.reset()
                    settled_since = None
            else:
                settled_since = None
            if a.execute:
                if now-last_progress > a.stall_timeout and mode not in ('settle', 'wait_for_detection'):
                    raise RuntimeError('Motion stalled; safety arbitration may be blocking')
                self.active_motion = True
                self.publish(v, w)
            elif now-preview_since >= a.preview_duration:
                self.emit('preview_complete', robot=robot, goal=goal)
                return 0
            if now-last_report >= 1.:
                self.emit('control', mode=mode, robot=robot, goal=goal, requested_v=v, requested_w=w, travelled=travelled)
                last_report = now
        raise RuntimeError('Interrupted')


def main(args=None):
    argv = sys.argv if args is None else ['dock_controller.py', *args]
    p = parser()
    config = p.parse_args(remove_ros_args(argv)[1:])
    try:
        validate(config)
    except ValueError as e:
        p.error(str(e))
    rclpy.init(args=argv)
    stopped = [False]
    old_handlers = {s: signal.signal(s, lambda *_: stopped.__setitem__(0, True)) for s in (signal.SIGINT, signal.SIGTERM)}
    node = None
    result = 1
    try:
        node = DockController(config)
        result = node.run(lambda: stopped[0])
    except Exception as e:
        if node is not None:
            node.emit('failed', reason=str(e))
        else:
            print(str(e), file=sys.stderr)
    finally:
        if node is not None:
            # Continue zero commands through downstream smoothing on completion/error.
            if node.active_motion and rclpy.ok():
                until = time.monotonic()+2.
                next_zero = 0.
                while time.monotonic() < until:
                    if time.monotonic() >= next_zero:
                        node.publish()
                        next_zero = time.monotonic()+.05
                    rclpy.spin_once(node, timeout_sec=.05)
            if node.logfile:
                node.logfile.close()
            node.destroy_node()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        rclpy.try_shutdown()
    return result
