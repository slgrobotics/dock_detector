"""Perception only: no actions, velocity commands, or simulator ground truth."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from .geometry import detect_v, detect_trapezoid


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_points(xy, tf):
    xyz = np.column_stack([xy, np.zeros(len(xy))])
    t = tf.transform.translation
    return (xyz @ rotation(tf.transform.rotation).T + [t.x, t.y, t.z])[:, :2]


class DockDetector(Node):
    def __init__(self):
        super().__init__('dock_detector')
        defaults = dict(target_shape='trapezoid', trapezoid_width=0.5,
                        trapezoid_depth=0.5*math.cos(math.radians(32.5)),
                        trapezoid_flare_angle=math.radians(32.5), trapezoid_max_line_gap=0.30,
                        joint_refinement=True,
                        dimension_tolerance=0.10,
                        max_corner_gap=0.18, scan_topic='scan', pose_topic='detected_dock_pose',
                        fixed_frame='odom', expected_frame='map',
                        expected_dock_x=-1.0, expected_dock_y=3.0, expected_dock_yaw=0.0,
                        v_offset=0.6, trapezoid_offset=0.425,
                        search_radius=0.85, max_scan_age=0.5, detection_timeout=0.6,
                        distance_threshold=0.025, min_points=5, min_length=0.12,
                        max_length=1.30, expected_angle=math.radians(110),
                        angle_tolerance=0.20, position_tolerance=0.45,
                        yaw_tolerance=0.45, max_apex_gap=0.25)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for k in ('search_radius', 'max_scan_age', 'detection_timeout', 'distance_threshold',
                  'min_length', 'max_length', 'position_tolerance', 'yaw_tolerance',
                  'angle_tolerance', 'max_apex_gap',
                  'dimension_tolerance', 'max_corner_gap', 'trapezoid_width', 'trapezoid_depth', 'trapezoid_max_line_gap'):
            if self.cfg[k] <= 0:
                raise ValueError(f'{k} must be positive')
        for k in ('v_offset', 'trapezoid_offset'):
            if not math.isfinite(self.cfg[k]) or self.cfg[k] < 0:
                raise ValueError(f'{k} must be finite and nonnegative')
        if self.cfg['min_points'] < 3 or self.cfg['min_length'] >= self.cfg['max_length']:
            raise ValueError('Require min_points >= 3 and min_length < max_length')
        if not 0 < self.cfg['expected_angle'] < math.pi:
            raise ValueError('expected_angle must be between zero and pi')
        if not 0 < self.cfg['trapezoid_flare_angle'] < math.pi/2:
            raise ValueError('trapezoid_flare_angle must be between zero and pi/2')
        if self.cfg['target_shape'] not in ('v', 'trapezoid'):
            raise ValueError('target_shape must be v or trapezoid')
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, self.cfg['pose_topic'], 10)
        self.status = self.create_publisher(Bool, '~/detected', 10)
        self.markers = self.create_publisher(MarkerArray, '~/markers', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '~/diagnostics', 10)
        self.confidence_pub = self.create_publisher(Float32, '~/confidence', 10)
        self.last_diagnostic_wall = 0.
        self.scan_count = self.processed_count = self.match_count = 0
        self.last_scan_wall = time.monotonic()
        self.pending = None
        self.last_detection = None
        self.last_processed_stamp = None
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, qos_profile_sensor_data)
        # Retry a scan briefly when TF is slightly behind the sensor stream.
        self.create_timer(0.05, self.process, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Dock detector active: {self.cfg['target_shape']} (perception only); parameters are startup-only.")

    def on_scan(self, scan):
        self.scan_count += 1
        self.last_scan_wall = time.monotonic()
        self.pending = scan
        self.received_at = time.monotonic()
        self.process()

    def report(self, reason, metrics=None, scan=None, throttled=False):
        now = time.monotonic()
        if throttled and now-self.last_diagnostic_wall < 1.:
            return
        self.last_diagnostic_wall = now
        values = dict(metrics or {})
        score = values.get('confidence')
        values.update(reason=reason, shape=self.cfg['target_shape'],
                      scans_received=self.scan_count, scans_processed=self.processed_count,
                      matches=self.match_count,
                      match_fraction=self.match_count/max(1, self.processed_count))
        msg = DiagnosticArray()
        msg.header.stamp = scan.header.stamp if scan is not None else self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['fixed_frame']
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name()+'/match'
        status.hardware_id = 'lidar_dock_detector'
        status.level = DiagnosticStatus.OK if reason == 'matched' else DiagnosticStatus.WARN
        status.message = reason
        status.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False))
                         for k, v in values.items()]
        msg.status = [status]
        self.diagnostic_pub.publish(msg)
        self.confidence_pub.publish(Float32(data=float(score) if score is not None else 0.))

    def invalidate(self):
        self.status.publish(Bool(data=False))
        delete = Marker(action=Marker.DELETEALL)
        self.markers.publish(MarkerArray(markers=[delete]))
        self.last_detection = None

    def process(self):
        c = self.cfg
        now = time.monotonic()
        if self.last_detection is not None and now-self.last_detection > c['detection_timeout']:
            self.invalidate()
            self.report('detection_timeout', throttled=True)
        if now-self.last_scan_wall > c['detection_timeout']:
            self.invalidate()
            self.report('no_fresh_scan', throttled=True)
        scan = self.pending
        if scan is None:
            return
        stamp = Time.from_msg(scan.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds / 1e9
        if age > c['max_scan_age'] or age < -0.1 or now-self.received_at > c['max_scan_age']:
            self.pending = None
            self.invalidate()
            self.report('stale_scan', {'scan_age_s': age}, scan)
            return
        key = (scan.header.stamp.sec, scan.header.stamp.nanosec)
        if key == self.last_processed_stamp:
            self.pending = None
            return
        if not scan.header.frame_id or not scan.ranges or scan.angle_increment == 0:
            self.pending = None
            self.invalidate()
            self.report('invalid_scan', scan=scan)
            return
        try:
            laser_tf = self.buffer.lookup_transform(c['fixed_frame'], scan.header.frame_id, stamp)
            prior_tf = self.buffer.lookup_transform(c['fixed_frame'], c['expected_frame'], stamp)
        except TransformException:
            self.report('waiting_for_tf', scan=scan, throttled=True)
            return
        self.pending = None
        self.last_processed_stamp = key
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        good = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        xy = np.column_stack([ranges[good]*np.cos(angles[good]), ranges[good]*np.sin(angles[good])])
        points = transform_points(xy, laser_tf)
        prior = transform_points(np.array([[c['expected_dock_x'], c['expected_dock_y']]]), prior_tf)[0]
        rot = rotation(prior_tf.transform.rotation)
        expected_yaw = c['expected_dock_yaw']+math.atan2(rot[1, 0], rot[0, 0])
        offset = c['v_offset'] if c['target_shape'] == 'v' else c['trapezoid_offset']
        expected_apex = prior-offset*np.array([math.cos(expected_yaw), math.sin(expected_yaw)])
        points = points[np.linalg.norm(points-expected_apex, axis=1) <= c['search_radius']]
        keys = ('distance_threshold', 'min_points', 'min_length', 'max_length',
                'angle_tolerance', 'position_tolerance', 'yaw_tolerance')
        kwargs = {k: c[k] for k in keys}
        metrics = {}
        fit_started = time.monotonic()
        if c['target_shape'] == 'trapezoid':
            detection = detect_trapezoid(
                points, expected_apex, expected_yaw, **kwargs,
                width=c['trapezoid_width'], depth=c['trapezoid_depth'],
                flare_angle=c['trapezoid_flare_angle'], max_line_gap=c['trapezoid_max_line_gap'],
                joint_refinement=c['joint_refinement'], diagnostics=metrics,
                dimension_tolerance=c['dimension_tolerance'], max_corner_gap=c['max_corner_gap'])
        else:
            detection = detect_v(points, expected_apex, expected_yaw, **kwargs,
                                 expected_angle=c['expected_angle'], max_apex_gap=c['max_apex_gap'])
            metrics.update(reason='matched' if detection is not None else 'no_v_candidate',
                           refinement='not_applicable', confidence=None)
            if detection is not None:
                metrics['fit_rms_m'] = detection.rms
        self.processed_count += 1
        metrics['processing_ms'] = (time.monotonic()-fit_started)*1000
        if detection is None:
            self.invalidate()
            self.report(metrics.get('reason', 'no_geometric_candidate'), metrics, scan)
            return
        self.match_count += 1
        self.report('matched', metrics, scan)
        pose = PoseStamped()
        pose.header.frame_id = c['fixed_frame']
        pose.header.stamp = scan.header.stamp  # Never replace measurement time with now.
        dock = detection.apex+offset*np.array([math.cos(detection.yaw), math.sin(detection.yaw)])
        pose.pose.position.x, pose.pose.position.y = map(float, dock)
        pose.pose.orientation.z = math.sin(detection.yaw/2)
        pose.pose.orientation.w = math.cos(detection.yaw/2)
        self.pub.publish(pose)
        self.status.publish(Bool(data=True))
        self.last_detection = now
        line = Marker()
        line.header = pose.header
        line.ns = 'dock_target'; line.id = 0; line.type = Marker.LINE_LIST
        line.action = Marker.ADD; line.pose.orientation.w = 1.0
        line.scale.x = 0.02; line.color.g = 1.0; line.color.a = 1.0
        line.lifetime.sec = 1
        for arm in detection.arms:
            for x, y in arm:
                line.points.append(Point(x=float(x), y=float(y), z=0.1))
        arrow = Marker()
        arrow.header = pose.header; arrow.ns = 'parking_pose'; arrow.id = 1
        arrow.type = Marker.ARROW; arrow.action = Marker.ADD; arrow.pose = pose.pose
        arrow.scale.x = 0.3; arrow.scale.y = 0.04; arrow.scale.z = 0.04
        arrow.color.b = 1.0; arrow.color.a = 1.0; arrow.lifetime.sec = 1
        self.markers.publish(MarkerArray(markers=[line, arrow]))


def main(args=None):
    rclpy.init(args=args)
    node = DockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
