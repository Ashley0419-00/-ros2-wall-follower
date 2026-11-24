#!/usr/bin/env python3
"""Map batched visual/LiDAR observations using measurement-time optical TF.

Accumulate in odom and reproject with current map->odom to follow SLAM changes.
Legacy ID topics are diagnostic outputs and are not mapping inputs.

Stage 2 marker file (landmarks_file), rewritten every second during the run:
    start_x, start_y, start_heading
    marker_x, marker_y, marker_type
    ...
start_x, start_y, start_heading are the wall follower's first /odom pose, as given.
Markers are the SLAM-corrected positions in the map frame.

odom and map only coincide if odom was zero when Cartographer started (not in Gazebo,
whose odom is world-referenced). So the same start pose expressed in the map frame is
written to a separate file, <landmarks_file>.map_start ("x, y, heading"), which
marker_navigator uses as the initial pose. The marker file itself keeps the spec format.
"""
import copy
import math
import os
import tempfile
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
from wall_follower.landmark import marker_type, max_markers, Landmark


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.declare_parameter('tf_wait_sec', 0.5)
        self.declare_parameter('max_detection_age_sec', 1.0)
        self.declare_parameter('landmarks_file', '')  # set by the launch file; '' disables saving
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.observation_subscriber = self.create_subscription(
            MarkerArray, '/marker_observations', self.observation_callback, 10)
        # Latched start pose published once by the wall follower.
        self.start_odom = None   # (x, y, heading) from /odom
        self.start_stamp = None
        self.start_map = None    # the same pose in map, from map->odom at the start
        self.start_subscriber = self.create_subscription(
            PoseStamped, '/wall_follower/start_pose', self.start_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.marker_publisher_ = self.create_publisher(
            MarkerArray, '/visualization_marker_array',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_array = MarkerArray()
        self.marker_position = [Landmark(i, self.marker_array.markers)
                                for i in range(max_markers)]
        self._display_cache = {}
        self._map_stamp = None
        self.pending = deque(maxlen=60)
        self.pending_timer = self.create_timer(0.05, self.process_pending)
        self.republish_timer = self.create_timer(1.0, self.periodic_update)
        self._gate_radius = 0.50
        self._reset_after = 10
        self._candidates = [[] for _ in range(max_markers)]

    def periodic_update(self):
        self.update_start_map()
        self.publish_markers()
        self.saveLandmarks()

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def start_callback(self, msg):
        self.start_odom = (msg.pose.position.x, msg.pose.position.y, self.yaw(msg.pose.orientation))
        self.start_stamp = msg.header.stamp
        self.start_map = None
        self.get_logger().info('Start pose (odom): x=%.3f y=%.3f heading=%.3f' % self.start_odom)
        self.update_start_map()

    def update_start_map(self):
        """Express the odom start in map with map->odom at the start time (latest if
        Cartographer started after the wall follower; its early drift is small)."""
        if self.start_odom is None or self.start_map is not None:
            return
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'odom', rclpy.time.Time.from_msg(self.start_stamp))
            except tf2_ros.ExtrapolationException:
                transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().warn('No map->odom yet; map start pose pending (is Cartographer up?)',
                                   throttle_duration_sec=10.0)
            return
        x, y, heading = self.start_odom
        t = transform.transform.translation
        yaw = self.yaw(transform.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        heading = math.atan2(math.sin(heading + yaw), math.cos(heading + yaw))
        self.start_map = (t.x + c * x - s * y, t.y + s * x + c * y, heading)
        self.get_logger().info('Start pose (map): x=%.3f y=%.3f heading=%.3f' % self.start_map)

    def publish_markers(self):
        if not self.marker_array.markers:
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        # Average measurements in continuous odom coordinates. Reproject the mean
        # with the latest SLAM correction; never average different map revisions.
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        # Rebase after a TF interruption. Otherwise update a landmark only when
        # its observed odom estimate changes, including while republishing at 1 Hz.
        if self._map_stamp is not None and not 0 <= stamp - self._map_stamp <= 3:
            self._display_cache.clear()
        self._map_stamp = stamp
        output = copy.deepcopy(self.marker_array)
        for marker in output.markers:
            lm = self.marker_position[marker.id // 2]
            x, y = lm.estimate()
            cached = self._display_cache.get(marker.id)
            if cached and cached[:2] == (x, y):
                marker.pose.position = cached[2]
                continue
            point = PointStamped()
            point.header.frame_id = 'odom'
            point.point.x, point.point.y = x, y
            point.point.z = marker.pose.position.z
            marker.pose.position = tf2_geometry_msgs.do_transform_point(point, transform).point
            self._display_cache[marker.id] = (x, y, marker.pose.position)
        self.last_map_markers = output.markers
        self.marker_publisher_.publish(output)

    def observation_callback(self, batch):
        for marker in batch.markers:
            if marker.action != marker.ADD:
                continue
            source, sep, kind = marker.ns.partition('/')
            if not sep or source not in ('approx', 'lidar') or kind not in marker_type:
                self.get_logger().warn('Ignoring unknown observation namespace: ' + marker.ns,
                                       throttle_duration_sec=2.0)
                continue
            point = PointStamped()
            point.header = marker.header
            point.point = marker.pose.position
            self.point_callback(point, marker_type.index(kind), source)

    def saveLandmarks(self):
        filename = self.get_parameter('landmarks_file').value
        if not filename:
            return
        if self.start_odom is None:
            self.get_logger().warn('No /wall_follower/start_pose yet; marker file not written',
                                   throttle_duration_sec=10.0)
            return
        # Top-half markers only (even ids), one per marker type, in map coordinates.
        markers = sorted((m for m in getattr(self, 'last_map_markers', []) if m.id % 2 == 0),
                         key=lambda m: m.id)
        lines = ['%.3f, %.3f, %.3f' % self.start_odom]
        lines += ['%.3f, %.3f, %s' % (m.pose.position.x, m.pose.position.y,
                                      marker_type[m.id // 2]) for m in markers]
        path = Path(filename).expanduser().resolve()
        if self.write_once(path, '\n'.join(lines) + '\n'):
            self.get_logger().info('Saved start + %d markers to %s' % (len(markers), path),
                                   throttle_duration_sec=10.0)
        if self.start_map is not None:
            self.write_once(Path(str(path) + '.map_start'), '%.3f, %.3f, %.3f\n' % self.start_map)

    def write_once(self, path, content):
        """Atomically replace path with content unless unchanged; True if written."""
        saved = self.__dict__.setdefault('_saved_content', {})
        if saved.get(path) == content:
            return False
        tmp = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', newline='', dir=path.parent,
                                             prefix=path.name + '.', delete=False) as f:
                tmp = f.name
                f.write(content)
            os.replace(tmp, path)
            saved[path] = content
            return True
        except OSError as exc:
            if tmp:
                Path(tmp).unlink(missing_ok=True)
            self.get_logger().error('Cannot save %s: %s' % (path, exc))
            return False

    def point_callback(self, msg, marker_id, source='approx'):
        coords = (msg.point.x, msg.point.y, msg.point.z)
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if (not msg.header.frame_id or msg.header.frame_id.startswith('/') or
                stamp_ns <= 0 or not all(math.isfinite(v) for v in coords) or
                (source == 'approx' and msg.point.z <= 0) or not 0 <= marker_id < max_markers):
            self.get_logger().warn('Dropping invalid marker coordinates/frame/timestamp',
                                   throttle_duration_sec=2.0)
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        max_age = float(self.get_parameter('max_detection_age_sec').value)
        if age > max_age or age < -0.1:
            self.get_logger().warn('Dropping unsynchronised/stale image (age %.3fs)' % age,
                                   throttle_duration_sec=2.0)
            return
        self.pending.append((time.monotonic(), marker_id, msg))

    def process_pending(self):
        # Nonblocking retries allow TransformListener to receive TF in this executor.
        now = time.monotonic()
        max_wait = float(self.get_parameter('tf_wait_sec').value)
        for _ in range(len(self.pending)):
            arrival, marker_id, msg = self.pending.popleft()
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                if now - arrival < max_wait:
                    self.pending.append((arrival, marker_id, msg))
                else:
                    self.get_logger().warn('Image-time TF unavailable; dropping marker: %s' % exc,
                                           throttle_duration_sec=2.0)
                continue
            point = tf2_geometry_msgs.do_transform_point(msg, transform).point
            if all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                self.update_landmark(marker_id, point)

    def update_landmark(self, marker_id, point):
        lm = self.marker_position[marker_id]
        candidates = self._candidates[marker_id]
        if lm.count:
            ex, ey = lm.estimate()
            if math.hypot(point.x - ex, point.y - ey) > self._gate_radius:
                if candidates:
                    cx = sum(p.x for p in candidates) / len(candidates)
                    cy = sum(p.y for p in candidates) / len(candidates)
                    if math.hypot(point.x - cx, point.y - cy) > 0.15:
                        candidates.clear()
                candidates.append(point)
                if len(candidates) < self._reset_after:
                    return
                lm.reset()
                for candidate in candidates:
                    lm.update_position(candidate)
                candidates.clear()
                self.get_logger().warn('Re-seeded consistent cluster for ' + marker_type[marker_id])
                self.publish_markers()
                return
        candidates.clear()
        lm.update_position(point)
        self.publish_markers()


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.saveLandmarks()  # final write; the 1 Hz saves already cover a hard kill
        except Exception as exc:  # logging may be unavailable during shutdown
            print('Final marker save failed: %s' % exc)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()