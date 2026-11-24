#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Fixed vision node for real TurtleBot3 marker detection.
#
# Detects two-colour pillar landmarks (pink + blue/green/yellow, stacked) in the
# camera image and publishes their position on /marker_position.
#
# Message convention (PointStamped, expressed in the camera OPTICAL frame):
#   point.x = metres to the right of the camera axis
#   point.y = marker index into landmark.marker_type   (an ID, NOT a coordinate)
#   point.z = metres forward along the camera axis
# Legacy /marker_position keeps this format for existing monitors.
# Mapping uses /marker_observations batches with full xyz and source/type namespaces.
# /marker_position/id_<ID> remains an optical-height diagnostic interface only.

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from rclpy.time import Time
from wall_follower.beacon_tracking import matrix, lidar_match

from wall_follower.landmark import marker_type

HFOV_DEG = 62.2          # only used if no CameraInfo arrives
MIN_AREA = 150           # px, minimum blob area
CONFIRM_FRAMES = 3       # consecutive consistent frames before publishing
MAX_JUMP_FRAC = 0.10     # max horizontal movement between frames (fraction of width)
MAX_H_CHANGE = 0.40      # max relative change in blob height between frames

COLOURS = {
    "pink":   ((140, 80, 60), (170, 255, 255)),
    "blue":   ((90, 80, 60), (130, 255, 255)),
    "green":  ((40,  80, 60), (80,  255, 255)),
    "yellow": ((25,  80, 60), (32,  255, 255)),
}
OTHER_COLOURS = ("blue", "green", "yellow")
DRAW_BGR = {"pink": (180, 105, 255), "blue": (255, 0, 0),
            "green": (0, 255, 0), "yellow": (0, 255, 255)}

_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))


def find_blobs(hsv, colour, bounds=None, min_area=MIN_AREA):
    """Return ALL valid blobs of one colour (not just the largest)."""
    lower, upper = bounds if bounds is not None else COLOURS[colour]
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

    blobs = []
    for i in range(1, n):
        x = int(stats[i, cv2.CC_STAT_LEFT])
        y = int(stats[i, cv2.CC_STAT_TOP])
        w = int(stats[i, cv2.CC_STAT_WIDTH])
        h = int(stats[i, cv2.CC_STAT_HEIGHT])
        area = int(stats[i, cv2.CC_STAT_AREA])
        if w <= 0 or h <= 0:
            continue
        solidity = area / float(w * h)
        aspect = h / float(w)
        if area < min_area or aspect < 0.4 or aspect > 4.0 or solidity < 0.5:
            continue
        blobs.append({"x": x, "y": y, "w": w, "h": h,
                      "cx": float(cents[i][0]), "cy": float(cents[i][1])})
    return blobs


def pair_blobs(pinks, others):
    """
    Pair each coloured blob with a vertically adjacent pink blob.
    Returns list of (top_colour, bottom_colour, pink_blob, other_blob).
    Each blob is used at most once (closest pairs win).
    """
    cands = []
    for colour, blobs in others.items():
        for bi, b in enumerate(blobs):
            for pi, p in enumerate(pinks):
                overlap = min(p["x"] + p["w"], b["x"] + b["w"]) - max(p["x"], b["x"])
                if overlap < 0.6 * min(p["w"], b["w"]):
                    continue
                if not 0.6 < p["h"] / float(b["h"]) < 1.6:   # both halves same size
                    continue
                if b["y"] < p["y"]:
                    gap = p["y"] - (b["y"] + b["h"])
                    top, bottom = colour, "pink"
                else:
                    gap = b["y"] - (p["y"] + p["h"])
                    top, bottom = "pink", colour
                if gap > 0.6 * max(p["h"], b["h"]):
                    continue
                if gap < -0.3 * min(p["h"], b["h"]):          # excessive overlap
                    continue
                cands.append((gap, pi, colour, bi, top, bottom))

    cands.sort(key=lambda c: c[0])
    used_p, used_b, pairs = set(), set(), []
    for gap, pi, colour, bi, top, bottom in cands:
        if pi in used_p or (colour, bi) in used_b:
            continue
        used_p.add(pi)
        used_b.add((colour, bi))
        pairs.append((top, bottom, pinks[pi], others[colour][bi]))
    return pairs


class SeeMarker(Node):
    def __init__(self):
        super().__init__('see_marker')
        defaults = dict(show_debug=False, processing_rate_hz=10.0,
                        min_area_reference_px=150.0, confirmation_gap_sec=0.5,
                        publish_legacy_position=True, camera_frame='', marker_height_m=.10,
                        fallback_focal_scale=0.88827, max_range_m=2.0, use_lidar=True,
                        allow_approximate=True, cylinder_diameter_m=.10,
                        sync_tolerance_sec=.08, scan_topic='/scan', odom_frame='odom')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for colour, (lower, upper) in COLOURS.items():
            self.declare_parameter(colour + '_hsv_lower', list(lower))
            self.declare_parameter(colour + '_hsv_upper', list(upper))
        rate = float(self.p('processing_rate_hz'))
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('processing_rate_hz must be positive')
        self.latest_image = None
        self.last_processed_time = None
        self.last_image_stamp = -1
        self.screen_tracks = []
        self.br = CvBridge()
        self.info = None
        self.scans = deque(maxlen=30)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.listener_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p('scan_topic'), self.scans.append, qos_profile_sensor_data)
        self.point_publisher = self.create_publisher(PointStamped, '/marker_position', 10)
        self.position_publishers = [self.create_publisher(PointStamped, '/marker_position/id_' + str(i), 10)
                                    for i in range(len(marker_type))]
        # Each array is one observation batch. ns=source/type, pose is full xyz,
        # header names its actual measurement frame/time. IDs are batch-local only.
        self.observation_publisher = self.create_publisher(MarkerArray, '/marker_observations', 10)
        self.processing_timer = self.create_timer(1.0/rate, self.process_latest)

    def p(self, name):
        return self.get_parameter(name).value

    def camera_info_callback(self, msg):
        if (msg.width > 0 and msg.height > 0 and
                all(math.isfinite(v) for v in msg.k) and msg.k[0] > 0 and msg.k[4] > 0):
            self.info = msg
        else:
            self.info = None

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    def intrinsics(self, width, height):
        info = self.info
        if info:
            sx, sy = width / info.width, height / info.height
            return info.k[0]*sx, info.k[4]*sy, info.k[2]*sx, info.k[5]*sy
        scale = float(self.p('fallback_focal_scale'))
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('fallback_focal_scale must be positive')
        self.get_logger().warn('No calibrated CameraInfo: approximate height range only; LiDAR fusion disabled',
                               throttle_duration_sec=10.)
        f = scale * width/2 / math.tan(math.radians(HFOV_DEG)/2)
        return f, f, width/2, height/2

    def listener_callback(self, data):
        self.latest_image = data

    def fused_point(self, data, frame_id, box, width, height):
        if not self.p('use_lidar') or not self.info:
            return None
        info = self.info
        # Only raw, matching CameraInfo is safe for scan projection. Do not use the
        # empirical 2.466 factor as if it were calibrated lens geometry.
        if ((info.width, info.height) != (width, height) or
                info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial') or
                not all(math.isfinite(v) for v in info.d)):
            return None
        if not self.scans:
            return None
        image_ns = self.stamp_ns(data.header.stamp)
        scan = min(self.scans, key=lambda v: abs(self.stamp_ns(v.header.stamp)-image_ns))
        if abs(self.stamp_ns(scan.header.stamp)-image_ns)/1e9 > self.p('sync_tolerance_sec'):
            return None
        try:
            transform = self.tf_buffer.lookup_transform_full(
                frame_id, Time.from_msg(data.header.stamp), scan.header.frame_id,
                Time.from_msg(scan.header.stamp), self.p('odom_frame'))
            point = lidar_match(scan, matrix(transform.transform),
                                np.array(info.k).reshape(3, 3),
                                np.array(info.d, dtype=float) if info.d else np.zeros(5),
                                box, self.p('cylinder_diameter_m'), self.p('max_range_m'))
        except (tf2_ros.TransformException, ValueError, cv2.error) as exc:
            self.get_logger().warn('LiDAR association unavailable: ' + str(exc), throttle_duration_sec=5.)
            return None
        if point is None:
            return None
        return point, scan.header

    def draw_label(self, frame, text, x, y, colour):
        width = frame.shape[1]
        scale = min(.5, max(.25, width/550.))
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        if tw+4 > width:
            scale *= (width-4)/(tw+4)
            (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        x = max(2, min(x, width-tw-2)); y = max(th+2, y-4)
        cv2.rectangle(frame, (x-2, y-th-2), (x+tw+2, y+baseline+1), (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1)

    def process_latest(self):
        data, self.latest_image = self.latest_image, None
        now = time.monotonic()
        if self.last_processed_time is not None and now-self.last_processed_time > self.p('confirmation_gap_sec'):
            self.screen_tracks = []
        if data is None:
            return
        stamp = self.stamp_ns(data.header.stamp)
        if stamp <= self.last_image_stamp:
            return
        self.last_image_stamp = stamp
        self.last_processed_time = now
        try:
            if data.encoding.lower() == 'nv21':
                stride = data.step
                rows = data.height * 3 // 2
                raw = np.frombuffer(data.data, dtype=np.uint8)
                if (data.width % 2 or data.height % 2 or stride < data.width
                        or raw.size < rows * stride):
                    raise ValueError(f'Invalid NV21 layout {data.width}x{data.height}, '
                                     f'stride={stride}, bytes={raw.size}')
                planes = raw[:rows * stride].reshape(rows, stride)[:, :data.width]
                frame = cv2.cvtColor(planes, cv2.COLOR_YUV2BGR_NV21)
            else:
                frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        except Exception as exc:
            self.screen_tracks = []
            self.get_logger().warn('Image conversion failed: '+str(exc), throttle_duration_sec=2.)
            return
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        fx, fy, cx, cy = self.intrinsics(width, height)
        frame_id = self.p('camera_frame') or data.header.frame_id
        area = max(9., self.p('min_area_reference_px') * width*height/(640.*480.))
        blobs = {c: find_blobs(hsv, c, (self.p(c+'_hsv_lower'), self.p(c+'_hsv_upper')), area)
                 for c in COLOURS}
        pairs = pair_blobs(blobs['pink'], {c: blobs[c] for c in OTHER_COLOURS})
        used, next_tracks, observations = set(), [], MarkerArray()
        for top, bottom, pink, other in pairs:
            kind = marker_type.index(top+'/'+bottom)
            x, y = min(pink['x'], other['x']), min(pink['y'], other['y'])
            right = max(pink['x']+pink['w'], other['x']+other['w'])
            end = max(pink['y']+pink['h'], other['y']+other['h'])
            u, v = .5*(pink['cx']+other['cx']), .5*(pink['cy']+other['cy'])
            h = .5*(pink['h']+other['h'])
            matches = [(abs(u-t['u']), i, t) for i, t in enumerate(self.screen_tracks)
                       if i not in used and t['kind'] == kind and
                       abs(u-t['u']) < MAX_JUMP_FRAC*width and
                       abs(v-t['v']) < .15*height and abs(h-t['h']) <= MAX_H_CHANGE*t['h']]
            count = 1
            if matches:
                _, idx, previous = min(matches, key=lambda a: a[0]); used.add(idx)
                count = previous['count']+1
            next_tracks.append(dict(kind=kind, u=u, v=v, h=h, count=count))
            cropped = x <= 1 or y <= 1 or right >= width-2 or end >= height-2
            z = fy * self.p('marker_height_m') / h
            point = np.array([(u-cx)*z/fx, (v-cy)*z/fy, z])
            source, header = 'approx', data.header
            label = marker_type[kind] + (' cropped' if cropped else ' %.2fm approx' % z)
            if not cropped and count >= CONFIRM_FRAMES:
                fused = self.fused_point(data, frame_id, (x, y, right-x, end-y), width, height)
                if fused:
                    point, header = fused
                    source = 'lidar'
                    label = marker_type[kind] + ' lidar'
                elif not self.p('allow_approximate'):
                    label = marker_type[kind] + ' no range'
                if source == 'lidar' or (self.p('allow_approximate') and 0 < z <= self.p('max_range_m')):
                    if source == 'approx' and 'optical' not in frame_id.lower():
                        # Approximate point is in optical-frame coordinates. If the
                        # frame_id is not an optical frame, TF will convert the wrong way.
                        self.get_logger().warn(
                            'Approximate point uses optical-frame coordinates but '
                            'frame_id=%s; set camera_frame:=<..._optical_frame> for correct TF.' % frame_id,
                            throttle_duration_sec=10.)
                    observation = Marker()
                    observation.header.stamp = header.stamp
                    observation.header.frame_id = header.frame_id if source == 'lidar' else frame_id
                    observation.ns = source + '/' + marker_type[kind]
                    observation.id = len(observations.markers)
                    observation.type, observation.action = Marker.SPHERE, Marker.ADD
                    observation.pose.position.x, observation.pose.position.y, observation.pose.position.z = map(float, point)
                    observation.pose.orientation.w = 1.
                    observation.scale.x = observation.scale.y = observation.scale.z = .02
                    observation.color.a = 1.
                    observations.markers.append(observation)
                    diagnostic = PointStamped()
                    diagnostic.header.stamp = data.header.stamp
                    diagnostic.header.frame_id = frame_id
                    diagnostic.point.x = float((u-cx)*z/fx)
                    diagnostic.point.y = float((v-cy)*z/fy)
                    diagnostic.point.z = float(z)
                    self.position_publishers[kind].publish(diagnostic)
                    # Preserve the old optical-height diagnostic outputs. The mapper
                    # consumes ONLY the batched observations, never these legacy IDs.
                    if self.p('publish_legacy_position') and 0 < z <= self.p('max_range_m'):
                        legacy = PointStamped(); legacy.header.stamp = data.header.stamp
                        legacy.header.frame_id = frame_id
                        legacy.point.x, legacy.point.y, legacy.point.z = float((u-cx)*z/fx), float(kind), float(z)
                        self.point_publisher.publish(legacy)
                    self.get_logger().info('Detected %s range_source=%s' % (marker_type[kind], source),
                                           throttle_duration_sec=1.)
            for blob, colour in ((pink, 'pink'), (other, top if bottom == 'pink' else bottom)):
                cv2.rectangle(frame, (blob['x'], blob['y']), (blob['x']+blob['w'], blob['y']+blob['h']), DRAW_BGR[colour], 1)
            if count < CONFIRM_FRAMES:
                label += ' ?'
            self.draw_label(frame, label, x, y, (0, 255, 255) if source == 'approx' else (0, 255, 0))
        self.screen_tracks = next_tracks
        # Empty batches also communicate loss of visual detections to the mapper.
        self.observation_publisher.publish(observations)
        if self.p('show_debug'):
            cv2.imshow('camera', frame); cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SeeMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
