#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# ROS 2 program to subscribe to real-time streaming 
# video from TurtleBot3 Pi camera and find coloured landmarks.

# Colour segementation and connected components example added by Claude sSammut
	
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import math

from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import LaserScan # Laser scan message type
from sensor_msgs.msg import CameraInfo # Need to know camera frame
from geometry_msgs.msg import Point, PointStamped

import wall_follower.landmark
from wall_follower.landmark import marker_type


field_of_view_h = 62.2


class SeeMarker(Node):
	"""
	Create an ImageSubscriber class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('see_marker')

		self.range = 0

		# --- Problem 1: intrinsics state (populated by CameraInfo) ---
		self.fx = None
		self.fy = None
		self.cx = None
		self.cy = None
		self.marker_height_m = 0.10   # real vertical height of one pillar, metres

		self.camera_info_sub = self.create_subscription(
			CameraInfo,
			'/camera/camera_info',
			self.camera_info_callback,
			10)
			
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
			Image,
			'/camera/image_raw', 
			self.listener_callback, 
			10)
		self.subscription # prevent unused variable warning
			
		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		self.point_publisher = self.create_publisher(PointStamped, '/marker_position', 10)

		# --- Problem 3: temporal confirmation state ---
		self.confirm_frames = 3
		self._confirm = {}        # marker_index -> consecutive count
		self._last_pos = {}       # marker_index -> (x, y)
		self._prev_by_marker = {} # marker_index -> (p_h, p_d)

	def camera_info_callback(self, msg):
		"""Store pinhole intrinsics for physical distance/bearing estimation."""
		k = msg.k
		if k[0] > 0.0 and k[4] > 0.0:
			self.fx, self.fy = k[0], k[4]
			self.cx, self.cy = k[2], k[5]

	def listener_callback(self, data):
		"""
		Callback function.
		"""
		# Convert ROS Image message to OpenCV image
		current_frame = self.br.imgmsg_to_cv2(data, 'bgra8')

		# Convert BGR image to HSV
		hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

		# --- Problem 2: detect all colours once, then associate geometrically ---
		dets = {}
		for c in colours:
			b = segment(current_frame, hsv_frame, c,
				self.fx, self.fy, self.cx, self.cy, self.marker_height_m)
			if b is not None:
				dets[c] = b

		pink = dets.get("pink")
		if pink:
			(pink_x, pink_y, pink_h, p_d, p_a,
			 p_left, p_top, p_w, p_h) = pink

			for c in ("blue", "green", "yellow"):
				b = dets.get(c)
				if not b:
					continue
				(c_x, c_y, c_h, c_d, c_a,
				 c_left, c_top, c_w, c_hh) = b

				# Horizontal overlap of bounding boxes (replaces centroid hack)
				overlap = min(p_left + p_w, c_left + c_w) - max(p_left, c_left)
				min_w = min(p_w, c_w)
				if min_w <= 0 or overlap < 0.6 * min_w:
					continue

				# Vertical gap must be small relative to blob height
				p_bottom = p_top + p_h
				c_bottom = c_top + c_hh
				if c_top < p_top:
					gap = p_top - c_bottom
					top_c, bottom_c = c, "pink"
				else:
					gap = c_top - p_bottom
					top_c, bottom_c = "pink", c
				if gap > 0.6 * max(p_h, c_hh):
					continue

				marker_at = PointStamped()
				marker_at.header.stamp = self.get_clock().now().to_msg()
				marker_at.header.frame_id = data.header.frame_id  # Problem 1
				marker_at.point.z = float(
					marker_type.index(top_c + '/' + bottom_c))

				x, y = polar_to_cartesian(c_d, c_a)
				marker_at.point.x = x
				marker_at.point.y = y
				midx = int(marker_at.point.z)

				# --- Problem 3: consistency and per-marker hysteresis ---
				prev = self._last_pos.get(midx)
				consistent = True
				if prev is not None:
					if math.hypot(x - prev[0], y - prev[1]) > 0.30:
						consistent = False

				prev_geom = self._prev_by_marker.get(midx)
				if prev_geom is not None:
					ph, pr = prev_geom
					if ph > 0.0 and abs(p_h - ph) > 0.40 * ph:
						consistent = False
					if pr > 0.0 and abs(p_d - pr) > 0.30 * pr:
						consistent = False
				self._prev_by_marker[midx] = (p_h, p_d)

				if consistent:
					self._confirm[midx] = self._confirm.get(midx, 0) + 1
				else:
					self._confirm[midx] = 1
				self._last_pos[midx] = (x, y)

				if self._confirm[midx] >= self.confirm_frames:
					self.point_publisher.publish(marker_at)

		# Display camera image
		cv2.imshow("camera", current_frame)	
		cv2.waitKey(1)


colours = {
	"pink":	 	((140, 80, 60), (170, 255, 255)),
	"blue":		((100, 80, 60), (130, 255, 255)),
	"green":	((40,  80, 60), (80,  255, 255)),
	"yellow":	((25,  80, 60), (32,  255, 255)),
}


def segment(current_frame, hsv_frame, colour,
            fx=None, fy=None, cx0=None, cy0=None, marker_height_m=0.10):
	(lower, upper) = colours[colour]
	mask = cv2.inRange(hsv_frame, lower, upper)

	# --- Problem 2: morphological cleanup ---
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	result = cv2.bitwise_and(current_frame, current_frame, mask=mask)
	blobs = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
	return get_stats(blobs, colour, fx, fy, cx0, cy0, marker_height_m)


def get_stats(blobs, colour, fx=None, fy=None, cx0=None, cy0=None,
              marker_height_m=0.10):
	(numLabels, labels, stats, centroids) = blobs
	if numLabels == 0:
		return None

	largest = 0
	rval = None

	for i in range(1, numLabels):
		x = stats[i, cv2.CC_STAT_LEFT]
		y = stats[i, cv2.CC_STAT_TOP]
		w = stats[i, cv2.CC_STAT_WIDTH]
		h = stats[i, cv2.CC_STAT_HEIGHT]
		area = stats[i, cv2.CC_STAT_AREA]
		(cx, cy) = centroids[i]

		# --- Problem 2: geometric blob validation ---
		bbox_area = float(w * h) if w > 0 and h > 0 else 0.0
		solidity = area / bbox_area if bbox_area > 0 else 0.0
		aspect = (h / w) if w > 0 else 0.0
		if area < 150 or aspect < 0.4 or aspect > 4.0 or solidity < 0.5:
			continue

		if area > largest:
			largest = area

			# --- Problem 1: intrinsics-based geometry ---
			if fx and fy and cx0 is not None and cy0 is not None:
				distance = (fy * marker_height_m / h) if h > 0 else 0.0
				bearing_rad = math.atan2(cx - cx0, fx)
				angle = math.degrees(bearing_rad)
				if angle < 0:
					angle += 360.0
			else:
				# Fallback to empirical model if intrinsics not yet received
				distance = 35.772 * pow(h, -0.859)
				centre = 320
				angle = (centre - cx) * field_of_view_h / 640
				if angle < 0:
					angle += 360.0

			rval = (cx, cy, h, distance, angle, x, y, w, h)

	return rval


def polar_to_cartesian(distance, angle):
	angle_rad = math.radians(angle)
	x = distance * math.cos(angle_rad)
	y = -distance * math.sin(angle_rad) # Added minus sign
	return x, y

def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node
	see_marker = SeeMarker()
	
	# Spin the node so the callback function is called.
	try:
		rclpy.spin(see_marker)
	except KeyboardInterrupt:
		exit()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	see_marker.destroy_node()
	
	# Shutdown the ROS client library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
