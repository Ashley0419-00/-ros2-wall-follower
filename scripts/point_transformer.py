#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# ROS 2 program to subscribe to update a MarkerArray subscribing to 
# PointStamed topic, assuming it's published by a vision node.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import csv
import math

import rclpy.duration


import wall_follower.landmark
from wall_follower.landmark import marker_type, max_markers, Landmark


class PointTransformer(Node):

	def __init__(self):
		super().__init__('point_transformer')
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.point_subscriber = self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10)
		self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

		self.marker_position = []
		self.marker_array = MarkerArray()			
		self.marker_array.markers = []

		for i in range(max_markers):
			self.marker_position.append(Landmark(i, self.marker_array.markers))

		self._filt_pos = {}       # marker index -> (x, y)
		self._filt_alpha = 0.3
		self._gate_radius = 0.50

	def saveLandmarks(self):
		with open('landmarks.csv', 'w', newline='') as csvfile:
			lm_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			for lm in self.marker_position:
				if lm.top_marker is not None:
					lm_writer.writerow(lm.toRow())


	def point_callback(self, msg):
		try:
			transform = self.tf_buffer.lookup_transform(
				'map',
				msg.header.frame_id,
				rclpy.time.Time(),
				timeout=rclpy.duration.Duration(seconds=0.1))
		except (tf2_ros.LookupException,
				tf2_ros.ConnectivityException,
				tf2_ros.ExtrapolationException) as e:
			self.get_logger().warn('Transform lookup failed: %s' % str(e))
			return

		which_marker = int(msg.point.z)
		if which_marker < 0 or which_marker >= max_markers:
			self.get_logger().warn('Invalid marker index %d' % which_marker)
			return
		m = marker_type[which_marker]
		msg.point.z = 0.0

		# Transform the point from camera_rgb_optical_frame to map frame
		map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

		# Print the transformed point in the map frame
#		self.get_logger().info(f'Mapped {m} marker to /map frame: x={map_point.point.x}, y={map_point.point.y}, z={map_point.point.z}')

		midx = which_marker
		x, y = map_point.point.x, map_point.point.y
		prev = self._filt_pos.get(midx)

		if prev is None:
			self._filt_pos[midx] = (x, y)
		else:
			jump = math.hypot(x - prev[0], y - prev[1])
			if jump > self._gate_radius:
				self.get_logger().warn(
					'Rejected outlier for marker %d (%.2f m)' % (midx, jump))
				return
			a = self._filt_alpha
			self._filt_pos[midx] = (
				a * x + (1.0 - a) * prev[0],
				a * y + (1.0 - a) * prev[1])

		fx, fy = self._filt_pos[midx]
		map_point.point.x = fx
		map_point.point.y = fy
		self.marker_position[which_marker].update_position(map_point.point)
		self.marker_publisher_.publish(self.marker_array)


def main(args=None):
	rclpy.init(args=args)
	node = PointTransformer()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.saveLandmarks()
		exit()

	rclpy.shutdown()

if __name__ == '__main__':
	main()

