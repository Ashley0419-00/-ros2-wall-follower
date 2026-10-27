#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# Creates two-colour cylindrical landmarks to be used in a MarkerArray




import rclpy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

colours = {
	"pink":	 	((140,0,0), (170, 255, 255)),
	"blue":		((100,0,0), (130, 255, 255)),
	"green":	((40,0,0), (80, 255, 255)),
	"yellow":	((25,0,0), (32, 255, 255))
}


marker_type = [
	"yellow/pink",
	"green/pink",
	"blue/pink",
	"pink/yellow",
	"pink/green",
	"pink/blue"
]

marker_colour = [
	((1.0, 1.0, 0.0), (1.0, 0.56, 0.75)),
	((0.0, 1.0, 0.0), (1.0, 0.56, 0.75)),
	((0.0, 0.0, 1.0), (1.0, 0.56, 0.75)),
	((1.0, 0.56, 0.75), (1.0, 1.0, 0.0)),
	((1.0, 0.56, 0.75), (0.0, 1.0, 0.0)),
	((1.0, 0.56, 0.75), (0.0, 0.0, 1.0))
]

max_markers = len(marker_type)

class Landmark:

	def __init__(self, mtype, markers):
		self.mtype = mtype
		self.count = 0;
		self.sum_x = 0.0
		self.sum_y = 0.0
		self.top_marker = None
		self.bottom_marker = None
		self.markers = markers

	def toRow(self):
		if self.top_marker is not None:
			return [self.top_marker.pose.position.x, self.top_marker.pose.position.y, self.mtype]
		else:
			return []


	def update_position(self, new_point):
		if self.count == 0:
			self.add_marker()

		self.count += 1
		self.sum_x += new_point.x
		self.sum_y += new_point.y

		new_x = self.sum_x/self.count
		new_y = self.sum_y/self.count

		self.top_marker.pose.position.x = new_x
		self.top_marker.pose.position.y = new_y
		self.bottom_marker.pose.position.x = new_x
		self.bottom_marker.pose.position.y = new_y


	def add_marker(self):
		self.top_marker = self.make_half_marker(0)
		self.markers.append(self.top_marker)
		self.bottom_marker = self.make_half_marker(1)
		self.markers.append(self.bottom_marker)


	def make_half_marker(self, bottom):
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.ns = marker_type[self.mtype]
		marker.id = 2*self.mtype+bottom
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = marker_colour[self.mtype][bottom][0]
		marker.color.g = marker_colour[self.mtype][bottom][1]
		marker.color.b = marker_colour[self.mtype][bottom][2]
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.15 if bottom else 0.35
		return marker

