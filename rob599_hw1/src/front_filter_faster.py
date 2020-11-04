#!/usr/bin/env python


# ROB 599 Homework 1 Reference Solution
#
# front_filter_faster.py
#
# Bill Smart
#
# This filters LaserScan messages, removing all of the points that are not in front of the robot.  This is a more general
# version that uses the base frame of the robot, rather than the frame of the laser sensor.  It also addresses some of the
# speed issues of the other implementation.


import rospy
import sys

import tf

from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud

from math import sin, cos
from numpy import linspace, inf


class FrontFilter:
	"""
	A class that implements a LaserScan filter that removes all of the points that are not in front of the robot.
	"""
	def __init__(self, width, frame):
		"""
		:param self: The self reference.
		:param width: The width of the robot.
		:param frame: The base coordinate frame of the robot.
		"""

		# We're going to assume that the robot is pointing up the x-axis, so that any points with y coordinates further
		# than half a robot width from the axis are not in front of the robot.
		self.extent = width / 2.0
		self.frame = frame

		# A subscriber and a publisher.  We're going to give these generic names, and deal with changing them at runtine
		# by using topic remapping.
		self.sub = rospy.Subscriber('input_scan', LaserScan, self.filter_scan, queue_size=1)
		self.pub = rospy.Publisher('output_scan', LaserScan, queue_size=10)

		# Set up a tf listener, so that we can do the frame transforms.
		self.listener = tf.TransformListener()

		# Make a PointCloud, which we're going use for the transform.  We'll do this once to avoid the creation/destruction
		# overhead in the callback.
		self.cloud = PointCloud()

	def filter_scan(self, msg):
		"""
		:param self: Self reference.
		:param msg: LaserScan message.
		"""

		# Set the header of the point cloud to be the same as the incoming LaserScan
		self.cloud.header = msg.header

		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams.
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Fill in the laser contact points into the point cloud.
		self.cloud.points = [Point32(r * cos(theta), r * sin(theta), 0) for r,theta in zip(msg.ranges, angles)]

		# Do the coordinate transform all in one.
		self.cloud = self.listener.transformPointCloud(self.frame, self.cloud)

		# Filter out the points that are not directly in front of the robot, swap into the original LaserScan, and then
		# republish it.
		new_ranges = [r if abs(p.y) < self.extent and p.x > 0 else inf for r,p in zip(msg.ranges, self.cloud.points)]
		msg.ranges = new_ranges
		self.pub.publish(msg)		


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('front_filter', argv=sys.argv)

	# Set up the filter, with the base frame of the robot.
	filter = FrontFilter(1, 'base_link')

	# Give control over to ROS.
	rospy.spin()
