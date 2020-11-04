#!/usr/bin/env python


# ROB 599 Homework 1 Reference Solution
#
# front_filter_better.py
#
# Bill Smart
#
# This filters LaserScan messages, removing all of the points that are not in front of the robot.  This is a more general
# version that uses the base frame of the robot, rather than the frame of the laser sensor.


import rospy
import sys

import tf

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan

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

	def filter_scan(self, msg):
		"""
		:param self: Self reference.
		:param msg: LaserScan message.
		"""

		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams.
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Turn the ranges into contact points.  We're going to use a PointStamped here, so that we can use tf to transform
		# the points to the base frame of the robot.
		points = [PointStamped(header=msg.header, point=Point(r * cos(theta), r * sin(theta), 0)) for r,theta in zip(msg.ranges, angles)]

		# Build a list of transformed points.  These will be in the base frame of the robot.
		points = [self.listener.transformPoint(self.frame, p) for p in points]

		# Figure out the ranges as before.  If we're close to the x-axis (in the new frame), keep the range, otherwise set it
		# to inf for "no return".
		new_ranges = [r if abs(p.point.y) < self.extent and p.point.x > 0 else inf for r,p in zip(msg.ranges, points)]

		# Swap the new ranges into the LaserScan message and republish it.
		msg.ranges = new_ranges
		self.pub.publish(msg)		


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('front_filter', argv=sys.argv)

	# Set up the filter.
	filter = FrontFilter(1, 'base_link')

	# Give control over to ROS.
	rospy.spin()
