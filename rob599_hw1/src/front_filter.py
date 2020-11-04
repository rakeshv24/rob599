#!/usr/bin/env python


# ROB 599 Homework 1 Reference Solution
#
# front_filter.py
#
# Bill Smart
#
# This filters LaserScan messages, removing all of the points that are not in front of the robot.  This is a simple version
# that uses the coordinate frame of the laser scan.  This implicitly assumes that the x-axis of the laser is aligned with
# the front of the robot.


import rospy
import sys

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan

from math import sin
from numpy import linspace, inf


class FrontFilter:
	"""
	A class that implements a LaserScan filter that removes all of the points that are not in front of the robot.
	"""
	def __init__(self, width):
		"""
		:param self: The self reference.
		:param width: The width of the robot.
		"""

		# We're going to assume that the robot is pointing up the x-axis, so that any points with y coordinates further
		# than half a robot width from the axis are not in front of the robot.
		self.extent = width / 2.0

		# A subscriber and a publisher.  We're going to give these generic names, and deal with changing them at runtine
		# by using topic remapping.
		self.sub = rospy.Subscriber('input_scan', LaserScan, self.filter_scan, queue_size=10)
		self.pub = rospy.Publisher('output_scan', LaserScan, queue_size=10)

	def filter_scan(self, msg):
		"""
		:param self: Self reference.
		:param msg: LaserScan message.
		"""

		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams.
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Work out the y coordinates of the ranges.
		points = [r * sin(theta) for r,theta in zip(msg.ranges, angles)]

		# If we're close to the x axis, keep the range, otherwise use inf, which means "no return".
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

		# Substitute in the new ranges in the original message, and republish it.
		msg.ranges = new_ranges
		self.pub.publish(msg)		


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('front_filter', argv=sys.argv)

	# Start the filter
	filter = FrontFilter(1)

	# Hand control over to ROS.
	rospy.spin()
