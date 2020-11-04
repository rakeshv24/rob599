#!/usr/bin/env python

# ROB 599 Homework 1 Reference Solution
#
# approach.py
#
# Bill Smart
#
# A node that approaches a wall to a given distance.  This shows how to integrate multiple interfaces
# to the wall approacher (startup, service, action) into the same node.


import rospy
import sys

import actionlib

# We define these messages for the service and action interfaces.  Note that we import from .msg
# even though there is no .msg source directory.  catkin_make will make the messages and put them
# in the right place (in the devel tree of the workspace) automatically.
from rob599_hw1.srv import SetDistance, SetDistanceResponse
from rob599_hw1.msg import DistanceAction, DistanceGoal, DistanceFeedback, DistanceResult

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

from math import tanh, sin, cos


class Approacher:
	"""
	A class to encapsulate approaching behavior for a mobile robot.  The robot will try to drive along the x-axis until
	the shortest laser range is at a given distance.
	"""
	def __init__(self, distance=1, max_speed=1, aggressiveness=1):
		"""
		:param distance: The stopping distance, in meters.
		:param max_speed: Set the absolute value of the maximum speed.
		:param aggressiveness: How aggressively to speed up and slow down.
		"""

		# Save the stopping distance.
		self.distance = distance
		self.max_speed = max_speed
		self.aggressiveness = aggressiveness

		# Set up a publisher and a subscriber.  We're going to call the subscriber "scan", and deal with the actual
		# topic name using remapping at run time.  For the publisher, we're going to use cmd_vel, which is a common
		# choice.  Again, we can deal with this at runtime if we have to.
		self.sub = rospy.Subscriber('scan', LaserScan, self.set_speed, queue_size=10)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		# Set up a service client to allow us to change the distance with a service call.
		self.service = rospy.Service('set_distance', SetDistance, self.set_distance)

		# Set up a simple action server to handle action requests.  As usual, we create it in stopped mode, and then
		# explicitly call start on it.  We're also going to keep a helper variable around to indicate whether or not
		# we have an action that's currently active.  We're going to use this below, in the LaserScan callback.
		self.action = actionlib.SimpleActionServer('approacher', DistanceAction, self.distance_action, False)
		self.action.start()
		self.action_active = False

		# Alocate a Twist to use, and set everything to zero.  We're going to do this here, to save some time in
		# the callback.
		self.twist = Twist()
		self.twist.linear.x = 0.0
		self.twist.linear.y = 0.0
		self.twist.linear.z = 0.0
		self.twist.angular.x = 0.0
		self.twist.angular.y = 0.0
		self.twist.angular.z = 0.0

		# Set up a marker to show the distance
		self.marker_pub = rospy.Publisher('approach_distance', Marker, queue_size=1)
		self.marker = Marker()
		self.marker.id = 0
		self.marker.type = self.marker.LINE_STRIP
		self.marker.action = self.marker.ADD
		self.marker.scale.x = 0.1
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0
		self.marker.color.a = 1.0

		# Set the initial line segment to be from the origin to the origin.
		self.marker.points.append(Point(0, 0, 0))
		self.marker.points.append(Point(0, 0, 0))

		# We'll use another marker for the text that says how far we are from the wall.  Note that the id of 
		# this marker is different from the one above.  This will allow us to publish them both on the same topic
		# and have them both show up.
		self.label = Marker()
		self.label.id = 1
		self.label.type = self.marker.TEXT_VIEW_FACING
		self.label.action = self.marker.ADD
		self.label.scale.z = 1
		self.label.color.r = 1.0
		self.label.color.g = 0.0
		self.label.color.b = 0.0
		self.label.color.a = 1.0
		self.label.pose.position.x = 0
		self.label.pose.position.y = 0
		self.label.pose.position.z = -1.5
		self.label.pose.orientation.x = 0
		self.label.pose.orientation.y = 0
		self.label.pose.orientation.z = 0
		self.label.pose.orientation.w = 1

	def set_speed(self, msg):
		"""
		:param self: Self reference.
		:param msg: The incoming message.

		This is the callback that fires when a LaserScan message comes in.
		"""

		# Calculate how far off we are from where we want to be.
		error = min(msg.ranges) - self.distance

		# If the action is active, then check to see if we're close and send feedback.
		if self.action_active:
			# Publish feedback.  This will work, since we will only get here if an action call has been previously made.
			self.action.publish_feedback(DistanceFeedback(error))

			# If we're close enough, save the final error value, and set action_active to False.  This will cause the 
			# action server to finish, and declare the action to be successful.
			if error < 0.01:
				self.action_final = error
				self.action_active = False	

		# Publish a marker that shows the shortest laser.  We're going to use the same header as the incoming laser message,
		# which will give us the right timestamp and frame ID.  We're going to leave the first point as the origin, and then
		# set the second point to be the closest laser contact point (in the laser's own frame, which we picked up from 
		# copying the header information).
		i = msg.ranges.index(min(msg.ranges))
		theta = i * msg.angle_increment + msg.angle_min
		self.marker.header = msg.header
		self.marker.points[1].x = msg.ranges[i] * cos(theta)
		self.marker.points[1].y = msg.ranges[i] * sin(theta)
		self.marker_pub.publish(self.marker)

		# Set up the distance label.  We're going to limit this to a fixed number of decimal places, to make it more readable.
		self.label.header = msg.header
		self.label.text = '{0:.2f}'.format(min(msg.ranges))
		self.marker_pub.publish(self.label)

		# Set the speed according to a tanh function.  There's nothing special about tanh, but it gives a nice smooth mapping
		# from distance to speed, and asymptotes at +/- 1.  Multiply this by the max speed.
		self.twist.linear.x = tanh(error * self.aggressiveness) * self.max_speed

		# Send some information to the info log channel, so that we know what's happening.
		rospy.loginfo('Error {0}: velocity {1}'.format(error, self.twist.linear.x))

		# Publish the velocity command.
		self.pub.publish(self.twist)

	def set_distance(self, msg):
		"""
		:param self: Self reference.
		:param msg: The incoming message.

		This is the callback used by the service to set the approach distance.  It does a basic check so that it does not
		set negative distances.
		"""

		# Reject negative distances, otherwise, just set the distance in the class instance variable.  This is playing
		# a bit fast and loose with writing across threads but, on the timescales we're dealing with, it shouldn't cause
		# a significant problem.
		if msg.distance > 0:
			rospy.loginfo('Successfully set distance to {0}'.format(msg.distance))
			self.distance = msg.distance
			return SetDistanceResponse(True)
		else:
			return SetDistanceResponse(False)

	def distance_action(self, goal):
		"""
		:param self: Self reference.
		:param goal: The incoming action goal.


		This function implements the functionality of the action callback, based on the existing mechanisms in the node.
		"""

		# Use the service callback to set the goal distance.  We can do this because of the polymorphism of Python.  Although
		# the service callback expects a service message, we can use the action goal message as a parameter because it also
		# has a distance attribute.  This would be harder in a strongly-typed language.
		self.set_distance(goal)

		# Set a flag to indicate that the action is running.  We're going to use this in the LaserScan callback to determine
		# if we should be publishing feedback.
		self.action_active = True

		# We're going to loop, waiting for the action_active variable to change to False.  This will be done in the LaserScan
		# callback.  Rather than busy waiting here (and consuming all of the CPU), we set a rate to check the variable.
		rate = rospy.Rate(10)
		while self.action_active:
			rate.sleep()

		# Once we get to the goal, then we're going to call the action a success.  This is a bit simplistic, but that's good
		# enough for this example.
		self.action.set_succeeded(DistanceResult(self.action_final))


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('approach', argv=sys.argv)

	# Start the approacher functionality.
	approacher = Approacher(2)

	# Give control over to ROS.
	rospy.spin()
