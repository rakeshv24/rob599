#!/usr/bin/env python


# ROB 599 Homework 1 Reference Solution
#
# wall.py
#
# Bill Smart
#
# Calcuate the angle from the robot x-axis and the perpendicular from the nearest wall.  We're not going to do any
# wall detection, sothis number might not mean anything.


from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

from math import sin, cos, pi
from numpy import linspace, array, polyfit, isinf


def callback(msg):
	# Calculate the angles for the beams.
	angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

	# Transform them into contact points, for those ranges that are not inf.
	points = [(r * cos(theta), r * sin(theta)) for r,theta in zip(msg.ranges, angles) if not isinf(r)]

	# Split out the x and y points, and make them numpy arrays.
	x, y = zip(*points)
	x = array(x)
	y = array(y)

	# Fit a line to them.  Note that we're fitting x = my + c, which will let us use m directly.
	m,c = polyfit(y, x, 1)

	# Publish the angle.
	pub.publish(m)


if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('wall', argv=sys.argv)

	# Set up a subscriber and publisher.
	sub = rospy.Subscriber('scan', LaserScan, callback, queue_size=1)
	pub = rospy.Publisher('wall_angle', Float32, queue_size=10)

	# Give control over to ROS.
	rospy.spin()

