#!/usr/bin/env python3


import rospy
import sys


def callback(event):
	rospy.loginfo('Callback fired: {0}'.format(event))


if __name__ == '__main__':
	rospy.init_node('timer', argv=sys.argv)

	rospy.Timer(rospy.Duration(1), callback)

	rospy.spin()
	