#!/usr/bin/env python


# ROB 599 Homework 1 Reference Solution
#
# front_filter.py
#
# Bill Smart
#
# A simple action client to test the functionality of the action server.


import rospy
import sys

import actionlib

from rob599_hw1.msg import DistanceAction, DistanceGoal


def feedback_callback(feedback):
	rospy.loginfo('Feedback: {0}'.format(feedback.error))


def active_callback():
	rospy.loginfo('Action active')


def done_callback(status, result):
	if status == actionlib.GoalStatus.SUCCEEDED:
		rospy.loginfo('Suceeded with result {0}'.format(result.final))
	else:
		rospy.loginfo('Failed with result {0}'.format(result.final))


if __name__ == '__main__':
	rospy.init_node('action_client', argv=sys.argv)

	client = actionlib.SimpleActionClient('approacher', DistanceAction)
	client.wait_for_server()

	goal = DistanceGoal(distance = 2.0)
	client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)
	client.wait_for_result()
