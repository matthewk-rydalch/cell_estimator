#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu


def callback(data):
	#this line is simply to verify that messages are being subscribed to during development
	rospy.loginfo(rospy.get_caller_id() + " orientation x %s", data.orientation.x)

def estimator():

	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('cell/imu', imu, callback)

	rospy.spin()
#

if __name__ == '__main__':
	estimator()
#
