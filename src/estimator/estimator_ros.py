#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime


def imu_callback(data):
	#this line is simply to verify that messages are being subscribed to during development
	# rospy.loginfo(rospy.get_caller_id() + " orientation x %s", data.orientation.x)
	print("imu_callback \n")

def ned_callback(data):
	#this line is simply to verify that messages are being subscribed to during development
	print("ned_callback \n")

def lla_callback(data):
	#this line is simply to verify that messages are being subscribed to during development
	print("lla_callback \n")

def estimator():
	print('estimator test')
	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('imu', imu, imu_callback)
	rospy.Subscriber('NED', RelPos, ned_callback)
	rospy.Subscriber('lla', PositionVelocityTime, lla_callback)

	rospy.spin()
#

if __name__ == '__main__':
	estimator()
#
