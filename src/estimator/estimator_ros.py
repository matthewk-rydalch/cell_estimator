#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
from estimator import Estimator

def ros_estimator():
	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('imu', imu, est.imu_callback)
	rospy.Subscriber('NED', RelPos, est.ned_callback)
	rospy.Subscriber('lla', PositionVelocityTime, est.lla_callback)

	rospy.spin()
#

if __name__ == '__main__':
	est = Estimator()
	ros_estimator()
#
