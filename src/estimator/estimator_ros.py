#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
from ublox.msg import PositionVelocityTime
from ublox.msg import RelPos
from estimator import Estimator

def ros_estimator():
	rospy.init_node('estimator', anonymous=True)
	rospy.Subscriber('cell/imu', imu, est.imu_callback)
	rospy.Subscriber('cell/NED', RelPos, est.ned_callback)
	rospy.Subscriber('cell/lla', PositionVelocityTime, est.lla_callback)
	rospy.Subscriber('rover/RelPos', RelPos, est.rover_RelPos_callback)

	rospy.spin()
#

if __name__ == '__main__':
	est = Estimator()
	ros_estimator()
#
