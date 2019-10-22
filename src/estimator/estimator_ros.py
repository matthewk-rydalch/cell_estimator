#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def estimator():

	rospy.init_node('estimator', anonymous=True)
	# rospy.Subscriber('multirotor/imu', roscop_imu, self.roscopImuCallback)
	# rospy.Subscriber('rover/RelPosNED', rtk_relposned, self.rtkRelposnedCallback)
	rospy.Subscriber('cell/imu', String, callback)
	# rospy.Subscriber('cell/imu', cell_imu, self.cellImuCallback)
	# rospy.Subscriber('cell/RelPosNED', cell_RelPosNED, self.cellRelposnedCallback)
	
	# rospy.Publisher('est/cell/relPosNED', est_cell_relposned, self.estCellRelposnedCallback)
	
#

if __name__ == '__main__':
	estimator()
	rospy.spin()
#
