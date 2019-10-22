#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def cell():

	# rospy.Subscriber('multirotor/imu', roscop_imu, self.roscopImuCallback)
	# rospy.Subscriber('rover/RelPosNED', rtk_relposned, self.rtkRelposnedCallback)
	# rospy.Subscriber('cell/imu', cell_imu, self.cellImuCallback)
	# rospy.Subscriber('cell/RelPosNED', cell_RelPosNED, self.cellRelposnedCallback)
	
	pub = rospy.Publisher('cell/imu', String) #can include a queue size.  See ros docs for publishers
	rospy.init_node('cell', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		cell()
	except rospy.ROSInterruptException:
		pass
		