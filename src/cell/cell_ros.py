#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from imu.msg import geometry_msgs

def cell_ros():

	# rospy.Subscriber('multirotor/imu', roscop_imu, self.roscopImuCallback)
	# rospy.Subscriber('rover/RelPosNED', rtk_relposned, self.rtkRelposnedCallback)
	# rospy.Subscriber('cell/imu', cell_imu, self.cellImuCallback)
	# rospy.Subscriber('cell/RelPosNED', cell_RelPosNED, self.cellRelposnedCallback)
	
	pub = rospy.Publisher('cell/imu', imu, queue_size=1024) #can include a queue size.  See ros docs for publishers
	rospy.init_node('cell_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():

		imu_data = imu()

		imu_data.var = 5.0

		pub.publish(imu_data)

		rate.sleep()
		rospy.spin()

		# ###michael, this is just to test the ros publisher.  If you do use this, comment out rospy.spin() Otherwise keep what is below commented out
		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)
		# rate.sleep()

if __name__ == '__main__':
	cell_ros()
		
