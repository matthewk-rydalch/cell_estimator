#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def cell_ros():
	
	pub = rospy.Publisher('cell/imu', String, queue_size=10) #can include a queue size.  See ros docs for publishers
	rospy.init_node('cell_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():

		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	cell_ros()
		
