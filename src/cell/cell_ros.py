#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from imu.msg import geometry_msgs

def cell_ros():

	# rospy.Subscriber('multirotor/imu', roscop_imu, self.roscopImuCallback)
	# rospy.Subscriber('rover/RelPosNED', rtk_relposned, self.rtkRelposnedCallback)
	# rospy.Subscriber('cell/imu', cell_imu, self.cellImuCallback)
	# rospy.Subscriber('cell/RelPosNED', cell_RelPosNED, self.cellRelposnedCallback)
	
	pub = rospy.Publisher('cell/imu', imu, queue_size=10) #can include a queue size.  See ros docs for publishers
	rospy.init_node('cell_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():

		std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance
		orientation = imu.orientation()
		orientation.x = 1
		orientation.y = 2
		orientation.z = 3
		orientation.w = 4
		
		imu_new.orientation


		rate.sleep()
		rospy.spin()

		# ###michael, this is just to test the ros publisher.  If you do use this, comment out rospy.spin() Otherwise keep what is below commented out
		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)
		# rate.sleep()

if __name__ == '__main__':
	cell_ros()
		
