#!/usr/bin/env python3

from IPython.core.debugger import set_trace
import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime

def cell_ros():

	pub_imu = rospy.Publisher('cell/imu', imu, queue_size=1024) #I just used the queue size from latis
	pub_NED = rospy.Publisher('cell/NED', RelPos, queue_size=1024) 
	pub_lla = rospy.Publisher('cell/lla', PositionVelocityTime, queue_size=1024) 

	rospy.init_node('cell_ros', anonymous=True)

	#example of how to set up a subscriber can be seen in estimator_ros.py

	while not rospy.is_shutdown():

		imu_data = imu()
		NED_data = RelPos()
		lla_data = PositionVelocityTime()

		#quaternian
		imu_data.orientation.x = 1.0
		imu_data.orientation.y = 2.0
		imu_data.orientation.z = 3.0
		imu_data.orientation.w = 4.0

		imu_data.orientation_covariance = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 1.0]

		imu_data.angular_velocity.x = 1.0
		imu_data.angular_velocity.y = 3.0
		imu_data.angular_velocity.z = 2.0

		imu_data.angular_velocity_covariance = [2.0, 5.0, 5.0, 8.0, 5.0, 5.0, 5.0, 5.0, 1.0]

		imu_data.linear_acceleration.x = 2.0
		imu_data.linear_acceleration.y = 1.0
		imu_data.linear_acceleration.z = 3.0

		imu_data.linear_acceleration_covariance = [4.0, 5.0, 5.0, 5.0, 5.0, 5.0, 9.0, 0.0, 1.0]

		pub_imu.publish(imu_data)
		pub_NED.publish(NED_data)
		pub_lla.publish(lla_data)

if __name__ == '__main__':
	cell_ros()
		
