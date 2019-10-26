#!/usr/bin/env python3

from IPython.core.debugger import set_trace
import rospy
from std_msgs.msg import String
from cell_estimator.msg import imu

def cell_ros():

	pub = rospy.Publisher('cell/imu', imu, queue_size=1024) #can include a queue size.  See ros docs for publishers
	rospy.init_node('cell_ros', anonymous=True)

	#example of how to set up a subscriber can be seen in estimator_ros.py

	while not rospy.is_shutdown():

		imu_data = imu()

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

		pub.publish(imu_data)

if __name__ == '__main__':
	cell_ros()
		
