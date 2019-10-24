cmake_minimum_required(VERSION 2.8.3)
project(cell_estimator)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
	DIRECTORY message
	FILES
	imu.msg)

generate_messages(
	DEPENDENCIES
	std_msgs)

catkin_package(
   CATKIN_DEPENDS std_msgs rospy message_runtime
)

