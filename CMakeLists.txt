cmake_minimum_required(VERSION 2.8.3)
project(cell_estimator)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  # cell_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs rospy
)

