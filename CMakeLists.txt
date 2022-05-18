cmake_minimum_required(VERSION 3.0.2)
project(dvrk_calib_hand_eye)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  LIBRARIES dvrk_calib_hand_eye
  CATKIN_DEPENDS rospy
)
