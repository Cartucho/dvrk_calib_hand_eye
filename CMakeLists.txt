cmake_minimum_required(VERSION 3.0.2)
project(dvrk_calib_arms_to_camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  LIBRARIES dvrk_calib_arms_to_camera
  CATKIN_DEPENDS rospy
)
