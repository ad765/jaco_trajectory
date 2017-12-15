cmake_minimum_required(VERSION 2.8.3)
project(jaco_trajectory)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  joint_state_controller
  robot_state_publisher
  roscpp
  rospy
  rqt_gui
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
