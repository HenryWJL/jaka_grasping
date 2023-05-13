cmake_minimum_required(VERSION 3.0.2)
project(visual_grasp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf2_msgs
  dh_gripper_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/object_location.py scripts/jaka_grasp.py scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
