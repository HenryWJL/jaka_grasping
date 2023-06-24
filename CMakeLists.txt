cmake_minimum_required(VERSION 3.0.2)
project(jaka_grasping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  dh_gripper_msgs	
  tf2_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs dh_gripper_msgs tf2_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/object_location.py
    scripts/jaka_grasp.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
