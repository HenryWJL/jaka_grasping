cmake_minimum_required(VERSION 3.0.2)
project(handeye_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf2_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hand_to_eye_calib.py scripts/easy_hand_to_eye_calib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

